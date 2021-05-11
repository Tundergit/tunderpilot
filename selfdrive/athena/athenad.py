#!/usr/bin/env python3
import base64
import hashlib
import io
import json
import os
import sys
import queue
import random
import select
import socket
import threading
import time
from collections import namedtuple
from functools import partial
from typing import Any

import requests
from jsonrpc import JSONRPCResponseManager, dispatcher
from websocket import ABNF, WebSocketTimeoutException, create_connection

import cereal.messaging as messaging
from cereal.services import service_list
from common.api import Api
from common.basedir import PERSIST
from common.params import Params
from common.realtime import sec_since_boot
from selfdrive.hardware import HARDWARE, PC
from selfdrive.loggerd.config import ROOT
from selfdrive.loggerd.xattr_cache import getxattr, setxattr
from selfdrive.swaglog import cloudlog, SWAGLOG_DIR
import selfdrive.crash as crash
from selfdrive.version import dirty, origin, branch, commit

ATHENA_HOST = os.getenv('ATHENA_HOST', 'wss://athena.comma.ai')
HANDLER_THREADS = int(os.getenv('HANDLER_THREADS', "4"))
LOCAL_PORT_WHITELIST = set([8022])

LOG_ATTR_NAME = 'user.upload'
LOG_ATTR_VALUE_MAX_UNIX_TIME = int.to_bytes(2147483647, 4, sys.byteorder)

dispatcher["echo"] = lambda s: s
recv_queue: Any = queue.Queue()
send_queue: Any = queue.Queue()
upload_queue: Any = queue.Queue()
log_send_queue: Any = queue.Queue()
log_recv_queue: Any = queue.Queue()
cancelled_uploads: Any = set()
UploadItem = namedtuple('UploadItem', ['path', 'url', 'headers', 'created_at', 'id'])


def handle_long_poll(ws):
  end_event = threading.Event()

  threads = [
    threading.Thread(target=ws_recv, args=(ws, end_event)),
    threading.Thread(target=ws_send, args=(ws, end_event)),
    threading.Thread(target=upload_handler, args=(end_event,)),
    threading.Thread(target=log_handler, args=(end_event,)),
  ] + [
    threading.Thread(target=jsonrpc_handler, args=(end_event,))
    for x in range(HANDLER_THREADS)
  ]

  for thread in threads:
    thread.start()
  try:
    while not end_event.is_set():
      time.sleep(0.1)
  except (KeyboardInterrupt, SystemExit):
    end_event.set()
    raise
  finally:
    for thread in threads:
      thread.join()

def jsonrpc_handler(end_event):
  dispatcher["startLocalProxy"] = partial(startLocalProxy, end_event)
  while not end_event.is_set():
    try:
      data = recv_queue.get(timeout=1)
      if "method" in data:
        response = JSONRPCResponseManager.handle(data, dispatcher)
        send_queue.put_nowait(response.json)
      elif "result" in data and "id" in data:
        log_recv_queue.put_nowait(data)
      else:
        raise Exception("not a valid request or response")
    except queue.Empty:
      pass
    except Exception as e:
      cloudlog.exception("athena jsonrpc handler failed")
      send_queue.put_nowait(json.dumps({"error": str(e)}))


def upload_handler(end_event):
  while not end_event.is_set():
    try:
      item = upload_queue.get(timeout=1)
      if item.id in cancelled_uploads:
        cancelled_uploads.remove(item.id)
        continue
      _do_upload(item)
    except queue.Empty:
      pass
    except Exception:
      cloudlog.exception("athena.upload_handler.exception")


def _do_upload(upload_item):
  with open(upload_item.path, "rb") as f:
    size = os.fstat(f.fileno()).st_size
    return requests.put(upload_item.url,
                        data=f,
                        headers={**upload_item.headers, 'Content-Length': str(size)},
                        timeout=10)


# security: user should be able to request any message from their car
@dispatcher.add_method
def getMessage(service=None, timeout=1000):
  if service is None or service not in service_list:
    raise Exception("invalid service")

  socket = messaging.sub_sock(service, timeout=timeout)
  ret = messaging.recv_one(socket)

  if ret is None:
    raise TimeoutError

  return ret.to_dict()


@dispatcher.add_method
def listDataDirectory():
  files = [os.path.relpath(os.path.join(dp, f), ROOT) for dp, dn, fn in os.walk(ROOT) for f in fn]
  return files


@dispatcher.add_method
def reboot():
  sock = messaging.sub_sock("deviceState", timeout=1000)
  ret = messaging.recv_one(sock)
  if ret is None or ret.deviceState.started:
    raise Exception("Reboot unavailable")

  def do_reboot():
    time.sleep(2)
    HARDWARE.reboot()

  threading.Thread(target=do_reboot).start()

  return {"success": 1}


@dispatcher.add_method
def uploadFileToUrl(fn, url, headers):
  if len(fn) == 0 or fn[0] == '/' or '..' in fn:
    return 500
  path = os.path.join(ROOT, fn)
  if not os.path.exists(path):
    return 404

  item = UploadItem(path=path, url=url, headers=headers, created_at=int(time.time() * 1000), id=None)
  upload_id = hashlib.sha1(str(item).encode()).hexdigest()
  item = item._replace(id=upload_id)

  upload_queue.put_nowait(item)

  return {"enqueued": 1, "item": item._asdict()}


@dispatcher.add_method
def listUploadQueue():
  return [item._asdict() for item in list(upload_queue.queue)]


@dispatcher.add_method
def cancelUpload(upload_id):
  upload_ids = set(item.id for item in list(upload_queue.queue))
  if upload_id not in upload_ids:
    return 404

  cancelled_uploads.add(upload_id)
  return {"success": 1}


def startLocalProxy(global_end_event, remote_ws_uri, local_port):
  try:
    if local_port not in LOCAL_PORT_WHITELIST:
      raise Exception("Requested local port not whitelisted")

    params = Params()
    dongle_id = params.get("DongleId").decode('utf8')
    identity_token = Api(dongle_id).get_token()
    ws = create_connection(remote_ws_uri,
                           cookie="jwt=" + identity_token,
                           enable_multithread=True)

    ssock, csock = socket.socketpair()
    local_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    local_sock.connect(('127.0.0.1', local_port))
    local_sock.setblocking(0)

    proxy_end_event = threading.Event()
    threads = [
      threading.Thread(target=ws_proxy_recv, args=(ws, local_sock, ssock, proxy_end_event, global_end_event)),
      threading.Thread(target=ws_proxy_send, args=(ws, local_sock, csock, proxy_end_event))
    ]
    for thread in threads:
      thread.start()

    return {"success": 1}
  except Exception as e:
    cloudlog.exception("athenad.startLocalProxy.exception")
    raise e


@dispatcher.add_method
def getPublicKey():
  if not os.path.isfile(PERSIST + '/comma/id_rsa.pub'):
    return None

  with open(PERSIST + '/comma/id_rsa.pub', 'r') as f:
    return f.read()


@dispatcher.add_method
def getSshAuthorizedKeys():
  return Params().get("GithubSshKeys", encoding='utf8') or ''


@dispatcher.add_method
def getSimInfo():
  return HARDWARE.get_sim_info()


@dispatcher.add_method
def getNetworkType():
  return HARDWARE.get_network_type()


@dispatcher.add_method
def takeSnapshot():
  from selfdrive.camerad.snapshot.snapshot import snapshot, jpeg_write
  ret = snapshot()
  if ret is not None:
    def b64jpeg(x):
      if x is not None:
        f = io.BytesIO()
        jpeg_write(f, x)
        return base64.b64encode(f.getvalue()).decode("utf-8")
      else:
        return None
    return {'jpegBack': b64jpeg(ret[0]),
            'jpegFront': b64jpeg(ret[1])}
  else:
    raise Exception("not available while camerad is started")


def get_logs_to_send_sorted():
  # TODO: scan once then use inotify to detect file creation/deletion
  curr_time = int(time.time())
  logs = []
  for log_entry in os.listdir(SWAGLOG_DIR):
    log_path = os.path.join(SWAGLOG_DIR, log_entry)
    try:
      time_sent = int.from_bytes(getxattr(log_path, LOG_ATTR_NAME), sys.byteorder)
    except (ValueError, TypeError):
      time_sent = 0
    # assume send failed and we lost the response if sent more than one hour ago
    if not time_sent or curr_time - time_sent > 3600:
      logs.append(log_entry)
  # return logs in order they should be sent
  # excluding most recent (active) log file
  return sorted(logs[:-1])


def log_handler(end_event):
  if PC:
    return

  log_files = []
  last_scan = 0
  log_retries = 0
  while not end_event.is_set():
    try:
      try:
        result = json.loads(log_recv_queue.get(timeout=1))
        log_success = result.get("success")
        log_entry = result.get("id")
        log_path = os.path.join(SWAGLOG_DIR, log_entry)
        if log_entry and log_success:
          try:
            setxattr(log_path, LOG_ATTR_NAME, LOG_ATTR_VALUE_MAX_UNIX_TIME)
          except OSError:
            pass # file could be deleted by log rotation
      except queue.Empty:
        pass

      curr_scan = sec_since_boot()
      if curr_scan - last_scan > 10:
        log_files = get_logs_to_send_sorted()
        last_scan = curr_scan

      # never send last log file because it is the active log
      # and only send one log file at a time (most recent first)
      if not len(log_files) or not log_send_queue.empty():
        continue

      log_entry = log_files.pop()
      try:
        curr_time = int(time.time())
        log_path = os.path.join(SWAGLOG_DIR, log_entry)
        setxattr(log_path, LOG_ATTR_NAME, int.to_bytes(curr_time, 4, sys.byteorder))
        with open(log_path, "r") as f:
          jsonrpc = {
            "method": "forwardLogs",
            "params": {
              "logs": f.read()
            },
            "jsonrpc": "2.0",
            "id": log_entry
          }
          log_send_queue.put_nowait(json.dumps(jsonrpc))
      except OSError:
        pass # file could be deleted by log rotation
      log_retries = 0
    except Exception:
      cloudlog.exception("athena.log_handler.exception")
      log_retries += 1

    if log_retries != 0:
      time.sleep(backoff(log_retries))


def ws_proxy_recv(ws, local_sock, ssock, end_event, global_end_event):
  while not (end_event.is_set() or global_end_event.is_set()):
    try:
      data = ws.recv()
      local_sock.sendall(data)
    except WebSocketTimeoutException:
      pass
    except Exception:
      cloudlog.exception("athenad.ws_proxy_recv.exception")
      break

  ssock.close()
  local_sock.close()
  end_event.set()


def ws_proxy_send(ws, local_sock, signal_sock, end_event):
  while not end_event.is_set():
    try:
      r, _, _ = select.select((local_sock, signal_sock), (), ())
      if r:
        if r[0].fileno() == signal_sock.fileno():
          # got end signal from ws_proxy_recv
          end_event.set()
          break
        data = local_sock.recv(4096)
        if not data:
          # local_sock is dead
          end_event.set()
          break

        ws.send(data, ABNF.OPCODE_BINARY)
    except Exception:
      cloudlog.exception("athenad.ws_proxy_send.exception")
      end_event.set()

  signal_sock.close()


def ws_recv(ws, end_event):
  while not end_event.is_set():
    try:
      opcode, data = ws.recv_data(control_frame=True)
      if opcode in (ABNF.OPCODE_TEXT, ABNF.OPCODE_BINARY):
        if opcode == ABNF.OPCODE_TEXT:
          data = data.decode("utf-8")
        recv_queue.put_nowait(data)
      elif opcode == ABNF.OPCODE_PING:
        Params().put("LastAthenaPingTime", str(int(sec_since_boot() * 1e9)))
    except WebSocketTimeoutException:
      pass
    except Exception:
      cloudlog.exception("athenad.ws_recv.exception")
      end_event.set()


def ws_send(ws, end_event):
  while not end_event.is_set():
    try:
      try:
        data = send_queue.get_nowait()
      except queue.Empty:
        data = log_send_queue.get(timeout=1)
      ws.send(data)
    except queue.Empty:
      pass
    except Exception:
      cloudlog.exception("athenad.ws_send.exception")
      end_event.set()


def backoff(retries):
  return random.randrange(0, min(128, int(2 ** retries)))


def main():
  params = Params()
  dongle_id = params.get("DongleId", encoding='utf-8')
  crash.init()
  crash.bind_user(id=dongle_id)
  crash.bind_extra(dirty=dirty, origin=origin, branch=branch, commit=commit,
                   device=HARDWARE.get_device_type())

  ws_uri = ATHENA_HOST + "/ws/v2/" + dongle_id

  api = Api(dongle_id)

  conn_retries = 0
  while 1:
    try:
      ws = create_connection(ws_uri,
                             cookie="jwt=" + api.get_token(),
                             enable_multithread=True)
      cloudlog.event("athenad.main.connected_ws", ws_uri=ws_uri)
      ws.settimeout(1)
      conn_retries = 0
      handle_long_poll(ws)
    except (KeyboardInterrupt, SystemExit):
      break
    except Exception:
      crash.capture_exception()
      cloudlog.exception("athenad.main.exception")

      conn_retries += 1
      params.delete("LastAthenaPingTime")

    time.sleep(backoff(conn_retries))


if __name__ == "__main__":
  main()
