#include <ftw.h>
#include <pthread.h>
#include <sys/resource.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>

#include "cereal/messaging/messaging.h"
#include "cereal/services.h"
#include "cereal/visionipc/visionipc.h"
#include "cereal/visionipc/visionipc_client.h"
#include "selfdrive/camerad/cameras/camera_common.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"

#include "selfdrive/loggerd/encoder.h"
#include "selfdrive/loggerd/logger.h"
#if defined(QCOM) || defined(QCOM2)
#include "selfdrive/loggerd/omx_encoder.h"
#define Encoder OmxEncoder
#else
#include "selfdrive/loggerd/raw_logger.h"
#define Encoder RawLogger
#endif

namespace {

constexpr int MAIN_FPS = 20;
const int MAIN_BITRATE = Hardware::TICI() ? 10000000 : 5000000;
const int DCAM_BITRATE = Hardware::TICI() ? MAIN_BITRATE : 2500000;

#define NO_CAMERA_PATIENCE 500 // fall back to time-based rotation if all cameras are dead

const bool LOGGERD_TEST = getenv("LOGGERD_TEST");
const int SEGMENT_LENGTH = LOGGERD_TEST ? atoi(getenv("LOGGERD_SEGMENT_LENGTH")) : 60;

ExitHandler do_exit;

const LogCameraInfo cameras_logged[] = {
  {
    .type = RoadCam,
    .stream_type = VISION_STREAM_YUV_BACK,
    .filename = "fcamera.hevc",
    .frame_packet_name = "roadCameraState",
    .fps = MAIN_FPS,
    .bitrate = MAIN_BITRATE,
    .is_h265 = true,
    .downscale = false,
    .has_qcamera = true,
    .trigger_rotate = true,
    .enable = true,
  },
  {
    .type = DriverCam,
    .stream_type = VISION_STREAM_YUV_FRONT,
    .filename = "dcamera.hevc",
    .frame_packet_name = "driverCameraState",
    .fps = MAIN_FPS, // on EONs, more compressed this way
    .bitrate = DCAM_BITRATE,
    .is_h265 = true,
    .downscale = false,
    .has_qcamera = false,
    .trigger_rotate = Hardware::TICI(),
    .enable = !Hardware::PC() && Params().getBool("RecordFront"),
  },
  {
    .type = WideRoadCam,
    .stream_type = VISION_STREAM_YUV_WIDE,
    .filename = "ecamera.hevc",
    .frame_packet_name = "wideRoadCameraState",
    .fps = MAIN_FPS,
    .bitrate = MAIN_BITRATE,
    .is_h265 = true,
    .downscale = false,
    .has_qcamera = false,
    .trigger_rotate = true,
    .enable = Hardware::TICI(),
  },
};
const LogCameraInfo qcam_info = {
  .filename = "qcamera.ts",
  .fps = MAIN_FPS,
  .bitrate = 256000,
  .is_h265 = false,
  .downscale = true,
  .frame_width = Hardware::TICI() ? 526 : 480,
  .frame_height = Hardware::TICI() ? 330 : 360 // keep pixel count the same?
};

struct LoggerdState {
  Context *ctx;
  LoggerState logger = {};
  char segment_path[4096];
  std::mutex rotate_lock;
  std::condition_variable rotate_cv;
  std::atomic<int> rotate_segment;
  std::atomic<double> last_camera_seen_tms;
  std::atomic<int> waiting_rotate;
  int max_waiting = 0;
  double last_rotate_tms = 0.;

  // Sync logic for startup
  std::atomic<bool> encoders_synced;
  std::atomic<int> encoders_ready;
  std::atomic<uint32_t> start_frame_id;
  std::atomic<uint32_t> latest_frame_id;
};
LoggerdState s;

void encoder_thread(const LogCameraInfo &cam_info) {
  set_thread_name(cam_info.filename);

  int cnt = 0, cur_seg = -1;
  int encode_idx = 0;
  LoggerHandle *lh = NULL;
  std::vector<Encoder *> encoders;
  VisionIpcClient vipc_client = VisionIpcClient("camerad", cam_info.stream_type, false);

  bool ready = false;

  while (!do_exit) {
    if (!vipc_client.connect(false)) {
      util::sleep_for(5);
      continue;
    }

    // init encoders
    if (encoders.empty()) {
      VisionBuf buf_info = vipc_client.buffers[0];
      LOGD("encoder init %dx%d", buf_info.width, buf_info.height);

      // main encoder
      encoders.push_back(new Encoder(cam_info.filename, buf_info.width, buf_info.height,
                                     cam_info.fps, cam_info.bitrate, cam_info.is_h265, cam_info.downscale));
      // qcamera encoder
      if (cam_info.has_qcamera) {
        encoders.push_back(new Encoder(qcam_info.filename, qcam_info.frame_width, qcam_info.frame_height,
                                       qcam_info.fps, qcam_info.bitrate, qcam_info.is_h265, qcam_info.downscale));
      }
    }

    while (!do_exit) {
      VisionIpcBufExtra extra;
      VisionBuf* buf = vipc_client.recv(&extra);
      if (buf == nullptr) continue;

      if (cam_info.trigger_rotate && (s.max_waiting > 1)) {
        if (!ready) {
          LOGE("%s encoder ready", cam_info.filename);
          ++s.encoders_ready;
          ready = true;
        }

        if (!s.encoders_synced) {
          update_max_atomic(s.latest_frame_id, extra.frame_id);
          continue;
        } else {
          // Wait for all encoders to reach the same frame id
          if (extra.frame_id < s.start_frame_id) {
            LOGE("%s waiting for frame %d, cur %d", cam_info.filename, s.start_frame_id.load(), extra.frame_id);
            continue;
          }
        }
      }

      if (cam_info.trigger_rotate) {
        s.last_camera_seen_tms = millis_since_boot();
      }

      if (cam_info.trigger_rotate && (cnt >= SEGMENT_LENGTH * MAIN_FPS)) {
        // trigger rotate and wait logger rotated to new segment
        ++s.waiting_rotate;
        std::unique_lock lk(s.rotate_lock);
        s.rotate_cv.wait(lk, [&] { return s.rotate_segment > cur_seg || do_exit; });
      }
      if (do_exit) break;

      // rotate the encoder if the logger is on a newer segment
      if (s.rotate_segment > cur_seg) {
        cur_seg = s.rotate_segment;
        cnt = 0;

        LOGW("camera %d rotate encoder to %s", cam_info.type, s.segment_path);
        for (auto &e : encoders) {
          e->encoder_close();
          e->encoder_open(s.segment_path);
        }
        if (lh) {
          lh_close(lh);
        }
        lh = logger_get_handle(&s.logger);
      }

      // encode a frame
      for (int i = 0; i < encoders.size(); ++i) {
        int out_id = encoders[i]->encode_frame(buf->y, buf->u, buf->v,
                                               buf->width, buf->height, extra.timestamp_eof);
        
        if (out_id == -1) {
          LOGE("Failed to encode frame. frame_id: %d encode_id: %d", extra.frame_id, encode_idx);
        }

        // publish encode index
        if (i == 0 && out_id != -1) {
          MessageBuilder msg;
          // this is really ugly
          auto eidx = cam_info.type == DriverCam ? msg.initEvent().initDriverEncodeIdx() :
                     (cam_info.type == WideRoadCam ? msg.initEvent().initWideRoadEncodeIdx() : msg.initEvent().initRoadEncodeIdx());
          eidx.setFrameId(extra.frame_id);
          eidx.setTimestampSof(extra.timestamp_sof);
          eidx.setTimestampEof(extra.timestamp_eof);
          if (Hardware::TICI()) {
            eidx.setType(cereal::EncodeIndex::Type::FULL_H_E_V_C);
          } else {
            eidx.setType(cam_info.type == DriverCam ? cereal::EncodeIndex::Type::FRONT : cereal::EncodeIndex::Type::FULL_H_E_V_C);
          }
          eidx.setEncodeId(encode_idx);
          eidx.setSegmentNum(cur_seg);
          eidx.setSegmentId(out_id);
          if (lh) {
            // TODO: this should read cereal/services.h for qlog decimation
            auto bytes = msg.toBytes();
            lh_log(lh, bytes.begin(), bytes.size(), true);
          }
        }
      }

      cnt++;
      encode_idx++;
    }

    if (lh) {
      lh_close(lh);
      lh = NULL;
    }
  }

  LOG("encoder destroy");
  for(auto &e : encoders) {
    e->encoder_close();
    delete e;
  }
}

int clear_locks_fn(const char* fpath, const struct stat *sb, int tyupeflag) {
  const char* dot = strrchr(fpath, '.');
  if (dot && strcmp(dot, ".lock") == 0) {
    unlink(fpath);
  }
  return 0;
}

void clear_locks() {
  ftw(LOG_ROOT.c_str(), clear_locks_fn, 16);
}

void logger_rotate() {
  {
    std::unique_lock lk(s.rotate_lock);
    int segment = -1;
    int err = logger_next(&s.logger, LOG_ROOT.c_str(), s.segment_path, sizeof(s.segment_path), &segment);
    assert(err == 0);
    s.rotate_segment = segment;
    s.waiting_rotate = 0;
    s.last_rotate_tms = millis_since_boot();
  }
  s.rotate_cv.notify_all();
  LOGW((s.logger.part == 0) ? "logging to %s" : "rotated to %s", s.segment_path);
}

void rotate_if_needed() {
  if (s.waiting_rotate == s.max_waiting) {
    logger_rotate();
  }

  double tms = millis_since_boot();
  if ((tms - s.last_rotate_tms) > SEGMENT_LENGTH * 1000 &&
      (tms - s.last_camera_seen_tms) > NO_CAMERA_PATIENCE &&
      !LOGGERD_TEST) {
    LOGW("no camera packet seen. auto rotating");
    logger_rotate();
  }
}

} // namespace

int main(int argc, char** argv) {
  setpriority(PRIO_PROCESS, 0, -20);

  clear_locks();

  // setup messaging
  typedef struct QlogState {
    int counter, freq;
  } QlogState;
  std::unordered_map<SubSocket*, QlogState> qlog_states;

  s.ctx = Context::create();
  Poller * poller = Poller::create();

  // subscribe to all socks
  for (const auto& it : services) {
    if (!it.should_log) continue;

    SubSocket * sock = SubSocket::create(s.ctx, it.name);
    assert(sock != NULL);
    poller->registerSocket(sock);
    qlog_states[sock] = {.counter = 0, .freq = it.decimation};
  }

  // init logger
  logger_init(&s.logger, "rlog", true);
  logger_rotate();
  Params().put("CurrentRoute", s.logger.route_name);

  // init encoders
  s.last_camera_seen_tms = millis_since_boot();
  std::vector<std::thread> encoder_threads;
  for (const auto &ci : cameras_logged) {
    if (ci.enable) {
      encoder_threads.push_back(std::thread(encoder_thread, ci));
      if (ci.trigger_rotate) s.max_waiting++;
    }
  }

  uint64_t msg_count = 0, bytes_count = 0;
  double start_ts = millis_since_boot();
  while (!do_exit) {
    // Check if all encoders are ready and start encoding at the same time
    if ((s.max_waiting > 1) && !s.encoders_synced && (s.encoders_ready == s.max_waiting)) {
      // Small margin in case one of the encoders already dropped the next frame
      s.start_frame_id = s.latest_frame_id + 2;
      s.encoders_synced = true;
      LOGE("starting encoders at frame id %d", s.start_frame_id.load());
    }


    // poll for new messages on all sockets
    for (auto sock : poller->poll(1000)) {
      // drain socket
      QlogState &qs = qlog_states[sock];
      Message *msg = nullptr;
      while (!do_exit && (msg = sock->receive(true))) {
        const bool in_qlog = qs.freq != -1 && (qs.counter++ % qs.freq == 0);
        logger_log(&s.logger, (uint8_t *)msg->getData(), msg->getSize(), in_qlog);
        bytes_count += msg->getSize();
        delete msg;

        rotate_if_needed();

        if ((++msg_count % 1000) == 0) {
          double seconds = (millis_since_boot() - start_ts) / 1000.0;
          LOGD("%lu messages, %.2f msg/sec, %.2f KB/sec", msg_count, msg_count / seconds, bytes_count * 0.001 / seconds);
        }
      }
    }
  }

  LOGW("closing encoders");
  s.rotate_cv.notify_all();
  for (auto &t : encoder_threads) t.join();

  LOGW("closing logger");
  logger_close(&s.logger, &do_exit);

  if (do_exit.power_failure) {
    LOGE("power failure");
    sync();
    LOGE("sync done");
  }

  // messaging cleanup
  for (auto &[sock, qs] : qlog_states) delete sock;
  delete poller;
  delete s.ctx;

  return 0;
}
