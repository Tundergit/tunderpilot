#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <eigen3/Eigen/Dense>

#include "visionipc_client.h"
#include "common/swaglog.h"
#include "common/clutil.h"
#include "common/util.h"

#include "models/driving.h"
#include "messaging.hpp"

ExitHandler do_exit;
// globals
bool live_calib_seen;
mat3 cur_transform;
std::mutex transform_lock;

void calibration_thread() {
  set_thread_name("calibration");
  set_realtime_priority(50);

  SubMaster sm({"liveCalibration"});

  /*
     import numpy as np
     from common.transformations.model import medmodel_frame_from_road_frame
     medmodel_frame_from_ground = medmodel_frame_from_road_frame[:, (0, 1, 3)]
     ground_from_medmodel_frame = np.linalg.inv(medmodel_frame_from_ground)
  */
  Eigen::Matrix<float, 3, 3> ground_from_medmodel_frame;
  ground_from_medmodel_frame <<
    0.00000000e+00, 0.00000000e+00, 1.00000000e+00,
    -1.09890110e-03, 0.00000000e+00, 2.81318681e-01,
    -1.84808520e-20, 9.00738606e-04,-4.28751576e-02;

  Eigen::Matrix<float, 3, 3> fcam_intrinsics = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>(fcam_intrinsic_matrix.v);
  const mat3 yuv_transform = get_model_yuv_transform();

  while (!do_exit) {
    if (sm.update(100) > 0){

      auto extrinsic_matrix = sm["liveCalibration"].getLiveCalibration().getExtrinsicMatrix();
      Eigen::Matrix<float, 3, 4> extrinsic_matrix_eigen;
      for (int i = 0; i < 4*3; i++){
        extrinsic_matrix_eigen(i / 4, i % 4) = extrinsic_matrix[i];
      }

      auto camera_frame_from_road_frame = fcam_intrinsics * extrinsic_matrix_eigen;
      Eigen::Matrix<float, 3, 3> camera_frame_from_ground;
      camera_frame_from_ground.col(0) = camera_frame_from_road_frame.col(0);
      camera_frame_from_ground.col(1) = camera_frame_from_road_frame.col(1);
      camera_frame_from_ground.col(2) = camera_frame_from_road_frame.col(3);

      auto warp_matrix = camera_frame_from_ground * ground_from_medmodel_frame;
      mat3 transform = {};
      for (int i=0; i<3*3; i++) {
        transform.v[i] = warp_matrix(i / 3, i % 3);
      }
      mat3 model_transform = matmul3(yuv_transform, transform);
      std::lock_guard lk(transform_lock);
      cur_transform = model_transform;
      live_calib_seen = true;
    }
  }
}

void run_model(ModelState &model, VisionIpcClient &vipc_client) {
  // messaging
  PubMaster pm({"modelV2", "cameraOdometry"});
  SubMaster sm({"lateralPlan", "roadCameraState"});

  // setup filter to track dropped frames
  FirstOrderFilter frame_dropped_filter(0., 10., 1. / MODEL_FREQ);

  uint32_t frame_id = 0, last_vipc_frame_id = 0;
  double last = 0;
  int desire = -1;
  uint32_t run_count = 0;

  while (!do_exit) {
    VisionIpcBufExtra extra = {};
    VisionBuf *buf = vipc_client.recv(&extra);
    if (buf == nullptr) continue;

    transform_lock.lock();
    mat3 model_transform = cur_transform;
    const bool run_model_this_iter = live_calib_seen;
    transform_lock.unlock();

    if (sm.update(0) > 0) {
      // TODO: path planner timeout?
      desire = ((int)sm["lateralPlan"].getLateralPlan().getDesire());
      frame_id = sm["roadCameraState"].getRoadCameraState().getFrameId();
    }

    if (run_model_this_iter) {
      run_count++;

      float vec_desire[DESIRE_LEN] = {0};
      if (desire >= 0 && desire < DESIRE_LEN) {
        vec_desire[desire] = 1.0;
      }

      double mt1 = millis_since_boot();
      ModelDataRaw model_buf = model_eval_frame(&model, buf->buf_cl, buf->width, buf->height,
                                                model_transform, vec_desire);
      double mt2 = millis_since_boot();
      float model_execution_time = (mt2 - mt1) / 1000.0;

      // tracked dropped frames
      uint32_t vipc_dropped_frames = extra.frame_id - last_vipc_frame_id - 1;
      float frames_dropped = frame_dropped_filter.update((float)std::min(vipc_dropped_frames, 10U));
      if (run_count < 10) { // let frame drops warm up
        frame_dropped_filter.reset(0);
        frames_dropped = 0.;
      }
      
      float frame_drop_ratio = frames_dropped / (1 + frames_dropped);

      model_publish(pm, extra.frame_id, frame_id, frame_drop_ratio, model_buf, extra.timestamp_eof, model_execution_time,
                    kj::ArrayPtr<const float>(model.output.data(), model.output.size()));
      posenet_publish(pm, extra.frame_id, vipc_dropped_frames, model_buf, extra.timestamp_eof);

      //printf("model process: %.2fms, from last %.2fms, vipc_frame_id %u, frame_id, %u, frame_drop %.3f\n", mt2 - mt1, mt1 - last, extra.frame_id, frame_id, frame_drop_ratio);
      last = mt1;
      last_vipc_frame_id = extra.frame_id;
    }
  }
}

int main(int argc, char **argv) {
  set_realtime_priority(54);

#ifdef QCOM
  set_core_affinity(2);
#elif QCOM2
  // CPU usage is much lower when pinned to a single big core
  set_core_affinity(4);
#endif

  // start calibration thread
  std::thread thread = std::thread(calibration_thread);

  // cl init
  cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  cl_context context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));

  // init the models
  ModelState model;
  model_init(&model, device_id, context);
  LOGW("models loaded, modeld starting");

  VisionIpcClient vipc_client = VisionIpcClient("camerad", VISION_STREAM_YUV_BACK, true, device_id, context);
  while (!do_exit && !vipc_client.connect(false)) {
    util::sleep_for(100);
  }

  // run the models
  // vipc_client.connected is false only when do_exit is true
  if (vipc_client.connected) {
    const VisionBuf *b = &vipc_client.buffers[0];
    LOGW("connected with buffer size: %d (%d x %d)", b->len, b->width, b->height);
    run_model(model, vipc_client);
  }

  model_free(&model);
  LOG("joining calibration thread");
  thread.join();
  CL_CHECK(clReleaseContext(context));
  return 0;
}
