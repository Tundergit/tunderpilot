
#include "driving.h"

#include <assert.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <eigen3/Eigen/Dense>

#include "selfdrive/common/clutil.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/timing.h"

constexpr int DESIRE_PRED_SIZE = 32;
constexpr int OTHER_META_SIZE = 4;

constexpr int MODEL_WIDTH = 512;
constexpr int MODEL_HEIGHT = 256;
constexpr int MODEL_FRAME_SIZE = MODEL_WIDTH * MODEL_HEIGHT * 3 / 2;

constexpr int PLAN_MHP_N = 5;
constexpr int PLAN_MHP_COLUMNS = 30;
constexpr int PLAN_MHP_VALS = 30*33;
constexpr int PLAN_MHP_SELECTION = 1;
constexpr int PLAN_MHP_GROUP_SIZE =  (2*PLAN_MHP_VALS + PLAN_MHP_SELECTION);

constexpr int LEAD_MHP_N = 5;
constexpr int LEAD_MHP_VALS = 4;
constexpr int LEAD_MHP_SELECTION = 3;
constexpr int LEAD_MHP_GROUP_SIZE = (2*LEAD_MHP_VALS + LEAD_MHP_SELECTION);

constexpr int POSE_SIZE = 12;

constexpr int PLAN_IDX = 0;
constexpr int LL_IDX = PLAN_IDX + PLAN_MHP_N*PLAN_MHP_GROUP_SIZE;
constexpr int LL_PROB_IDX = LL_IDX + 4*2*2*33;
constexpr int RE_IDX = LL_PROB_IDX + 4;
constexpr int LEAD_IDX = RE_IDX + 2*2*2*33;
constexpr int LEAD_PROB_IDX = LEAD_IDX + LEAD_MHP_N*(LEAD_MHP_GROUP_SIZE);
constexpr int DESIRE_STATE_IDX = LEAD_PROB_IDX + 3;
constexpr int META_IDX = DESIRE_STATE_IDX + DESIRE_LEN;
constexpr int POSE_IDX = META_IDX + OTHER_META_SIZE + DESIRE_PRED_SIZE;
constexpr int OUTPUT_SIZE =  POSE_IDX + POSE_SIZE;
#ifdef TEMPORAL
  constexpr int TEMPORAL_SIZE = 512;
#else
  constexpr int TEMPORAL_SIZE = 0;
#endif

// #define DUMP_YUV

void model_init(ModelState* s, cl_device_id device_id, cl_context context) {
  frame_init(&s->frame, MODEL_WIDTH, MODEL_HEIGHT, device_id, context);
  s->input_frames = std::make_unique<float[]>(MODEL_FRAME_SIZE * 2);

  constexpr int output_size = OUTPUT_SIZE + TEMPORAL_SIZE;
  s->output.resize(output_size);

#if defined(QCOM) || defined(QCOM2)
  s->m = std::make_unique<ThneedModel>("../../models/supercombo.thneed", &s->output[0], output_size, USE_GPU_RUNTIME);
#else
  s->m = std::make_unique<DefaultRunModel>("../../models/supercombo.dlc", &s->output[0], output_size, USE_GPU_RUNTIME);
#endif

#ifdef TEMPORAL
  s->m->addRecurrent(&s->output[OUTPUT_SIZE], TEMPORAL_SIZE);
#endif

#ifdef DESIRE
  s->m->addDesire(s->pulse_desire, DESIRE_LEN);
#endif

#ifdef TRAFFIC_CONVENTION
  const int idx = Params().getBool("IsRHD") ? 1 : 0;
  s->traffic_convention[idx] = 1.0;
  s->m->addTrafficConvention(s->traffic_convention, TRAFFIC_CONVENTION_LEN);
#endif

  s->q = CL_CHECK_ERR(clCreateCommandQueue(context, device_id, 0, &err));
}

ModelDataRaw model_eval_frame(ModelState* s, cl_mem yuv_cl, int width, int height,
                           const mat3 &transform, float *desire_in) {
#ifdef DESIRE
  if (desire_in != NULL) {
    for (int i = 1; i < DESIRE_LEN; i++) {
      // Model decides when action is completed
      // so desire input is just a pulse triggered on rising edge
      if (desire_in[i] - s->prev_desire[i] > .99) {
        s->pulse_desire[i] = desire_in[i];
      } else {
        s->pulse_desire[i] = 0.0;
      }
      s->prev_desire[i] = desire_in[i];
    }
  }
#endif

  //for (int i = 0; i < OUTPUT_SIZE + TEMPORAL_SIZE; i++) { printf("%f ", s->output[i]); } printf("\n");

  float *new_frame_buf = frame_prepare(&s->frame, s->q, yuv_cl, width, height, transform);
  memmove(&s->input_frames[0], &s->input_frames[MODEL_FRAME_SIZE], sizeof(float)*MODEL_FRAME_SIZE);
  memmove(&s->input_frames[MODEL_FRAME_SIZE], new_frame_buf, sizeof(float)*MODEL_FRAME_SIZE);
  s->m->execute(&s->input_frames[0], MODEL_FRAME_SIZE*2);

  #ifdef DUMP_YUV
    FILE *dump_yuv_file = fopen("/sdcard/dump.yuv", "wb");
    fwrite(new_frame_buf, MODEL_HEIGHT*MODEL_WIDTH*3/2, sizeof(float), dump_yuv_file);
    fclose(dump_yuv_file);
    assert(1==2);
  #endif

  clEnqueueUnmapMemObject(s->q, s->frame.net_input, (void*)new_frame_buf, 0, NULL, NULL);

  // net outputs
  ModelDataRaw net_outputs;
  net_outputs.plan = &s->output[PLAN_IDX];
  net_outputs.lane_lines = &s->output[LL_IDX];
  net_outputs.lane_lines_prob = &s->output[LL_PROB_IDX];
  net_outputs.road_edges = &s->output[RE_IDX];
  net_outputs.lead = &s->output[LEAD_IDX];
  net_outputs.lead_prob = &s->output[LEAD_PROB_IDX];
  net_outputs.meta = &s->output[DESIRE_STATE_IDX];
  net_outputs.pose = &s->output[POSE_IDX];
  return net_outputs;
}

void model_free(ModelState* s) {
  frame_free(&s->frame);
  CL_CHECK(clReleaseCommandQueue(s->q));
}

static const float *get_best_data(const float *data, int size, int group_size, int offset) {
  int max_idx = 0;
  for (int i = 1; i < size; i++) {
    if (data[(i + 1) * group_size + offset] >
        data[(max_idx + 1) * group_size + offset]) {
      max_idx = i;
    }
  }
  return &data[max_idx * group_size];
}

static const float *get_plan_data(float *plan) {
  return get_best_data(plan, PLAN_MHP_N, PLAN_MHP_GROUP_SIZE, -1);
}

static const float *get_lead_data(const float *lead, int t_offset) {
  return get_best_data(lead, LEAD_MHP_N, LEAD_MHP_GROUP_SIZE, t_offset - LEAD_MHP_SELECTION);
}


void fill_lead_v2(cereal::ModelDataV2::LeadDataV2::Builder lead, const float *lead_data, const float *prob, int t_offset, float t) {
  const float *data = get_lead_data(lead_data, t_offset);
  lead.setProb(sigmoid(prob[t_offset]));
  lead.setT(t);
  float xyva_arr[LEAD_MHP_VALS];
  float xyva_stds_arr[LEAD_MHP_VALS];
  for (int i=0; i<LEAD_MHP_VALS; i++) {
    xyva_arr[i] = data[i];
    xyva_stds_arr[i] = exp(data[LEAD_MHP_VALS + i]);
  }
  lead.setXyva(xyva_arr);
  lead.setXyvaStd(xyva_stds_arr);
}

void fill_meta(cereal::ModelDataV2::MetaData::Builder meta, const float *meta_data) {
  float desire_state_softmax[DESIRE_LEN];
  float desire_pred_softmax[4*DESIRE_LEN];
  softmax(&meta_data[0], desire_state_softmax, DESIRE_LEN);
  for (int i=0; i<4; i++) {
    softmax(&meta_data[DESIRE_LEN + OTHER_META_SIZE + i*DESIRE_LEN],
            &desire_pred_softmax[i*DESIRE_LEN], DESIRE_LEN);
  }
  meta.setDesireState(desire_state_softmax);
  meta.setEngagedProb(sigmoid(meta_data[DESIRE_LEN]));
  meta.setGasDisengageProb(sigmoid(meta_data[DESIRE_LEN + 1]));
  meta.setBrakeDisengageProb(sigmoid(meta_data[DESIRE_LEN + 2]));
  meta.setSteerOverrideProb(sigmoid(meta_data[DESIRE_LEN + 3]));
  meta.setDesirePrediction(desire_pred_softmax);
}

void fill_xyzt(cereal::ModelDataV2::XYZTData::Builder xyzt, const float * data,
               int columns, int column_offset, float * plan_t_arr, bool fill_std) {
  float x_arr[TRAJECTORY_SIZE] = {};
  float y_arr[TRAJECTORY_SIZE] = {};
  float z_arr[TRAJECTORY_SIZE] = {};
  float x_std_arr[TRAJECTORY_SIZE];
  float y_std_arr[TRAJECTORY_SIZE];
  float z_std_arr[TRAJECTORY_SIZE];
  float t_arr[TRAJECTORY_SIZE];
  for (int i=0; i<TRAJECTORY_SIZE; i++) {
    // column_offset == -1 means this data is X indexed not T indexed
    if (column_offset >= 0) {
      t_arr[i] = T_IDXS[i];
      x_arr[i] = data[i*columns + 0 + column_offset];
      x_std_arr[i] = data[columns*(TRAJECTORY_SIZE + i) + 0 + column_offset];
    } else {
      t_arr[i] = plan_t_arr[i];
      x_arr[i] = X_IDXS[i];
      x_std_arr[i] = NAN;
    }
    y_arr[i] = data[i*columns + 1 + column_offset];
    y_std_arr[i] = data[columns*(TRAJECTORY_SIZE + i) + 1 + column_offset];
    z_arr[i] = data[i*columns + 2 + column_offset];
    z_std_arr[i] = data[columns*(TRAJECTORY_SIZE + i) + 2 + column_offset];
  }
  xyzt.setX(x_arr);
  xyzt.setY(y_arr);
  xyzt.setZ(z_arr);
  xyzt.setT(t_arr);
  if (fill_std) {
    xyzt.setXStd(x_std_arr);
    xyzt.setYStd(y_std_arr);
    xyzt.setZStd(z_std_arr);
  }
}

void fill_model(cereal::ModelDataV2::Builder &framed, const ModelDataRaw &net_outputs) {
  // plan
  const float *best_plan = get_plan_data(net_outputs.plan);
  float plan_t_arr[TRAJECTORY_SIZE];
  for (int i=0; i<TRAJECTORY_SIZE; i++) {
    plan_t_arr[i] = best_plan[i*PLAN_MHP_COLUMNS + 15];
  }

  fill_xyzt(framed.initPosition(), best_plan, PLAN_MHP_COLUMNS, 0, plan_t_arr, true);
  fill_xyzt(framed.initVelocity(), best_plan, PLAN_MHP_COLUMNS, 3, plan_t_arr, false);
  fill_xyzt(framed.initOrientation(), best_plan, PLAN_MHP_COLUMNS, 9, plan_t_arr, false);
  fill_xyzt(framed.initOrientationRate(), best_plan, PLAN_MHP_COLUMNS, 12, plan_t_arr, false);

  // lane lines
  auto lane_lines = framed.initLaneLines(4);
  float lane_line_probs_arr[4];
  float lane_line_stds_arr[4];
  for (int i = 0; i < 4; i++) {
    fill_xyzt(lane_lines[i], &net_outputs.lane_lines[i*TRAJECTORY_SIZE*2], 2, -1, plan_t_arr, false);
    lane_line_probs_arr[i] = sigmoid(net_outputs.lane_lines_prob[i]);
    lane_line_stds_arr[i] = exp(net_outputs.lane_lines[2*TRAJECTORY_SIZE*(4 + i)]);
  }
  framed.setLaneLineProbs(lane_line_probs_arr);
  framed.setLaneLineStds(lane_line_stds_arr);

  // road edges
  auto road_edges = framed.initRoadEdges(2);
  float road_edge_stds_arr[2];
  for (int i = 0; i < 2; i++) {
    fill_xyzt(road_edges[i], &net_outputs.road_edges[i*TRAJECTORY_SIZE*2], 2, -1, plan_t_arr, false);
    road_edge_stds_arr[i] = exp(net_outputs.road_edges[2*TRAJECTORY_SIZE*(2 + i)]);
  }
  framed.setRoadEdgeStds(road_edge_stds_arr);

  // meta
  fill_meta(framed.initMeta(), net_outputs.meta);

  // leads
  auto leads = framed.initLeads(LEAD_MHP_SELECTION);
  float t_offsets[LEAD_MHP_SELECTION] = {0.0, 2.0, 4.0};
  for (int t_offset=0; t_offset<LEAD_MHP_SELECTION; t_offset++) {
    fill_lead_v2(leads[t_offset], net_outputs.lead, net_outputs.lead_prob, t_offset, t_offsets[t_offset]);
  }
}

void model_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t frame_id, float frame_drop,
                   const ModelDataRaw &net_outputs, uint64_t timestamp_eof,
                   float model_execution_time, kj::ArrayPtr<const float> raw_pred) {
  const uint32_t frame_age = (frame_id > vipc_frame_id) ? (frame_id - vipc_frame_id) : 0;
  MessageBuilder msg;
  auto framed = msg.initEvent().initModelV2();
  framed.setFrameId(vipc_frame_id);
  framed.setFrameAge(frame_age);
  framed.setFrameDropPerc(frame_drop * 100);
  framed.setTimestampEof(timestamp_eof);
  framed.setModelExecutionTime(model_execution_time);
  if (send_raw_pred) {
    framed.setRawPredictions(raw_pred.asBytes());
  }
  fill_model(framed, net_outputs);
  pm.send("modelV2", msg);
}

void posenet_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t vipc_dropped_frames,
                     const ModelDataRaw &net_outputs, uint64_t timestamp_eof) {
  float trans_arr[3];
  float trans_std_arr[3];
  float rot_arr[3];
  float rot_std_arr[3];

  for (int i =0; i < 3; i++) {
    trans_arr[i] = net_outputs.pose[i];
    trans_std_arr[i] = exp(net_outputs.pose[6 + i]);

    rot_arr[i] = net_outputs.pose[3 + i];
    rot_std_arr[i] = exp(net_outputs.pose[9 + i]);
  }

  MessageBuilder msg;
  auto posenetd = msg.initEvent(vipc_dropped_frames < 1).initCameraOdometry();
  posenetd.setTrans(trans_arr);
  posenetd.setRot(rot_arr);
  posenetd.setTransStd(trans_std_arr);
  posenetd.setRotStd(rot_std_arr);

  posenetd.setTimestampEof(timestamp_eof);
  posenetd.setFrameId(vipc_frame_id);

  pm.send("cameraOdometry", msg);
}
