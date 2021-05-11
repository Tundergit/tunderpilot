#include "live_kf.h"

using namespace EKFS;
using namespace Eigen;

Eigen::Map<Eigen::VectorXd> get_mapvec(Eigen::VectorXd& vec) {
  return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.rows(), vec.cols());
}

Eigen::Map<MatrixXdr> get_mapmat(MatrixXdr& mat) {
  return Eigen::Map<MatrixXdr>(mat.data(), mat.rows(), mat.cols());
}

std::vector<Eigen::Map<Eigen::VectorXd>> get_vec_mapvec(std::vector<Eigen::VectorXd>& vec_vec) {
  std::vector<Eigen::Map<Eigen::VectorXd>> res;
  for (Eigen::VectorXd& vec : vec_vec) {
    res.push_back(get_mapvec(vec));
  }
  return res;
}

std::vector<Eigen::Map<MatrixXdr>> get_vec_mapmat(std::vector<MatrixXdr>& mat_vec) {
  std::vector<Eigen::Map<MatrixXdr>> res;
  for (MatrixXdr& mat : mat_vec) {
    res.push_back(get_mapmat(mat));
  }
  return res;
}

LiveKalman::LiveKalman() {
  this->dim_state = 23;
  this->dim_state_err = 22;

  this->initial_x = live_initial_x;
  this->initial_P = live_initial_P_diag.asDiagonal();
  this->Q = live_Q_diag.asDiagonal();
  for (auto& pair : live_obs_noise_diag) {
    this->obs_noise[pair.first] = pair.second.asDiagonal();
  }

  // init filter
  this->filter = std::make_shared<EKFSym>(this->name, get_mapmat(this->Q), get_mapvec(this->initial_x),
    get_mapmat(initial_P),  this->dim_state, this->dim_state_err, 0, 0, 0, std::vector<int>(),
    std::vector<int>{3}, std::vector<std::string>(), 0.2);
}

void LiveKalman::init_state(VectorXd& state, VectorXd& covs_diag, double filter_time) {
  MatrixXdr covs = covs_diag.asDiagonal();
  this->filter->init_state(get_mapvec(state), get_mapmat(covs), filter_time);
}

void LiveKalman::init_state(VectorXd& state, MatrixXdr& covs, double filter_time) {
  this->filter->init_state(get_mapvec(state), get_mapmat(covs), filter_time);
}

void LiveKalman::init_state(VectorXd& state, double filter_time) {
  MatrixXdr covs = this->filter->covs();
  this->filter->init_state(get_mapvec(state), get_mapmat(covs), filter_time);
}

VectorXd LiveKalman::get_x() {
  return this->filter->state();
}

MatrixXdr LiveKalman::get_P() {
  return this->filter->covs();
}

std::vector<MatrixXdr> LiveKalman::get_R(int kind, int n) {
  std::vector<MatrixXdr> R;
  for (int i = 0; i < n; i++) {
    R.push_back(this->obs_noise[kind]);
  }
  return R;
}

std::optional<Estimate> LiveKalman::predict_and_observe(double t, int kind, std::vector<VectorXd> meas, std::vector<MatrixXdr> R) {
  std::optional<Estimate> r;
  switch (kind) {
  case OBSERVATION_CAMERA_ODO_TRANSLATION:
    r = this->predict_and_update_odo_trans(meas, t, kind);
    break;
  case OBSERVATION_CAMERA_ODO_ROTATION:
    r = this->predict_and_update_odo_rot(meas, t, kind);
    break;
  case OBSERVATION_ODOMETRIC_SPEED:
    r = this->predict_and_update_odo_speed(meas, t, kind);
    break;
  default:
    if (R.size() == 0) {
      R = this->get_R(kind, meas.size());
    }
    r = this->filter->predict_and_update_batch(t, kind, get_vec_mapvec(meas), get_vec_mapmat(R));
    break;
  }
  return r;
}

std::optional<Estimate> LiveKalman::predict_and_update_odo_speed(std::vector<VectorXd> speed, double t, int kind) {
  std::vector<MatrixXdr> R;
  R.assign(speed.size(), (MatrixXdr(1, 1) << std::pow(0.2, 2)).finished().asDiagonal());
  return this->filter->predict_and_update_batch(t, kind, get_vec_mapvec(speed), get_vec_mapmat(R));
}

std::optional<Estimate> LiveKalman::predict_and_update_odo_trans(std::vector<VectorXd> trans, double t, int kind) {
  std::vector<VectorXd> z;
  std::vector<MatrixXdr> R;
  for (VectorXd& trns : trans) {
    assert(trns.size() == 6); // TODO remove
    z.push_back(trns.head(3));
    R.push_back(trns.segment<3>(3).array().square().matrix().asDiagonal());
  }
  return this->filter->predict_and_update_batch(t, kind, get_vec_mapvec(z), get_vec_mapmat(R));
}

std::optional<Estimate> LiveKalman::predict_and_update_odo_rot(std::vector<VectorXd> rot, double t, int kind) {
  std::vector<VectorXd> z;
  std::vector<MatrixXdr> R;
  for (VectorXd& rt : rot) {
    assert(rt.size() == 6); // TODO remove
    z.push_back(rt.head(3));
    R.push_back(rt.segment<3>(3).array().square().matrix().asDiagonal());
  }
  return this->filter->predict_and_update_batch(t, kind, get_vec_mapvec(z), get_vec_mapmat(R));
}

Eigen::VectorXd LiveKalman::get_initial_x() {
  return this->initial_x;
}

MatrixXdr LiveKalman::get_initial_P() {
  return this->initial_P;
}

MatrixXdr LiveKalman::H(VectorXd in) {
  assert(in.size() == 6);
  Matrix<double, 3, 6, Eigen::RowMajor> res;
  this->filter->get_extra_routine("H")(in.data(), res.data());
  return res;
}
