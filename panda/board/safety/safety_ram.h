const int RAM_MAX_STEER = 261;
const int RAM_MAX_RT_DELTA = 112;        // max delta torque allowed for real time checks
const uint32_t RAM_RT_INTERVAL = 250000;  // 250ms between real time checks
const int RAM_MAX_RATE_UP = 10;
const int RAM_MAX_RATE_DOWN = 50;
const int RAM_MAX_TORQUE_ERROR = 240;    // max torque cmd in excess of torque motor up from 240
const int RAM_GAS_THRSLD = 30;  // 7% more than 2m/s
const int RAM_STANDSTILL_THRSLD = 10;  // about 1m/s
const CanMsg RAM_TX_MSGS[] = {{166, 0, 8}}; //, {250, 0, 8}}; // {177, 0, 8}};  // 177 is for long

AddrCheckStruct ram_addr_checks[] = {
  {.msg = {{35, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}}},  // EPS module
  {.msg = {{139, 0, 8, .check_checksum = false, .max_counter = 0U, .expected_timestep = 20000U}}},  // wheel speeds
  {.msg = {{153, 0, 8, .check_checksum = false, .max_counter = 15U, .expected_timestep = 20000U}}},  // forward cam ACC
  {.msg = {{129, 0, 8, .check_checksum = false, .max_counter = 15U,  .expected_timestep = 20000U}}},  // gas pedal
  {.msg = {{121, 0, 8, .check_checksum = false, .max_counter = 15U,  .expected_timestep = 20000U}}},  // brake pressed
};
#define RAM_ADDR_CHECK_LEN (sizeof(ram_addr_checks) / sizeof(ram_addr_checks[0]))
addr_checks ram_rx_checks = {ram_addr_checks, RAM_ADDR_CHECK_LEN};

static uint8_t ram_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static uint8_t ram_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  /* This function does not want the checksum byte in the input data.
  jeep ram canbus checksum from http://illmatics.com/Remote%20Car%20Hacking.pdf */
  uint8_t checksum = 0xFF;
  int len = GET_LEN(to_push);
  for (int j = 0; j < (len - 1); j++) {
    uint8_t shift = 0x80;
    uint8_t curr = (uint8_t)GET_BYTE(to_push, j);
    for (int i=0; i<8; i++) {
      uint8_t bit_sum = curr & shift;
      uint8_t temp_chk = checksum & 0x80U;
      if (bit_sum != 0U) {
        bit_sum = 0x1C;
        if (temp_chk != 0U) {
          bit_sum = 1;
        }
        checksum = checksum << 1;
        temp_chk = checksum | 1U;
        bit_sum ^= temp_chk;
      } else {
        if (temp_chk != 0U) {
          bit_sum = 0x1D;
        }
        checksum = checksum << 1;
        bit_sum ^= checksum;
      }
      checksum = bit_sum;
      shift = shift >> 1;
    }
  }
  return ~checksum;
}

static uint8_t ram_get_counter(CAN_FIFOMailBox_TypeDef *to_push) {
  // Well defined counter only for 8 bytes messages
  return (uint8_t)(GET_BYTE(to_push, 6) >> 4);
}

static int ram_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid = addr_safety_check(to_push, &ram_rx_checks,
                                 ram_get_checksum, ram_compute_checksum,
                                 ram_get_counter);

  if (valid && (GET_BUS(to_push) == 0)) {
    int addr = GET_ADDR(to_push);

    // Measured eps torque
    if (addr == 35) {
      int torque_meas_new = ((GET_BYTE(to_push, 4) & 0x7U) << 8) + GET_BYTE(to_push, 5) - 1024U;

      // update array of samples
      update_sample(&torque_meas, torque_meas_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == 153) {
      int cruise_engaged = ((GET_BYTE(to_push, 2) & 0x38) >> 3) == 7;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // update speed
    if (addr == 139) {
      int speed_l = (GET_BYTE(to_push, 0) << 4) + (GET_BYTE(to_push, 1) >> 4);
      int speed_r = (GET_BYTE(to_push, 2) << 4) + (GET_BYTE(to_push, 3) >> 4);
      vehicle_speed = (speed_l + speed_r) / 2;
      vehicle_moving = (int)vehicle_speed > RAM_STANDSTILL_THRSLD;
    }

    // exit controls on rising edge of gas press
    if (addr == 129) {
      gas_pressed = ((GET_BYTE(to_push, 5) & 0x7F) != 0) && ((int)vehicle_speed > RAM_GAS_THRSLD);
    }

    // exit controls on rising edge of brake press
    if (addr == 121) {
      brake_pressed = (GET_BYTE(to_push, 0) & 0x7) == 5;
      if (brake_pressed && (!brake_pressed_prev || vehicle_moving)) {
        controls_allowed = 0;
      }
      brake_pressed_prev = brake_pressed;
    }

    generic_rx_checks((addr == 0xa6));
  }
  return valid;
}

static int ram_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);

  if (!msg_allowed(to_send, RAM_TX_MSGS, sizeof(RAM_TX_MSGS) / sizeof(RAM_TX_MSGS[0]))) {
    tx = 0;  // for dev
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // LKA STEER
  if (addr == 0xa6) {
    int desired_torque = ((GET_BYTE(to_send, 0) & 0x7U) << 8) + GET_BYTE(to_send, 1) - 1024U;
    uint32_t ts = TIM2->CNT;
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, RAM_MAX_STEER, -RAM_MAX_STEER);

      // *** torque rate limit check ***
      violation |= dist_to_meas_check(desired_torque, desired_torque_last,
        &torque_meas, RAM_MAX_RATE_UP, RAM_MAX_RATE_DOWN, RAM_MAX_TORQUE_ERROR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, RAM_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > RAM_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 0;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 1;  // for dev
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if (addr == 177) {
    if ((GET_BYTE(to_send, 0) != 1) || ((GET_BYTE(to_send, 1) & 1) == 1)) {
      tx = 1;
    }
  }

  return tx;
}

static int ram_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  if (!relay_malfunction) {
    // forward CAN 0 -> 2 so stock LKAS camera sees messages
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    // forward all messages from camera except LKAS_COMMAND and LKAS_HUD
    if ((bus_num == 2) && (addr != 166)) { // && (addr != 250)) { - restoring auto high beams for now
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

static const addr_checks* ram_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
  return &ram_rx_checks;
}

const safety_hooks ram_hooks = {
  .init = ram_init,
  .rx = ram_rx_hook,
  .tx = ram_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = ram_fwd_hook,
};
