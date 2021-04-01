# CAN controls for MQB platform Volkswagen, Audi, Skoda and SEAT.
# PQ35/PQ46/NMS, and any future MLB, to come later.

def create_mqb_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "SET_ME_0X3": 0x3,
    "Assist_Torque": abs(apply_steer),
    "Assist_Requested": lkas_enabled,
    "Assist_VZ": 1 if apply_steer < 0 else 0,
    "HCA_Available": 1,
    "HCA_Standby": not lkas_enabled,
    "HCA_Active": lkas_enabled,
    "SET_ME_0XFE": 0xFE,
    "SET_ME_0X07": 0x07,
  }
  return packer.make_can_msg("HCA_01", bus, values, idx)

def create_mqb_hud_control(packer, bus, hca_enabled, steering_pressed, hud_alert, left_lane_visible, right_lane_visible,
                           ldw_lane_warning_left, ldw_lane_warning_right, ldw_side_dlc_tlc, ldw_dlc, ldw_tlc):
  if hca_enabled:
    left_lane_hud = 3 if left_lane_visible else 1
    right_lane_hud = 3 if right_lane_visible else 1
  else:
    left_lane_hud = 2 if left_lane_visible else 1
    right_lane_hud = 2 if right_lane_visible else 1

  values = {
    "LDW_Status_LED_gelb": 1 if hca_enabled and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if hca_enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": left_lane_hud,
    "LDW_Lernmodus_rechts": right_lane_hud,
    "LDW_Texte": hud_alert,
    "LDW_SW_Warnung_links": ldw_lane_warning_left,
    "LDW_SW_Warnung_rechts": ldw_lane_warning_right,
    "LDW_Seite_DLCTLC": ldw_side_dlc_tlc,
    "LDW_DLC": ldw_dlc,
    "LDW_TLC": ldw_tlc
  }
  return packer.make_can_msg("LDW_02", bus, values)

def create_mqb_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Hauptschalter": CS.graHauptschalter,
    "GRA_Abbrechen": buttonStatesToSend["cancel"],
    "GRA_Tip_Setzen": buttonStatesToSend["setCruise"],
    "GRA_Tip_Hoch": buttonStatesToSend["accelCruise"],
    "GRA_Tip_Runter": buttonStatesToSend["decelCruise"],
    "GRA_Tip_Wiederaufnahme": buttonStatesToSend["resumeCruise"],
    "GRA_Verstellung_Zeitluecke": 3 if buttonStatesToSend["gapAdjustCruise"] else 0,
    "GRA_Typ_Hauptschalter": CS.graTypHauptschalter,
    "GRA_Codierung": 2,
    "GRA_Tip_Stufe_2": CS.graTipStufe2,
    "GRA_ButtonTypeInfo": CS.graButtonTypeInfo
  }
  return packer.make_can_msg("GRA_ACC_01", bus, values, idx)
