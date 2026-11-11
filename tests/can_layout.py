"""Audit both boards as three physical buses, using the actual config headers.

This checks planned ownership, not the wiring or actual feedback rate of hardware.
Run with both sibling projects present; no third party Python packages are needed.
"""
from pathlib import Path
import math
import re

root = Path(__file__).resolve().parents[2]
chassis = root / "sentry_chassis_steering_code"
gimbal = root / "sentry_gimbal_steering_code"


def definitions(path):
    return dict(re.findall(r"^#define[ \t]+(\w+)[ \t]+([^/\r\n]+)", path.read_text(encoding="utf-8"), re.M))


def number(value):
    return int(value.strip().rstrip("uUlL"), 0)


configs = {"chassis": definitions(chassis / "application/board_config.h"),
           "gimbal": definitions(gimbal / "application/board_config.h")}
protocol = definitions(chassis / "application/board_link.h")
assert (chassis / "application/robot_def.h").read_bytes() == (gimbal / "application/robot_def.h").read_bytes()
assert (chassis / "application/board_link.h").read_bytes() == (gimbal / "application/board_link.h").read_bytes()
for common in ("application/board_transport.c", "application/board_transport.h", "bsp/can/bsp_can.c", "modules/can_comm/can_comm.c", "modules/can_comm/can_comm.h",
               "modules/super_cap/super_cap.c", "modules/super_cap/super_cap.h",
               "modules/motor/power_control.c", "modules/referee/rm_referee.c"):
    assert (chassis / common).read_bytes() == (gimbal / common).read_bytes(), common


def port(board, macro):
    match = re.fullmatch(r"\(&hcan([12])\)\s*", configs[board][macro])
    assert match, (board, macro, configs[board][macro])
    return board, int(match[1])


linked = {port(board, "BOARD_LINK_CAN_HANDLE") for board in configs}
assert len(linked) == 2
assert linked == {("chassis", 1), ("gimbal", 1)}, "Both board-link endpoints must use CAN1"


def net(endpoint):
    return "shared" if endpoint in linked else f"{endpoint[0]}-CAN{endpoint[1]}"


frames = {}
motors = []
groups = {}


def producer(endpoint, can_id, owner):
    key = net(endpoint), can_id
    old = frames.setdefault(key, owner)
    assert old == owner, f"CAN ID collision {key}: {old} / {owner}"


def motor(board, macro, motor_id, kind, name):
    endpoint = port(board, macro)
    feedback = (0x204 if kind == "GM6020" else 0x200) + motor_id
    command = (0x1FF if motor_id <= 4 else 0x2FF) if kind == "GM6020" else (0x200 if motor_id <= 4 else 0x1FF)
    slot = (motor_id - 1) % 4
    producer(endpoint, feedback, name)
    producer(endpoint, command, board)
    slot_key = net(endpoint), command, slot
    assert slot_key not in groups, f"Motor control slot collision: {name} / {groups.get(slot_key)}"
    groups[slot_key] = name
    motors.append((name, net(endpoint), board, command))


for i in range(1, 5):
    motor("chassis", "CHASSIS_DRIVE_CAN_HANDLE", i, "M3508", f"drive-{i}")
    motor("chassis", "CHASSIS_STEERING_CAN_HANDLE", i, "GM6020", f"steer-{i}")
for macro, id_macro, kind, name in (
    ("GIMBAL_YAW_CAN_HANDLE", "GIMBAL_YAW_MOTOR_ID", "GM6020", "yaw"),
    ("GIMBAL_PITCH_CAN_HANDLE", "GIMBAL_PITCH_MOTOR_ID", "GM6020", "pitch"),
    ("SHOOT_CAN_HANDLE", "SHOOT_FRICTION_RIGHT_ID", "M3508", "friction-right"),
    ("SHOOT_CAN_HANDLE", "SHOOT_FRICTION_LEFT_ID", "M3508", "friction-left"),
    ("SHOOT_CAN_HANDLE", "SHOOT_LOADER_ID", "M2006", "loader"),
):
    motor("gimbal", macro, number(configs["gimbal"][id_macro]), kind, name)

cap_port = port("chassis", "CHASSIS_CAP_CAN_HANDLE")
producer(cap_port, number(configs["chassis"]["CHASSIS_CAP_TX_ID"]), "chassis")
producer(cap_port, number(configs["chassis"]["CHASSIS_CAP_RX_ID"]), "supercap")
producer(port("chassis", "BOARD_LINK_CAN_HANDLE"), number(protocol["BOARD_CHASSIS_TX_ID"]), "chassis")
producer(port("gimbal", "BOARD_LINK_CAN_HANDLE"), number(protocol["BOARD_GIMBAL_TX_ID"]), "gimbal")
assert len(motors) == 13
assert "UITaskInit(&huart6" in (chassis / "application/chassis/chassis.c").read_text(encoding="utf-8")

# ARM ABI sizes are also asserted by board_link.h and exercised by the CAN tests.
command_frames = math.ceil((26 + 4) / 8) * 1000 / number(protocol["BOARD_COMMAND_PERIOD_MS"])
feedback_frames = math.ceil((28 + 4) / 8) * 1000 / number(protocol["BOARD_FEEDBACK_PERIOD_MS"])
for board, project in (("chassis", chassis), ("gimbal", gimbal)):
    task = (project / "application/robot_task.h").read_text(encoding="utf-8")
    motor_task = task.split("void StartMOTORTASK(void const *argument)\n{", 1)[1].split("__attribute__", 1)[0]
    assert "osDelay(2)" in motor_task, f"Unexpected motor send interval: {board}"

print("PASS: 13 motors, supercap and two board-link producers have no physical-bus ID or slot conflicts.")
print("Referee: chassis USART6; forwarded by board link and supercap command, not a direct CAN node.")
for bus in sorted({item[1] for item in motors}):
    members = [item for item in motors if item[1] == bus]
    current_groups = {(item[2], item[3]) for item in members}
    base = len(current_groups) * 500
    if bus == "shared": base += command_frames + feedback_frames
    if bus == net(cap_port): base += 10 + 50  # Cap feedback 50Hz is an explicit estimate, not measured.
    all_1khz = base + len(members) * 1000
    reduced = base + sum(500 if item[0].startswith("drive-") else 1000 for item in members)
    print(f"{bus}: motors={len(members)}, current groups={len(current_groups)}, "
          f"all motor feedback 1kHz <= {all_1khz * 135 / 1e4:.1f}%, "
          f"drive feedback 500Hz <= {reduced * 135 / 1e4:.1f}% @ 1Mbit/s")
print("Load figures assume 135 bits per 8-byte frame including stuffing/intermission; no errors or retries.")
print("Feedback rates are device-side settings and must be checked on hardware; software does not change them.")
