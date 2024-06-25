import os
import threading
import time
from enum import Enum

from pymavlink import mavutil


class ParachuteStatusFields(Enum):
    TIME_BOOT_MS = 0
    ERROR_STATUS = 1
    ARM_STATUS = 2
    DEPLOYMENT_STATUS = 3
    SAFETY_STATUS = 4
    ATS_ARM_ALTITUDE = 5
    PARACHUTE_PACKED_DATE = 6


def stream_heartbeat(event, mavlink_connection_out):
    while True:
        event.wait()
        mavlink_connection_out.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_PARACHUTE, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )
        time.sleep(1)


def stream_parachute_status(event, mavlink_connection_out):
    while True:
        event.wait()
        mavlink_connection_out.mav.parachute_status_send(*parachute_status)
        time.sleep(1)


def publish_parachute_status(mavlink_connection_out):
    mavlink_connection_out.mav.parachute_status_send(*parachute_status)


def publish_component_information_basic(mavlink_connection_out):
    mavlink_connection_out.mav.component_information_basic_send(
        0,  # time_boot_ms
        0,  # capabilities
        0,  # time_manufacture_s
        "Parachute Vendor".encode(),  # vendor_name
        "Parachute Model".encode(),  # model_name
        "Parachute Software Version".encode(),  # software_version
        "Parachute Hardware Version".encode(),  # hardware_version
        "Parachute Serial Number".encode(),  # serial_number
    )


def update_parachute_status(field, value):
    global parachute_status
    tmp = list(parachute_status)
    tmp[field.value] = value
    parachute_status = tuple(tmp)


def handle_command_long(msg, mavlink_connection_out):
    # MAV_CMD_REQUEST_MESSAGE
    if msg.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
        if msg.param1 == mavutil.mavlink.MAVLINK_MSG_ID_PARACHUTE_STATUS:
            mavlink_connection_out.mav.command_ack_send(
                msg.command, result=mavutil.mavlink.MAV_RESULT_ACCEPTED
            )
            publish_parachute_status(mavlink_connection_out)
        elif msg.param1 == mavutil.mavlink.MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC:
            mavlink_connection_out.mav.command_ack_send(
                msg.command, result=mavutil.mavlink.MAV_RESULT_ACCEPTED
            )
            publish_component_information_basic(mavlink_connection_out)
        else:
            mavlink_connection_out.mav.command_ack_send(
                msg.command, result=mavutil.mavlink.MAV_RESULT_DENIED
            )

    # MAV_CMD_SET_MESSAGE_INTERVAL
    elif msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
        if msg.param1 == mavutil.mavlink.MAVLINK_MSG_ID_PARACHUTE_STATUS:
            if msg.param2 == -1:
                parachute_status_thread_event.clear()
            else:
                parachute_status_thread_event.set()
            mavlink_connection_out.mav.command_ack_send(
                msg.command, result=mavutil.mavlink.MAV_RESULT_ACCEPTED
            )
        else:
            mavlink_connection_out.mav.command_ack_send(
                msg.command, result=mavutil.mavlink.MAV_RESULT_DENIED
            )

    # MAV_CMD_DO_PARACHUTE
    elif msg.command == mavutil.mavlink.MAV_CMD_DO_PARACHUTE:
        if msg.param1 == mavutil.mavlink.PARACHUTE_RELEASE:
            if (
                parachute_status[ParachuteStatusFields.ARM_STATUS.value]
                & mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC
            ):
                if (
                    parachute_status[ParachuteStatusFields.SAFETY_STATUS.value]
                    & mavutil.mavlink.PARACHUTE_SAFETY_FLAGS_GROUND_CLEARED
                ):
                    mavlink_connection_out.mav.command_ack_send(
                        msg.command, result=mavutil.mavlink.MAV_RESULT_ACCEPTED
                    )
                    update_parachute_status(
                        ParachuteStatusFields.DEPLOYMENT_STATUS,
                        int(mavutil.mavlink.PARACHUTE_DEPLOYMENT_TRIGGER_DRONE),
                    )
                else:
                    mavlink_connection_out.mav.command_ack_send(
                        msg.command, result=mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
                    )
            else:
                mavlink_connection_out.mav.command_ack_send(
                    msg.command, result=mavutil.mavlink.MAV_RESULT_FAILED
                )
        else:
            mavlink_connection_out.mav.command_ack_send(
                msg.command, result=mavutil.mavlink.MAV_RESULT_DENIED
            )

    # MAV_CMD_SET_PARACHUTE_ARM
    elif msg.command == mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM:
        arm_flags = int(msg.param1)
        bitmask = int(msg.param2)
        parachute_trigger_source_invalid = (
            (mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ENUM_END - 1) << 1
        ) - 1
        if (
            arm_flags & ~parachute_trigger_source_invalid
            or bitmask & ~parachute_trigger_source_invalid
        ):
            mavlink_connection_out.mav.command_ack_send(
                msg.command, result=mavutil.mavlink.MAV_RESULT_DENIED
            )
        else:
            mavlink_connection_out.mav.command_ack_send(
                msg.command, result=mavutil.mavlink.MAV_RESULT_ACCEPTED
            )
            flag = 1
            new_arm_status = 0
            while flag < mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ENUM_END:
                if bitmask & flag:
                    new_arm_status |= arm_flags & flag
                else:
                    new_arm_status |= (
                        parachute_status[ParachuteStatusFields.ARM_STATUS.value] & flag
                    )
                flag <<= 1
            update_parachute_status(ParachuteStatusFields.ARM_STATUS, new_arm_status)

    else:
        mavlink_connection_out.mav.command_ack_send(
            msg.command, result=mavutil.mavlink.MAV_RESULT_UNSUPPORTED
        )


def main():
    os.environ["MAVLINK20"] = "1"
    os.environ["MAVLINK_DIALECT"] = "common"
    global parachute_status_thread_event
    global parachute_status

    # Start UDP connections
    mavlink_connection_udpin = mavutil.mavlink_connection("udpin:localhost:14540", dialect="common")
    mavlink_connection_udpout = mavutil.mavlink_connection(
        "udpout:localhost:14541", dialect="common"
    )

    # Heartbeat
    heartbeat_thread_event = threading.Event()
    heartbeat_thread = threading.Thread(
        target=stream_heartbeat,
        daemon=True,
        args=(heartbeat_thread_event, mavlink_connection_udpout),
    )
    heartbeat_thread.start()

    # Parachute status
    parachute_status = (
        0,  # time_boot_ms
        0,  # error_status
        0,  # arm_status
        mavutil.mavlink.PARACHUTE_DEPLOYMENT_TRIGGER_NONE,  # deployment_status
        mavutil.mavlink.PARACHUTE_SAFETY_FLAGS_GROUND_CLEARED,  # safety_status
        50,  # ats_arm_altitude
        "2024-06-19".encode(),  # parachute_packed_date
    )
    parachute_status_thread_event = threading.Event()
    parachute_status_thread = threading.Thread(
        target=stream_parachute_status,
        daemon=True,
        args=(parachute_status_thread_event, mavlink_connection_udpout),
    )
    parachute_status_thread.start()

    heartbeat_thread_event.set()
    parachute_status_thread_event.set()

    while True:
        msg = mavlink_connection_udpin.recv_match(type="COMMAND_LONG", blocking=True)
        if msg:
            # print(msg)
            handle_command_long(msg, mavlink_connection_udpout)


if __name__ == "__main__":
    main()
