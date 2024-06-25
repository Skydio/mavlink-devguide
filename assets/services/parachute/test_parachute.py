import os
import unittest

from pymavlink import mavutil

""" Helper functions """


class MavlinkUDPConnection:
    def __init__(self):
        os.environ["MAVLINK20"] = "1"
        os.environ["MAVLINK_DIALECT"] = "common"
        self.udpin = mavutil.mavlink_connection("udpin:localhost:14541", dialect="common")
        self.udpout = mavutil.mavlink_connection("udpout:localhost:14540", dialect="common")

    def close(self):
        self.udpin.close()
        self.udpout.close()


def encode_and_send_mavlink_command_long(mavlink_connection, command, param1, param2=0):
    message = mavlink_connection.udpout.mav.command_long_encode(
        mavlink_connection.udpout.target_system,  # Target System
        mavlink_connection.udpout.target_component,  # Target Component
        command,  # Command
        0,  # Confirmation
        param1,  # param1
        param2,  # param2
        0,  # param3 (unused)
        0,  # param4 (unused)
        0,  # param5 (unused)
        0,  # param6 (unused)
        0,  # param7: Response Target - flight-stack default
    )
    mavlink_connection.udpout.mav.send(message)


def assert_mavlink_command_ack(self, command, result, ack_timeout=3):
    ack = self.parachute_connection.udpin.recv_match(
        type="COMMAND_ACK", blocking=True, timeout=ack_timeout
    )
    self.assertIsNotNone(ack, "Did not recieve a COMMAND_ACK message")
    self.assertEqual(ack.command, command, f"Did not recieve a COMMAND_ACK message for {command}")
    self.assertEqual(ack.result, result, f"Did not recieve a {result} COMMAND_ACK message")


""" Test Cases """


class MavlinkParachuteStatusTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parachute_connection = MavlinkUDPConnection()
        cls.status_timeout = 5

    @classmethod
    def tearDownClass(cls):
        cls.parachute_connection.close()

    def setUp(self):
        # Disable the parachute status stream
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            mavutil.mavlink.MAVLINK_MSG_ID_PARACHUTE_STATUS,
            -1,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.status_timeout
        )
        self.assertIsNone(
            msg, "Recieved a PARACHUTE_STATUS message when the stream should be disabled"
        )

    def tearDown(self):
        # Re-enable the parachute status stream at default rate
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            mavutil.mavlink.MAVLINK_MSG_ID_PARACHUTE_STATUS,
            0,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.status_timeout
        )
        msg_2 = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.status_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertIsNotNone(msg_2, "Did not recieve a second PARACHUTE_STATUS message")

    def test_parachute_status_request(self):
        # Request one parachute status message
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAVLINK_MSG_ID_PARACHUTE_STATUS,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.status_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")


class MavlinkComponentInformationBasicTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parachute_connection = MavlinkUDPConnection()
        cls.msg_timeout = 5

    @classmethod
    def tearDownClass(cls):
        cls.parachute_connection.close()

    def setUp(self):
        msg = self.parachute_connection.udpin.recv_match(
            type="COMPONENT_INFORMATION_BASIC", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNone(
            msg, "Recieved a COMPONENT_INFORMATION_BASIC message when the stream should be disabled"
        )

    def test_component_information_basic_request(self):
        # Request component information basic message
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAVLINK_MSG_ID_COMPONENT_INFORMATION_BASIC,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="COMPONENT_INFORMATION_BASIC", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a COMPONENT_INFORMATION_BASIC message")


class MavlinkSetParachuteArmTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parachute_connection = MavlinkUDPConnection()
        cls.msg_timeout = 5

    @classmethod
    def tearDownClass(cls):
        cls.parachute_connection.close()

    def setUp(self):
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "PARACHUTE_STATUS stream is disabled")

    def tearDown(self):
        # Disarm all triggers
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            0,
            ((mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ENUM_END - 1) << 1) - 1,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertFalse(msg.arm_status, f"Parachute arm status not disarmed: {msg}")

    def test_arm_ats(self):
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertTrue(
            (msg.arm_status & mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS),
            "ATS trigger flag not set in arm status",
        )

    def test_arm_fc(self):
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertTrue(
            (msg.arm_status & mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC),
            "FC trigger flag not set in arm status",
        )

    def test_arm_ats_and_fc(self):
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS
            | mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS
            | mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")

    def test_arm_ats_then_fc(self):
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertTrue(
            (msg.arm_status & mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS),
            "ATS trigger flag not set in arm status",
        )
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertTrue(
            (msg.arm_status & mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC),
            "FC trigger flag not set in arm status",
        )
        self.assertFalse(
            (
                msg.arm_status
                & ~(
                    mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS
                    | mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC
                )
            ),
            "Trigger flags beside ATS and FC are set in arm status",
        )

    def test_arm_invalid_upper_bound(self):
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ENUM_END << 1,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ENUM_END << 1,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_DENIED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertFalse(msg.arm_status, f"Upper bound trigger flag set in arm status: {msg}")


class MavlinkDeployParachuteTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parachute_connection = MavlinkUDPConnection()
        cls.msg_timeout = 5

    @classmethod
    def tearDownClass(cls):
        cls.parachute_connection.close()

    def setUp(self):
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "PARACHUTE_STATUS stream is disabled")
        self.assertFalse(msg.arm_status, f"Parachute arm status not disarmed: {msg}")

    def tearDown(self):
        # Disarm all triggers
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            0,
            ((mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ENUM_END - 1) << 1) - 1,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertFalse(msg.arm_status, f"Parachute arm status not disarmed: {msg}")

    def test_deploy_parachute_with_ats(self):
        # Arm just the ATS trigger
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertTrue(
            (msg.arm_status & mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_ATS),
            "ATS trigger flag not set in arm status",
        )
        self.assertFalse(
            (msg.arm_status & mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC),
            "FC trigger flag set in arm status",
        )
        # Deploy parachute
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            mavutil.mavlink.PARACHUTE_RELEASE,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_DO_PARACHUTE, mavutil.mavlink.MAV_RESULT_FAILED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertFalse(msg.deployment_status, f"Parachute deployed: {msg}")

    def test_deploy_parachute_with_fc(self):
        # Arm FC trigger
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC,
            mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_SET_PARACHUTE_ARM, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertTrue(
            (msg.arm_status & mavutil.mavlink.PARACHUTE_TRIGGER_FLAGS_FC),
            "ATS trigger flag not set in arm status",
        )
        # Deploy parachute
        encode_and_send_mavlink_command_long(
            self.parachute_connection,
            mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
            mavutil.mavlink.PARACHUTE_RELEASE,
        )
        assert_mavlink_command_ack(
            self, mavutil.mavlink.MAV_CMD_DO_PARACHUTE, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        msg = self.parachute_connection.udpin.recv_match(
            type="PARACHUTE_STATUS", blocking=True, timeout=self.msg_timeout
        )
        self.assertIsNotNone(msg, "Did not recieve a PARACHUTE_STATUS message")
        self.assertEqual(
            msg.deployment_status,
            mavutil.mavlink.PARACHUTE_DEPLOYMENT_TRIGGER_DRONE,
            f"Parachute did not deployed: {msg}",
        )


if __name__ == "__main__":
    unittest.main()
