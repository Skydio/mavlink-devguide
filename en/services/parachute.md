# Parachute Protocol

## Introduction

The parachute protocol allows MAVLink control over the behavior of a parachute that is integrated or attached to the drone. Along with this, the parachute protocol also publishes status information for developers or users. 


## MAVLink Parachute Implementations

These parachutes have built-in MAVLink support:

- AVSS Parachute for X10

## Message/Enum Summary

| Message                                                                                                              | Description               |
| ------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| <span id="PARACHUTE_STATUS"></span>[PARACHUTE_STATUS](../messages/common.md#PARACHUTE_STATUS)                                           | Current status of the parachute. Recommended to publish this at a regular rate.  
| <span id="MAV_CMD_SET_PARACHUTE_ARM"></span>[MAV_CMD_SET_PARACHUTE_ARM](../messages/common.md#MAV_CMD_SET_PARACHUTE_ARM)                | Command to arm/disarm parachute module trigger sources.|

| Enum Values                                                                                                              | Description               |
| ------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| <span id="MAV_TYPE_PARACHUTE"></span>[MAV_TYPE_PARACHUTE](../messages/minimal.md#MAV_TYPE_PARACHUTE)                             | Type of the component (parachute).                   |
| <span id="MAV_COMP_ID_PARACHUTE"></span>[MAV_COMP_ID_PARACHUTE](../messages/minimal.md#MAV_COMP_ID_PARACHUTE)                           | ID of the component (parachute).   


| Enum                                                                                                               | Description               |
| ------------------------------------------------------------------------------------------------------------------ | ------------------------- |
| <span id="PARACHUTE_TRIGGER_FLAGS"></span>[PARACHUTE_TRIGGER_FLAGS](../messages/common.md#PARACHUTE_TRIGGER_FLAGS) | Parachute trigger sources.        |
| <span id="PARACHUTE_DEPLOYMENT_TRIGGER"></span>[PARACHUTE_DEPLOYMENT_TRIGGER](../messages/common.md#PARACHUTE_DEPLOYMENT_TRIGGER) | Parachute deployment trigger source. |
| <span id="PARACHUTE_SAFETY_FLAGS"></span>[PARACHUTE_SAFETY_FLAGS](../messages/common.md#PARACHUTE_SAFETY_FLAGS) | Parachute module safety-related flags. |
| <span id="PARACHUTE_ERROR_FLAGS"></span>[PARACHUTE_ERROR_FLAGS](../messages/common.md#PARACHUTE_ERROR_FLAGS) | Fault/health indications. |

## Implementation and Messages

### Parachute Connection

Parachutes are expected to follow the [Heartbeat/Connection Protocol](https://github.com/mavlink/mavlink-devguide/blob/master/en/services/heartbeat.md) and send a constant flow of heartbeats (nominally at 1Hz). Parachutes are identified via their type [MAV_TYPE_PARACHUTE](#MAV_TYPE_PARACHUTE).
Individual parachutes are distinguished via their unique component ID, which by default should be [MAV_COMP_ID_PARACHUTE](#MAV_COMP_ID_PARACHUTE) (though this is not mandated and any ID may be used).

Once a heartbeat is received, the drone can then send a [MAV_CMD_REQUEST_MESSAGE](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE) command to the parachute to receive information about its status. [MAV_CMD_SET_PARACHUTE_ARM](#MAV_CMD_SET_PARACHUTE_ARM) can also then be sent to configure the arming status of the various possible trigger mechanisms of the parachute. 


### STATUS
The [PARACHUTE_STATUS](#PARACHUTE_STATUS) message can be requested to receive information about the status of the parachute. This includes information such as uptime, errors, arm status, deployment status, safety status, arm altitiude, and parachute pack date.

The parameter list can be found below:
| Parameter                | Description                                                                                                                                    |
|--------------------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| `time_boot_ms`           | Time since system boot up in milliseconds.                                                                                                     |
| `error_status`           | Bitmask detailing the [PARACHUTE_ERROR_FLAGS](#PARACHUTE_ERROR_FLAGS).                                                                         |
| `arm_status`             | Bitmask detailing the arming status via [PARACHUTE_TRIGGER_FLAGS](#PARACHUTE_TRIGGER_FLAGS).                                                   |
| `deployment_status`      | Parachute deployment status.                                                                                                                   |
| `safety_status`          | Parachute safety status.                                                                                                                       |
| `ats_arm_altitude`       | Parachute Automatic Trigger System (ATS) auto-arming/disarming altitude in meters.                                                             |
| `parachute_packed_date`  | Parachute packed date (YYYY-MM-DD) in ASCII characters, 0 terminated. All 0: field not provided.                                               |


#### PARACHUTE_ERROR_FLAGS
[PARACHUTE_ERROR_FLAGS](#PARACHUTE_ERROR_FLAGS) is a bitmask that details the various critical errors that may occur within the parachute module or within the communication between the parachute and the drone. This is published in the [PARACHUTE_STATUS](#PARACHUTE_STATUS) message.

A list of the various errors can be found below:
| Value    | Parameter                                    | Description                                                                                                     |
|----------|----------------------------------------------|-----------------------------------------------------------------------------------------------------------------|
| 1        | `PARACHUTE_ERROR_FLAGS_BAROMETER_ERROR`      | There is an error with the parachute barometer.                                                                  |
| 2        | `PARACHUTE_ERROR_FLAGS_IMU_ERROR`            | There is an error with the parachute IMU.                                                                        |
| 4        | `PARACHUTE_ERROR_FLAGS_RF_CONNECTION_ERROR`  | There is an error with the parachute's RF that is used for manual control.                                       |
| 8        | `PARACHUTE_ERROR_FLAGS_LOW_POWER`            | Parachute module has low power.                                                                                  |
| 16       | `PARACHUTE_ERROR_FLAGS_FC_CONNECTION_ERROR`  | There is an error with the connection between parachute and flight controller (FC).                              |
| 32       | `PARACHUTE_ERROR_FLAGS_EFTS_CONNECTION_ERROR`| There is an error with the connection between parachute and Electrical Flight Termination System (EFTS).         |
| 64       | `PARACHUTE_ERROR_FLAGS_POD_CONNECTION_ERROR` | There is an error with the parachute pod.                                                                        |
| 128      | `PARACHUTE_ERROR_FLAGS_EFTS_DIAGNOSE`        | Parachute Electrical Flight Termination System (EFTS) diagnosis failed.                                          |
| 256      | `PARACHUTE_ERROR_FLAGS_CHARGING_FAILED`      | Parachute module charging failed.                                                                                |
| 512      | `PARACHUTE_ERROR_FLAGS_EXTERNAL_POWER_ERROR` | There is an error with the parachute external power source.                                                      |
| 1024     | `PARACHUTE_ERROR_FLAGS_GS_CONNECTION_ERROR`  | There is an error with the connection between parachute and Ground Station (GS).                                 |
| 2048     | `PARACHUTE_ERROR_FLAGS_GPS_ERROR`            | There is an error with the parachute's GPS.                                                                      |
| 4096     | `PARACHUTE_ERROR_FLAGS_SUBSYSTEM_CONNECTION_ERROR` | There is an error with the connection between parachute and subsystem (e.g. remote controller, expansion board, etc.). |
| 8192     | `PARACHUTE_ERROR_FLAGS_SUBSYSTEM_FW_ERROR`   | There is an error with the parachute subsystem firmware (e.g. wrong firmware version).                           |
| 16384    | `PARACHUTE_ERROR_FLAGS_RESERVED_1`           | Reserved for future use.                                                                                         |
| 32768    | `PARACHUTE_ERROR_FLAGS_RESERVED_2`           | Reserved for future use.                                                                                         |
| 65536    | `PARACHUTE_ERROR_FLAGS_LOGGING_ERROR`        | There is an error with the parachute's internal logging system.                                                  |
| 131072   | `PARACHUTE_ERROR_FLAGS_MODULE_RETIRED`       | This parachute module is retired (i.e. too many deployments).                                                    |
| 262144   | `PARACHUTE_ERROR_FLAGS_GLOW_WIRE_ERROR`      | There is an error with the parachute glow wire.                                                                  |
| 524288   | `PARACHUTE_ERROR_FLAGS_OFFBOARD_CONNECTION_ERROR` | There is an error with the MAVLink connection between parachute and offboard computer.                        |
| 1048576  | `PARACHUTE_ERROR_FLAGS_IMU_CALIBRATION_ERROR`| Parachute's internal IMU calibration failed.                                                                     |


#### MAV_CMD_SET_PARACHUTE_ARM
This command can be used to arm/disarm the various parachute trigger sources. By configuring the arm/disarm setting of the various trigger sources, a user is able to change the behavior of the parachute when it is being used. The operation follows the normal [Command Protocol](https://github.com/mavlink/mavlink-devguide/blob/master/en/services/command.md) rules for command/acknowledgment.

There are two parameters for this command. The first parameter is used to indicate the arm/disarm setting. Setting a bit to "0" indicated disarm and setting a bit to "1" indicates arm. The second parameter is a mask-byte that indicates which trigger source is being configured. The bits of the second parameter are mapped to [PARACHUTE_TRIGGER_FLAGS](#PARACHUTE_TRIGGER_FLAGS). The second parameter allows users to not have to track the status of all trigger sources when you want to set arm/disarm for specific sources. 

For example, the user wants to arm the automatic trigger system and flight controller trigger, and the user wants to disarm the manual and geofence trigger. 
- The first parameter would be `0b01100`.
   - Bit 0 for `PARACHUTE_TRIGGER_FLAGS_MANUAL` is set to "0" to disarm.
   - Bit 1 for `PARACHUTE_TRIGGER_FLAGS_ATS` is set to "1" to arm.
   - Bit 2 for `PARACHUTE_TRIGGER_FLAGS_FC` is set to "1" to arm.
   - Bit 4 for `PARACHUTE_TRIGGER_FLAGS_GEOFENCE` is set to "0" to disarm.
- The second parameter would be `0b11001`.
   - Since bit 0, 1, 2, and 4 are being configured and no other trigger sources are being configured, all of those bits are set to "1" to indicate that those specific trigger sources need their arm statuses to be updated.
   - Although bit 3 in the first parameter is set to "0", since bit 3 is set to "0" in the second parameter, the parachute should ignore the bit in the first parameter and not change the arm/disarm status of the trigger source mapped to bit 3.


##### PARACHUTE_TRIGGER_FLAGS

[PARACHUTE_TRIGGER_FLAGS](#PARACHUTE_TRIGGER_FLAGS) is a bitmask that is used in the parameters of the [MAV_CMD_SET_PARACHUTE_ARM](#MAV_CMD_SET_PARACHUTE_ARM) command and is published in the [PARACHUTE_STATUS](#PARACHUTE_STATUS) message. When published in the [PARACHUTE_STATUS](#PARACHUTE_STATUS) message, the bitmask details the various arm/disarm states of all the trigger flags.

A list of the various flags can be found below:

| Bit| Value  | Parameter                                      | Description                                                                                                            |
|----|--------|------------------------------------------------|------------------------------------------------------------------------------------------------------------------------|
| 0  | 1      | `PARACHUTE_TRIGGER_FLAGS_MANUAL`              | Manual trigger (ground based control via parachute-specific RF channel)                                                |
| 1  | 2      | `PARACHUTE_TRIGGER_FLAGS_ATS`                 | Automatic trigger system (ATS)                                                                                         |
| 2  | 4      | `PARACHUTE_TRIGGER_FLAGS_FC`                  | Flight controller trigger (e.g. MAVLink from FC, PWM, DroneCan, RC Control)                                            |
| 3  | 8      | `PARACHUTE_TRIGGER_FLAGS_OFFBOARD`            | Offboard computer trigger (via MAVLink)                                                                                |
| 4  | 16     | `PARACHUTE_TRIGGER_FLAGS_GEOFENCE`            | Geofence trigger (by parachute). Parachute uses MAVLink mission protocol to fetch geofence.                            |
| 5  | 32     | `PARACHUTE_TRIGGER_FLAGS_FTS_PRECHECKING`     | FTS (flight termination system) pre-checking protocol trigger                                                          |
| 6  | 64     | `PARACHUTE_TRIGGER_FLAGS_ATS_AUTO_ARM`        | Auto-arming of parachute automatic trigger system (ATS). This allows a parachute to enable ATS after reaching a desired altitude. |
| 7  | 128    | `PARACHUTE_TRIGGER_FLAGS_ATS_AUTO_DISARM`     | Auto-disarming of parachute automatic trigger system (ATS). This allows a parachute to disable ATS after detecting that it is below a desired altitude. |

#### PARACHUTE_DEPLOYMENT_TRIGGER
[PARACHUTE_DEPLOYMENT_TRIGGER](#PARACHUTE_DEPLOYMENT_TRIGGER) is an enum that details out the deployment status of the parachute. This enum is set to "0" when the parachute has not been deployed. All other enum values indicate that the parachute has deployed and by which source. This is published in the [PARACHUTE_STATUS](#PARACHUTE_STATUS) message.

A list of the various enum values can be found below:

| Value | Parameter                                          | Description                                                             |
|-------|----------------------------------------------------|-------------------------------------------------------------------------|
| 0     | `PARACHUTE_DEPLOYMENT_TRIGGER_NONE`                | None (parachute has not deployed)                                       |
| 1     | `PARACHUTE_DEPLOYMENT_TRIGGER_MANUAL`              | Manual                                                                  |
| 2     | `PARACHUTE_DEPLOYMENT_TRIGGER_ATS`                 | Automatic trigger system (ATS)                                          |
| 3     | `PARACHUTE_DEPLOYMENT_TRIGGER_DRONE`               | Drone                                                                   |
| 4     | `PARACHUTE_DEPLOYMENT_TRIGGER_MAVLINK`             | MAVLink, e.g. from an offboard computer                                 |
| 5     | `PARACHUTE_DEPLOYMENT_TRIGGER_GEOFENCE`            | Geofence                                                                |
| 6     | `PARACHUTE_DEPLOYMENT_TRIGGER_FTS_PRECHECKING`     | Flight Termination System (FTS) pre-checking protocol                   |


#### PARACHUTE_SAFETY_FLAGS
[PARACHUTE_SAFETY_FLAGS](#PARACHUTE_SAFETY_FLAGS) is a bitmask that details the safety status of the parachute. This is published in the [PARACHUTE_STATUS](#PARACHUTE_STATUS) message.

A breakdown of the bitmask can be found below:

| Value | Parameter                                             | Description                                                                                                     |
|-------|-------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------|
| 1     | `PARACHUTE_SAFETY_FLAGS_GROUND_CLEARED`               | The parachute module has cleared a safe distance from the ground for deployment.  |
| 2     | `PARACHUTE_SAFETY_FLAGS_ON_GROUND`                    | The parachute's own sensor has confirmed it is stably on the ground.              |
| 4     | `PARACHUTE_SAFETY_FLAGS_GEOFENCE_MISSION_SET`         | The parachute module has downloaded geofence mission successfully and can be triggered by geofence source. |


## Test Script

#### Description

The test suite included in `parachute.py` allows for testing both sides of the parachute communication interaction. The first script `parachute.py` will emulate a standard parachute module. The second script will run a standard test suite against the emulator, testing all commands listed in this document.

##### Instructions

1. Run simple parachute emulator `python3 parachute.py`
2. Run test `python3 -m unittest -v test_parachute.py`