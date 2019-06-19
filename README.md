# cfsd-logic-lynx-state-machine
This microservice provides the state machine for Lynx. Receives messages from different microservices and sets the appropriate autonomous state and state specific behaviour.  
The state machine decides if torque and brake request should be used based on current state.
- Senderstamp Offsets
  - GPIO: 1000
  - Analog: 1200
  - PWM: 1300

### Build
AMD64: docker build -f Dockerfile.amd64 -t chalmersfsd/cfsd-logic-lynx-state-machine:v0.0.0 .

### Enumerated States
|   asState    |     asMission    |    ebsState     |
| :----------: | :--------------: | :-------------: |
|    AS_OFF    |     AMI_NONE     | EBS_UNAVAILABLE |
|   AS_READY   | AMI_ACCELERATION |    EBS_ARMED    |
|  AS_DRIVING  |    AMI_SKIDPAD   |  EBS_ACTIVATED  |
|  AS_FINISHED |  AMI_TRACKDRIVE  |                 |
| AS_EMERGENCY |   AMI_AUTOCROSS  |                 |
|  AS_MANUAL   |  AMI_BRAKETEST   |                 |
|              |  AMI_INSPECTION  |                 |
|              |    AMI_MANUAL    |                 |


### Input
|               OpenDLV message               | sender stamp |      msg content       |
| :-----------------------------------------: | :----------: | :--------------------: |
|      opendlv::proxy::SwitchStateReading     |     1049     |        EBS okay        |
|                                             |     1112     |      clampExtended     |
|                                             |     1115     |          ASMS          |
|                                             |      ???     |         TS_on          |
|                                             |     1801     |       RES Status       |
|                                             |     1802     |    RES stop signal     |
|                                             |     1804     |    RES buttons         |
|      opendlv::proxy::SwitchStateRquest      |     2102     |      Finish signal     |
|      opendlv::proxy::GroundSpeedReading     |      ???     |      vehicle speed     |
|      opendlv::proxy::WheelSpeedReading      |     1901     |        Wheel RR        |
|                                             |     1902     |        Wheel RL        |
|                                             |     1903     |        Wheel FR        |
|                                             |     1904     |        Wheel FL        |
|      opendlv::proxy::PressureReading        |     1201     |        EBS line        |
|                                             |     1202     |     Service tank       |
|                                             |     1203     |      EBS actuator      |
|                                             |     1205     |   Service regulator    |
|    opendlv::cfsdProxy::TorqueRequestDual    |     2101     |      Torque request    |
|    opendlv::proxy::GroundSteeringReading    |     1200     |   steer pos actuator   |
|                                             |     1206     |     steer pos rack     |
| opendlv::proxy::PulseWidthModulationRequest |     1341     |    brake dutycycle     |


### Output
|               OpenDLV message               | sender stamp |      msg content       |
| :-----------------------------------------: | :----------: | :--------------------: |
|      opendlv::proxy::SwitchStateReading     |     1904     |            RTD         |
|                                             |     2101     |        AS state        |
|                                             |     2105     |        EBS fault       |
|                                             |     2114     |        EBS state       |
|      opendlv::proxy::SwitchStateRequest     |     1027     |        Heartbeat       |
|                                             |     1044     |      EBS speaker       |
|                                             |     1045     |      compressor        |
|                                             |     1066     |     finished signal    |
|                                             |     1067     |     shutdown signal    |
|                                             |     1069     |    brake valve state   |
|      opendlv::proxy::PressureReading        |     1509     | brake target (missing) |
|     opendlv::cfsdProxy::TorqueRequestDual   |     1910     |  torque request to CAN |
| opendlv::proxy::PulseWidthModulationRequest |     1300     |        ASSI blue       |
|                                             |     1320     |        ASSI red        |
|                                             |     1321     |        ASSI green      |

### Transition Chart