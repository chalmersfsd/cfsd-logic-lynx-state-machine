# cfsd-logic-lynx-state-machine
This microservice provides the state machine for Lynx. Receives messages from different microservices and sets the appropriate autonomous state and state specific behaviour.  
The state machine decides if torque and brake request should be used based on current state.
- Senderstamp Offsets
  - GPIO: 1000
  - Analog: 1200
  - PWM: 1300

### Build
AMD64: docker build -f Dockerfile.amd64 -t chalmersfsd/cfsd-logic-lynx-state-machine:v0.0.0 .

### Enumerated states
- asState:
  - AS_OFF
  - AS_READY,
  - AS_DRIVING
  - AS_FINISHED
  - AS_EMERGENCY
  - AS_MANUAL

- asMission:
  - AMI_NONE
  - AMI_ACCELERATION
  - AMI_SKIDPAD
  - AMI_TRACKDRIVE
  - AMI_AUTOCROSS
  - AMI_BRAKETEST
  - AMI_INSPECTION
  - AMI_MANUAL
  - AMI_TEST

- ebsState: 
  - EBS_UNAVAILABLE
  - EBS_ARMED
  - EBS_ACTIVATED


### Input
- opendlv::proxy::SwitchStateReading
  - EBS speaker             (1044)
  - EBS okay true/false     (1049)
  - Steering clamp extended (1112)
  - ASMS on/off             (1115)
  - TS on/off               (???)
  - RES status              (1801)
  - RES stop signal         (1802)
  - RES buttons             (1804)
  - RTD                     (2104)

- opendlv::proxy::SwitchStateRequest
  - Finish signal           (2102)

- opendlv::proxy::GroundSpeedReading
  - Vehicle Speed (???)
or
- opendlv::proxy::WheelSpeedReading
  - Wheel RR (1901)
  - Wheel RL (1902)
  - Wheel FR (1903)
  - Wheel FL (1904)

- opendlv::proxy::PressureReading
  - Pneumatic pressure: EBS line          (1201)
  - Pneumatic pressure: service tank      (1202)
  - Pneumatic pressure: EBS actuator      (1203)
  - Pneumatic pressure: service regulator (1205)

- opendlv::cfsdProxy::TorqueRequestDual
  - Torque request  (2101)

- opendlv::proxy::GroundSteeringReading
  - Steering position       (1200)
  - Steering rack position  (1206)

- opendlv::proxy::PulseWidthModulationRequest
  - Service brake duty cycle (1341)

### Output
- opendlv::proxy::SwitchStateRequest
  - Heartbeat       (1027)
  - EBS speaker     (1044)
  - Compressor      (1045)
  - Finished signal (1066)
  - Shutdown signal (1067)

- opendlv::proxy::PulseWidthModulationRequest
  - ASSI blue signal  (1300)
  - ASSI red signal   (1320)
  - ASSI green signal (1321)

- opendlv::proxy::SwitchStateReading
  - RTD                        (1904)
  - Current AS state           (2101)
  - EBS fault                  (2105)
  - EBS state                  (2114)
  - Service brake valve state  (2115)

- opendlv::cfsdProxy::TorqueRequestDual
  - Torque request  (1910)

- opendlv::proxy::PressureReading
  - Brake Target (1509) (not yet implemented)