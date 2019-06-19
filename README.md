# cfsd-logic-lynx-state-machine
This microservice provides the state machine for Lynx. Receives messages from different microservices and sets the appropriate autonomous state and state specific behaviour.
- Senderstamp Offsets
  - GPIO: 1000
  - Analog: 1200
  - PWM: 1300

### Build
AMD64: docker build -f Dockerfile.amd64 -t chalmersfsd/state-machine:v0.0.0 .

### Input
- opendlv::proxy::SwitchStateReading
  - EBS sound               (1044)
  - EBS okay true/false     (1049)
  - Steering clamp extended (1112)
  - ASMS on/off             (1115)
  - TS on/off               (???)
  - RTD                     (2104)
  - RES stop signal         (1802)
  - RES status              (1801)
  - RES buttons             (1804)

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
  - Pneumatic pressure: EBS actuator      (1203)
  - Pneumatic pressure: EBS line          (1201)
  - Pneumatic pressure: service tank      (1202)
  - Pneumatic pressure: service regulator (1205)

- opendlv::proxy::TorqueRequest
  - Torque request right  (1501)
  - Torque request left   (1500)

- opendlv::proxy::GroundSteeringReading
  - Steering position       (1200)
  - Steering rack position  (1206)

- opendlv::proxy::PulseWidthModulationRequest
  - Service brake duty cycle (1341)

### Output
- opendlv::proxy::SwitchStateRequest (od4Gpio)
  - Heartbeat       (1027)
  - Finished signal (1066)
  - Shutdown signal (1067)
  - EBS speaker     (1044)
  - Compressor      (1045)

- opendlv::proxy::PulseWidthModulationRequest (od4Pwm)
  - ASSI red signal   (1320)
  - ASSI green signal (1321)
  - ASSI blue signal  (1300)

- opendlv::proxy::SwitchStateReading (od4)
  - Current AS state           (1401)
  - RTD                        (1404)
  - EBS fault                  (1405)
  - Steering state             (1413)
  - EBS state                  (1414)
  - Service brake valve state  (1415)

- opendlv::proxy::TorqueRequest (od4)
  - Torque request left  (1502)
  - Torque request right (1503)

- opendlv::proxy::PressureReading (od4)
  - Brake Target (1509)
  - Brake Actual (1510)

- Unclear if needed / used:
  - m_ebsTest (od4Gpio)
  - m_serviceBrake (od4Gpio)