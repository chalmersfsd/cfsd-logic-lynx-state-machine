# cfsd-logic-lynx-state-machine
This microservice provides the state machine for Lynx. Receives messages from different microservices and sets the appropriate autonomous state and state specific behaviour.

### Build
AMD64: docker build -f Dockerfile.amd64 -t chalmersfsd/state-machine:v0.0.0 .

### Input
 - Senderstamp Offsets
  - GPIO: 1000
  - Analog: 1200
  - PWM: 1300
 - opendlv::proxy::SwitchStateReading
  - ASMS on/off (stamp = 1115)
  - EBS okay true/false (stamp = 1049)
  - EBS sound on/off (stamp = 1044)
  - TS on/off (stamp = ???) (same as EBS okay for now)
  - RTD true/false (stamp = 1404)
  - Go signal true/false (stamp = 1410)
  - Finish signal true/false (stamp = 1403)
  - Steering clamp extended true/false (stamp = 1112)
  - AS mission (stamp = 1406)

- opendlv::proxy::GroundSpeedReading
  - Vehicle Speed (stamp = ???)
or
- opendlv::proxy::WheelSpeedReading
  - Wheel RR (stamp = 1901)
  - Wheel RL (stamp = 1902)
  - Wheel FR (stamp = 1903)
  - Wheel FL (stamp = 1904)

- opendlv::proxy::PressureReading
  - Pneumatic pressure reading: EBS actuator (stamp = 1203)
  - Pneumatic pressure reading: EBS line (stamp = 1201)
  - Pneumatic pressure reading: service tank (stamp = 1202)
  - Pneumatic pressure reading: service regulator (stamp = 1205)

- opendlv::proxy::TorqueRequest
  - Torque request right (stamp = 1501)
  - Torque request left (stamp = 1500)

- opendlv::proxy::GroundSteeringReading
  - Steering position (stamp = 1200)
  - Steering rack position (stamp = 1206)

- opendlv::proxy::PulseWidthModulationRequest
  - Service brake duty cycle (stamp = 1341)

### Output
 - opendlv::proxy::SwitchStateReading
  - Heartbeat (od4Gpio, stamp = 1027)
  - Finished signal (od4Gpio, stamp = 1066)
  - Shutdown signal (od4Gpio, stamp = )
 - ASSI red signal (od4Pwm)
 - ASSI green signal (od4Pwm)
 - ASSI blue signal (od4Pwm)
 - ASSI brake signal (od4Pwm)
 - Current AS state (od4/od4Gen)
 - RTD (od4)
 - EBS fault (od4)
 - Steering state (od4, stamp = 1413)
 - EBS state (od4, stamp = 1414)
 - Service brake state (od4, stamp = 1415)
 - Torque request left (od4, stamp = 1500)
 - Torque request right (od4, stamp = 1501)
 - m_brakeTarget (od4, stamp = 1509)
 - m_brakeActual (od4, stamp = 1510)
 - m_ebsSpeaker (od4Gpio)
 - m_compressor (od4Gpio, stamp = 1045)

 - Unclear if needed / used:
  - m_ebsTest (od4Gpio)
  - m_serviceBrake (od4Gpio)

Senderstamps:

GPIO:
1026 - Steering H-bridge select
1045 - Compressor On
1046 - Rack Right movement
1047 - Rack Left movement
1049 - Service Brake ON
1061 - Relief_GND_Pulldown
1062 - SPARE_3
1065 - Clamp cylinder
1066 - Finish flag from AS
1067 - Shutdown Off from AS
1068 - Redundency for EBS
1069 - Service Brake ON/OFF
1112 - Steering engaged
1115 - ASMS


Analog:
1200 - Position Steering actuator
1201 - Pressure EBS tank
1202 - Pressure Service brake tank
1203 - Pressure Pneumatic Actuator
1205 - Pressure regulator sensor
1206 - Position Rack


Pwm:
1300 - ASSI Blue
1320 - ASSI Red
1321 - ASSI Green
1340 - Rack speed movement
1341 - Pressure Regulator Service brake


State machine:
1401 - AS state
1402 - Go Signal
1403 - Finish Signal
1404 - RTD State
1405 - EBS Fault
1407 - ResStatus
1408 - ResEStop
1409 - ResQuality
1410 - ResButtons
1411 - Cones
1412 - Cones
1413 - Steering state (SwitchStateReading)
1414 - EBS State (SwitchStateReading)
1415 - Service state (SwitchStateReading)


Torque:
1500 - Torque Request Left from state machine
1501 - Torque Request Right from state machine
1502 - Torque Request Left to state machine
1503 - Torque Request Right to state machine
1504 - Wheel spd Left
1505 - Wheel spd Right
1506 - Brake front
1507 - Brake rear
1508 - Torque actual
1509 - Brake target
1510 - Brake actual














