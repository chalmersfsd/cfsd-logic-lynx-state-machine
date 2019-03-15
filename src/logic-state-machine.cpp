/*
 * Copyright (C) 2018  <Insert name here>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "logic-state-machine.hpp"

/*
float StateMachine::decode(const std::string &data) noexcept {
    std::cout << "[UDP] Got data:" << data << std::endl;
    float temp = std::stof(data);
    return temp;
}
*/

StateMachine::StateMachine(cluon::OD4Session &od4, cluon::OD4Session &od4Analog, cluon::OD4Session &od4Gpio)
  : m_od4{od4}
  , m_od4Analog{od4Analog}
  , m_od4Gpio{od4Gpio}
  , m_prevState{asState::AS_OFF}
  , m_currentState{asState::AS_OFF}
  , m_ebsState{ebsState::EBS_UNAVAILABLE}
  , m_brakeState{serviceBrakeState::BRAKE_UNAVAILABLE}
  , m_currentStateEbsInit{ebsInitState::EBS_INIT_ENTRY}
  , m_lastStateTransition{}
  , m_lastEbsInitTransition{}
  , m_initialized{false}
  , m_compressor{false}
  , m_modulesRunning{false}
  , m_lastUpdateAnalog{}
  , m_lastUpdateGpio{}

  , em_currentMission{asMission::AMI_NONE}
  , em_asms{false}
  , em_tsOn{false}
  , em_rtd{false}
  , em_goSignal{false}
  , em_finishSignal{false}
  , em_ebsSound{false}
  , em_ebsOk{false}
  , em_clampExtended{false}
  , em_vehicleSpeed{0.0f}
  , em_pressureEbsAct{0.0f}
  , em_pressureEbsLine{0.0f}
  , em_pressureServiceTank{0.0f}
  , em_pressureServiceReg{0.0f}
  , em_steerPosition{0.0f}
  , em_steerPositionRack{0.0f}
  , em_brakeDutyRequest{0U}
  , em_torqueReqLeft{0}
  , em_torqueReqRight{0}
{
  setUp();
}

StateMachine::~StateMachine()
{
  StateMachine::tearDown();
}

void StateMachine::setUp()
{
  m_initialized = true;
}

void StateMachine::tearDown()
{
}

void StateMachine::body()
{
  while(cluon::time::toMicroseconds(m_lastUpdateAnalog) == 0 || cluon::time::toMicroseconds(m_lastUpdateGpio) == 0){
    sleep(1);
  }
  m_modulesRunning = true;

  int64_t threadTime = cluon::time::toMicroseconds(cluon::time::now());
  if((((threadTime-cluon::time::toMicroseconds(m_lastUpdateAnalog)) > 500000) && ((threadTime-cluon::time::toMicroseconds(m_lastUpdateGpio)) > 1000000)) && m_modulesRunning){
      m_modulesRunning = false;
      std::cout << "[ASS-ERROR] Module has crashed. Last gpio update:" << (threadTime-cluon::time::toMicroseconds(m_lastUpdateGpio)) << "\t Last analog update: " << (threadTime-cluon::time::toMicroseconds(m_lastUpdateAnalog)) << std::endl;
  }

  if (m_currentStateEbsInit != ebsInitState::EBS_INIT_INITIALIZED){
      ebsInit();
  }

  ebsUpdate();
  stateUpdate();
  setAssi();

/*
  // m_heartbeat = !m_heartbeat;
  m_serviceBrake = 1;
  m_steeringState = m_clampExtended;
  m_brakeActual = (uint8_t) (m_asms * m_pressureServiceReg * 10);
  m_brakeTarget = m_brakeDuty/500;



  m_serviceBrakeOk = m_pressureServiceTank >= 6;
  m_ebsPressureOk = m_pressureEbsLine >= 6;
  bool systemReadyOrDriving = (m_currentState == asState::AS_DRIVING || m_currentState == asState::AS_READY);
  bool systemNotOff = (m_currentState != asState::AS_OFF);
  bool serviceBrakeLow = (m_pressureServiceTank <= 4) && systemReadyOrDriving;
  
  bool sensorDisconnected = (m_pressureEbsAct < -0.08 || m_pressureEbsLine < -0.06 || m_pressureServiceTank < -0.07);
  bool ebsPressureFail = (!m_ebsPressureOk && systemNotOff);

  if (ebsPressureFail){
    std::cout << "[ASS-EBS-ERROR] Ebs line pressure: " << m_pressureEbsLine << std::endl;
  }
  if (sensorDisconnected || ebsPressureFail || serviceBrakeLow || m_ebsInitFail){
    m_ebsFault = true;
    std::cout << "[ASS-ERROR] EBS Failure: sensorDisconnected: " << sensorDisconnected 
              << " ebsPressureFail: " << ebsPressureFail 
              << " serviceBrakeLow: " << serviceBrakeLow 
              << " m_ebsInitFail: " << m_ebsInitFail << std::endl;        
  }else{
      m_ebsFault = false;
  }
    bool steeringDiffLarge = (m_steerPosition-m_steerPositionRack) > 10 || (m_steerPosition-m_steerPositionRack) < -10;
    if (systemReadyOrDriving && (!m_clampExtended || steeringDiffLarge)){
        m_steerFault = true;
        std::cout << "[ASS-ERROR] Steering Failure: m_clampExtended: " << m_clampExtended << " steeringDiffLarge: " << steeringDiffLarge << std::endl;        

    }else{
        m_steerFault = false;
        
    };

    if ((m_pressureEbsLine > 6 && m_pressureServiceTank > 8) || m_pressureServiceTank > 9 || m_pressureServiceTank < -0.05 || m_currentState == asState::EBS_TRIGGERED){
        m_compressor = 0;
    }else if (m_pressureEbsLine < 5 || m_pressureServiceTank < 6){
        m_compressor = 1;
    }

    // Sending std messages
    cluon::data::TimeStamp sampleTime = cluon::time::now();
    int16_t senderStamp = 0;

    // GPIO Msg
    opendlv::proxy::SwitchStateRequest msgGpio;

    //Heartbeat Msg
    senderStamp = m_gpioPinHeartbeat + m_senderStampOffsetGpio;
    msgGpio.state(m_heartbeat);
	m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

    if (m_debug){
        std::cout << "[ASS-Machine] Current outputs: m_finished: " << m_finished << "\t m_shutdown: " << m_shutdown << std::endl;
    }
    // m_ebsSpeaker Msg
    if(m_ebsSpeaker != m_ebsSpeakerOld || m_refreshMsg){
        senderStamp = m_gpioPinEbsSpeaker + m_senderStampOffsetGpio;
        msgGpio.state(m_ebsSpeaker);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_ebsSpeakerOld = m_ebsSpeaker;
    }

    // m_compressor Msg
    if(m_compressor != m_compressorOld || m_refreshMsg){
        senderStamp = m_gpioPinCompressor + m_senderStampOffsetGpio;
        msgGpio.state(m_compressor);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_compressorOld = m_compressor;
    }

    // m_ebsTest Msg
    if(m_ebsTest != m_ebsTestOld || m_refreshMsg){
        senderStamp = m_gpioPinEbsRelief + m_senderStampOffsetGpio;
        msgGpio.state(m_ebsTest);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_ebsTestOld = m_ebsTest;
    }
    // m_finished Msg
    if(m_finished != m_finishedOld || m_refreshMsg){
        senderStamp = m_gpioPinFinished + m_senderStampOffsetGpio;
        msgGpio.state(m_finished);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_finishedOld = m_finished;
    }

    // m_shutdown Msg
    if(m_shutdown != m_shutdownOld || m_refreshMsg){
        senderStamp = m_gpioPinShutdown + m_senderStampOffsetGpio;
        msgGpio.state(m_shutdown);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_shutdownOld = m_shutdown;
    }

    // m_serviceBrake
    if(m_serviceBrake != m_serviceBrakeOld || m_refreshMsg){
        senderStamp = m_gpioPinServiceBrake + m_senderStampOffsetGpio;
        msgGpio.state(m_serviceBrake);
        m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
        m_serviceBrakeOld = m_serviceBrake;
    }

    // Send pwm Requests
    opendlv::proxy::PulseWidthModulationRequest msgPwm;
 
    if (m_redDuty != m_redDutyOld || m_refreshMsg){
        senderStamp = m_pwmPinAssiRed + m_senderStampOffsetPwm;
        msgPwm.dutyCycleNs(m_redDuty);
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_redDutyOld = m_redDuty;
    }
    if (m_greenDuty != m_greenDutyOld || m_refreshMsg){
        senderStamp = m_pwmPinAssiGreen + m_senderStampOffsetPwm;
        msgPwm.dutyCycleNs(m_greenDuty);
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_greenDutyOld = m_greenDuty;
    }
    if (m_blueDuty != m_blueDutyOld || m_refreshMsg){
        senderStamp = m_pwmPinAssiBlue + m_senderStampOffsetPwm;
        msgPwm.dutyCycleNs(m_blueDuty);
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_blueDutyOld = m_blueDuty;
    }
    if (m_brakeDuty != m_brakeDutyOld || m_refreshMsg){
        senderStamp = m_pwmPinBrake + m_senderStampOffsetPwm;
        msgPwm.dutyCycleNs(m_brakeDuty);
        m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
        m_brakeDutyOld = m_brakeDuty;
    }

    //Send Current state of state machine
    opendlv::proxy::SwitchStateReading msgGpioRead;

    senderStamp = m_senderStampCurrentState;
    msgGpioRead.state((uint16_t) m_currentState);
    m_od4.send(msgGpioRead, sampleTime, senderStamp);
    m_od4Gen.send(msgGpioRead, sampleTime, senderStamp);

    senderStamp = m_senderStampRTD;
    msgGpioRead.state((uint16_t) m_rtd);
    m_od4.send(msgGpioRead, sampleTime, senderStamp);

    senderStamp = m_senderStampEBSFault;
    msgGpioRead.state((uint16_t) m_ebsFault);
    m_od4.send(msgGpioRead, sampleTime, senderStamp);

    senderStamp = 1413;
    msgGpioRead.state((uint16_t) m_steeringState);
    m_od4.send(msgGpioRead, sampleTime, senderStamp);

    senderStamp = 1414;
    msgGpioRead.state((uint16_t) m_ebsState);
    m_od4.send(msgGpioRead, sampleTime, senderStamp);

    senderStamp = 1415;
    msgGpioRead.state((uint16_t) m_serviceState);
    m_od4.send(msgGpioRead, sampleTime, senderStamp);

    opendlv::proxy::TorqueRequest msgTorqueReq;

    senderStamp = 1500;
    msgTorqueReq.torque(m_torqueReqLeftCan);
    m_od4.send(msgTorqueReq, sampleTime, senderStamp);

    senderStamp = 1501;
    msgTorqueReq.torque(m_torqueReqRightCan);
    m_od4.send(msgTorqueReq, sampleTime, senderStamp);


    opendlv::proxy::PressureReading pressureReadingMsg; 

    senderStamp = 1509;
    pressureReadingMsg.pressure(m_brakeTarget);
    m_od4.send(pressureReadingMsg, sampleTime, senderStamp);

    senderStamp = 1510;
    pressureReadingMsg.pressure(m_brakeActual);
    m_od4.send(pressureReadingMsg, sampleTime, senderStamp);

    m_refreshMsg = m_flash2Hz;
    */
}

void StateMachine::stateUpdate()
{
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();

  switch(m_currentState) {
    case asState::AS_OFF:
      if (em_currentMission && m_ebsState == EBS_ARMED && em_asms && em_tsOn) { // TODO: Check that external tsOn button is used here
        m_prevState = asState::AS_OFF;
        m_currentState = asState::AS_READY;
        m_lastStateTransition = timeMillis;
      } else if (em_currentMission == AMI_MANUAL && m_ebsState == EBS_UNAVAILABLE && !em_asms && em_tsOn) {
        m_prevState = asState::AS_OFF;
        m_currentState = asState::AS_MANUAL;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_READY:
      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_READY;
        m_currentState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (!em_asms && m_brakeState == BRAKE_UNAVAILABLE) { //TODO: Check that brake pressure is zero instead
        m_prevState = asState::AS_READY;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      } else if ((timeMillis - m_lastStateTransition > 5000) && em_rtd) { //TODO: Verify that the timer is correct
        m_prevState = asState::AS_READY;
        m_currentState = asState::AS_DRIVING;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_DRIVING:
      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_DRIVING;
        m_currentState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (em_finishSignal && em_vehicleSpeed < 0.05f) {
        m_prevState = asState::AS_DRIVING;
        m_currentState = asState::AS_FINISHED;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_FINISHED:
      if (m_ebsState == EBS_ACTIVATED) { // TODO: Check if ebs_activated and RES triggered can be considered the same
        m_prevState = asState::AS_FINISHED;
        m_currentState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (!em_asms && m_brakeState == BRAKE_UNAVAILABLE) { //TODO: Check that brake pressure is zero instead
        m_prevState = asState::AS_FINISHED;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_EMERGENCY:
      if (!em_ebsSound && !em_asms && m_brakeState == BRAKE_UNAVAILABLE) { //TODO: Check that brake pressure is zero instead
        m_prevState = asState::AS_EMERGENCY;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      } 
      break;

    case asState::AS_MANUAL:
      if (!em_tsOn) {
        m_prevState = asState::AS_MANUAL;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      }
      break;

    default:
      break;
  }
}

void StateMachine::setAssi()
{
  // TODO: set all assi values here depending on current state
}

void StateMachine::ebsInit()
{
  // TODO: figure out if m_ebsTest is needed or not
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();

  switch (m_currentStateEbsInit)
  {
  case ebsInitState::EBS_INIT_ENTRY:
    if (em_asms && em_ebsOk)
    {
      m_currentStateEbsInit = ebsInitState::EBS_INIT_CHARGING;
      m_lastEbsInitTransition = timeMillis;
    }
    break;

  case ebsInitState::EBS_INIT_CHARGING:
    if (em_pressureEbsAct >= 5 && em_pressureEbsLine >= 5 && em_pressureServiceTank >= 6 && em_pressureServiceReg >= 3)
    {
      m_compressor = false;
      m_currentStateEbsInit = ebsInitState::EBS_INIT_COMPRESSOR;
      m_lastEbsInitTransition = timeMillis;
    }
    else if (((m_lastEbsInitTransition + 5000) <= timeMillis) && (em_pressureEbsAct <= 0.5 || em_pressureServiceReg <= 0.5))
    {
      std::cout << "[EBS-Init] Failed to increase pressure above 0.5bar in 5s."
                << " m_pressureEbsAct: " << em_pressureEbsAct
                << " m_pressureServiceReg: " << em_pressureServiceReg << std::endl;
      m_currentStateEbsInit = ebsInitState::EBS_INIT_FAILED;
      m_lastEbsInitTransition = timeMillis;
    }
    else if (((m_lastEbsInitTransition + 60000) <= timeMillis))
    {
      std::cout << "[EBS-Init] Failed to increase pressures 60s." << std::endl;
      m_currentStateEbsInit = ebsInitState::EBS_INIT_FAILED;
      m_lastEbsInitTransition = timeMillis;
    }
    break;

  case ebsInitState::EBS_INIT_COMPRESSOR:
    if (!m_compressor)
    {
      std::cout << "[EBS-Init] Initialisation done" << std::endl;
      m_currentStateEbsInit = ebsInitState::EBS_INIT_INITIALIZED;
      m_lastEbsInitTransition = timeMillis;
    }
    break;

  default:
    break;
  }

  // TODO: Add VERBOSE
  if (true)
  {
    std::cout << "[EBS-Init] Current EBS Init state: " << m_currentStateEbsInit << std::endl;
  }
}

void StateMachine::ebsUpdate()
{
  // TODO: Write update for m_ebsState here
}

asState StateMachine::getCurrentState()
{
  return m_currentState;
}

bool StateMachine::getInitialized() {return m_initialized;}
uint32_t StateMachine::getSenderStampOffsetAnalog() {return m_senderStampOffsetAnalog;}
uint32_t StateMachine::getSenderStampOffsetGpio() {return m_senderStampOffsetGpio;}
uint16_t StateMachine::getAnalogStampEbsLine() {return m_analogStampEbsLine;}
uint16_t StateMachine::getAnalogStampServiceTank() {return m_analogStampServiceTank;}
uint16_t StateMachine::getAnalogStampEbsActuator() {return m_analogStampEbsActuator;}
uint16_t StateMachine::getAnalogStampPressureReg() {return m_analogStampPressureReg;}
uint16_t StateMachine::getGpioStampEbsOk() {return m_gpioStampEbsOk;}
uint16_t StateMachine::getGpioStampAsms() {return m_gpioStampAsms;}
uint16_t StateMachine::getGpioStampClampSensor() {return m_gpioStampClampSensor;}

void StateMachine::setLastUpdateAnalog(cluon::data::TimeStamp lastUpdateAnalog)
{
  m_lastUpdateAnalog = lastUpdateAnalog;
}

void StateMachine::setLastUpdateGpio(cluon::data::TimeStamp lastUpdateGpio)
{
  m_lastUpdateGpio = lastUpdateGpio;
}

void StateMachine::setMission(uint16_t mission)
{
  em_currentMission = static_cast<asMission>(mission);
}

void StateMachine::setAsms(bool asms)
{
  em_asms = asms;
}

void StateMachine::setTsOn(bool tsOn)
{
  em_tsOn = tsOn;
}

void StateMachine::setRtd(bool rtd)
{
  em_rtd = rtd;
}

void StateMachine::setGoSignal(bool goSignal)
{
  em_goSignal = goSignal;
}

void StateMachine::setFinishSignal(bool finishSignal)
{
  em_finishSignal = finishSignal;
}

void StateMachine::setEbsSound(bool ebsSound)
{
  em_ebsSound = ebsSound;
}

void StateMachine::setEbsOk(bool ebsOk)
{
  em_ebsOk = ebsOk;
}

void StateMachine::setClampExtended(bool clampExtended)
{
  em_clampExtended = clampExtended;
}

void StateMachine::setVehicleSpeed(float vehicleSpeed)
{
  em_vehicleSpeed = vehicleSpeed;
}

void StateMachine::setPressureEbsAct(float pressure)
{
  em_pressureEbsAct = pressure;
}

void StateMachine::setPressureEbsLine(float pressure)
{
  em_pressureEbsLine = pressure;
}

void StateMachine::setPressureServiceTank(float pressure)
{
  em_pressureServiceTank = pressure;
}

void StateMachine::setPressureServiceReg(float pressure)
{
  em_pressureServiceReg = pressure;
}

void StateMachine::setSteerPosition(float pos)
{
  em_steerPosition = pos;
}

void StateMachine::setSteerPositionRack(float pos)
{
  em_steerPosition = pos;
}

void StateMachine::setBrakeDutyCycle(uint32_t duty)
{
  em_brakeDutyRequest = duty;
}

void StateMachine::setTorqueReqLeft(int16_t torque)
{
  em_torqueReqLeft = torque;
}

void StateMachine::setTorqueReqRight(int16_t torque)
{
  em_torqueReqRight = torque;
}