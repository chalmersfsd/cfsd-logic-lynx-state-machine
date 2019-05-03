/*
 * Copyright (C) 2018  Love Mowitz
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
#include "enum-print.hpp"
/*
float StateMachine::decode(const std::string &data) noexcept {
    std::cout << "[UDP] Got data:" << data << std::endl;
    float temp = std::stof(data);
    return temp;
}
*/

StateMachine::StateMachine(cluon::OD4Session &od4, cluon::OD4Session &od4Analog, cluon::OD4Session &od4Gpio, cluon::OD4Session &od4Pwm, bool verbose)
  : m_od4{od4}
  , m_od4Analog{od4Analog}
  , m_od4Gpio{od4Gpio}
  , m_od4Pwm{od4Pwm}
  , m_lastUpdateAnalog{}
  , m_lastUpdateGpio{}
  , m_prevState{asState::AS_OFF}
  , m_currentState{asState::AS_OFF}
  , m_ebsState{ebsState::EBS_UNAVAILABLE}
  , m_brakeState{serviceBrakeState::BRAKE_UNAVAILABLE}
  , m_currentStateEbsInit{ebsInitState::EBS_INIT_ENTRY}
  , m_lastStateTransition{0U}
  , m_lastEbsInitTransition{0U}
  , m_nextFlashTime{0U}
  , m_ebsActivatedTime{0U}
  , m_brakeDuty{0U}
  , m_brakeDutyOld{0U}
  , m_blueDuty{0U}
  , m_blueDutyOld{0U}
  , m_greenDuty{0U}
  , m_greenDutyOld{0U}
  , m_redDuty{0U}
  , m_redDutyOld{0U}
  , m_torqueReqLeftCan{0U}
  , m_torqueReqRightCan{0U}
  , m_brakeActual{0U}
  , m_brakeTarget{0U}
  , m_initialized{false}
  , m_compressor{false}
  , m_compressorOld{false}
  , m_modulesRunning{false}
  , m_serviceBrake{false}
  , m_serviceBrakeOld{false}
  , m_serviceBrakePressureOk{false}
  , m_ebsPressureOk{false}
  , m_steerFault{false}
  , m_ebsFault{false}
  , m_flash2Hz{false}
  , m_rtd{false}
  , m_firstCycleAsOff{true}
  , m_finished{false}
  , m_finishedOld{false}
  , m_shutdown{false}
  , m_shutdownOld{false}
  , m_ebsSpeaker{false}
  , m_ebsSpeakerOld{false}
  , m_heartbeat{false}
  , m_refreshMsg{true}
  , m_brakesReleased{false}
  , m_verbose{verbose}

  , em_currentMission{asMission::AMI_NONE}
  , em_asms{false}
  , em_tsOn{false}
  , em_resGoSignal{false}
  , em_resStopSignal{true}
  , em_finishSignal{false}
  , em_ebsOk{false}
  , em_clampExtended{false}
  , em_resStatus{false} // TODO: add a guard using this
  , em_resInitialized{false}
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
  std::cout << "Initializing state machine... ";
  m_initialized = true;

  cluon::data::TimeStamp sampleTime = cluon::time::now();
  int32_t senderStamp = m_senderStampResInitialize;

  // First part of handshake with RES CAN service
  opendlv::proxy::SwitchStateRequest msgCan;
  msgCan.state(m_initialized);
  m_od4.send(msgCan, sampleTime, senderStamp);
  
  std::cout << "done." << std::endl;
}

void StateMachine::tearDown()
{
}

void StateMachine::body()
{
  bool frontNodeOk = cluon::time::toMicroseconds(m_lastUpdateAnalog) != 0 && cluon::time::toMicroseconds(m_lastUpdateGpio) != 0;
  // Check if we're receiving data from AS node
  if(!frontNodeOk || !em_resInitialized){
    std::cout << "Front node status: " << (frontNodeOk ? "On\n" : "Off\n") << "RES status: " << (em_resInitialized ? "On" : "Off") << std::endl;
    return;
  }
  m_modulesRunning = true;

  // Check if we're continuously receiving data from AS node
  int64_t threadTime = cluon::time::toMicroseconds(cluon::time::now());
  if((((threadTime-cluon::time::toMicroseconds(m_lastUpdateAnalog)) > 500000) || ((threadTime-cluon::time::toMicroseconds(m_lastUpdateGpio)) > 1000000)) && m_modulesRunning){
      m_modulesRunning = false;
      std::cout << "[ASS-ERROR] Module has crashed. Last gpio update:" << (threadTime-cluon::time::toMicroseconds(m_lastUpdateGpio)) << "\t Last analog update: " << (threadTime-cluon::time::toMicroseconds(m_lastUpdateAnalog)) << std::endl;
  }

  if (em_ebsOk) { // TODO: Remove this when tsOn signal has been added to AS node
    em_tsOn = true;
  } else {
    em_tsOn = false;
  }

  if (m_currentStateEbsInit != ebsInitState::EBS_INIT_INITIALIZED){
      ebsInit();
  }

  brakeUpdate();
  stateUpdate();
  setAssi();
  sendMessages();

  // Check steering implausibility
  bool systemReadyOrDriving = (m_currentState == asState::AS_DRIVING || m_currentState == asState::AS_READY);
  bool steeringDiffLarge = std::fabs(em_steerPosition - em_steerPositionRack) > 10.0f;
  if (systemReadyOrDriving && (!em_clampExtended || steeringDiffLarge) && em_resGoSignal){
      m_steerFault = true;
      std::cout << "[ASS-ERROR] Steering Failure: m_clampExtended: " << em_clampExtended << " steeringDiffLarge: " << steeringDiffLarge << std::endl;        
  } else {
    m_steerFault = false;  
  }


}

// TODO: handle EBS error LED signal
void StateMachine::brakeUpdate()
{
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();

  m_brakeActual = static_cast<uint8_t>(em_asms * em_pressureServiceReg * 10); // TODO: Find a better way to find brake actual/target
  m_brakeTarget = m_brakeDuty/500;


  m_serviceBrakePressureOk = em_pressureServiceTank >= 6.0f;
  m_ebsPressureOk = em_pressureEbsLine >= 6.0f;
  m_brakesReleased = (em_pressureEbsAct < 0.1f && em_pressureServiceReg < 0.1f);
  bool systemReadyOrDriving = (m_currentState == asState::AS_DRIVING || m_currentState == asState::AS_READY);
  bool systemNotOff = (m_currentState != asState::AS_OFF && m_currentState != asState::AS_FINISHED);
  bool serviceBrakeLow = (em_pressureServiceTank <= 4.0f) && systemReadyOrDriving;
     
  bool sensorDisconnected = (em_pressureEbsAct < -0.08f || em_pressureEbsLine < -0.06f || em_pressureServiceTank < -0.07f);
  bool ebsPressureFail = (!m_ebsPressureOk && systemNotOff);

  // Check if the compressor should be on/off
  // TODO: Tune pressure parameters
  if ((em_pressureEbsLine > 6.0f && em_pressureServiceTank > 8.0f) || em_pressureServiceTank > 9.0f || em_pressureServiceTank < -0.05f || m_currentState == asState::AS_EMERGENCY){
    m_compressor = false;
  } else if (em_asms && (em_pressureEbsLine < 5.0f || em_pressureServiceTank < 6.0f)) {
    m_compressor = true;
  }

  if (m_currentState == AS_OFF && m_currentStateEbsInit == EBS_INIT_INITIALIZED && 
                                  m_ebsPressureOk && m_serviceBrakePressureOk && em_ebsOk && !m_compressor) {
    m_ebsState = ebsState::EBS_ARMED;
  }

  if (m_brakeState == BRAKE_AVAILABLE) {
    m_brakeDuty = ((m_lastStateTransition+500U) >= timeMillis) ? 20000U : em_brakeDutyRequest;
  } else if (m_brakeState == BRAKE_ENGAGED && em_pressureEbsAct > 0.1f) {
    m_brakeDuty = 20000U;
  } else {
    m_brakeDuty = 0U;
  }

  // Check the EBS, service and steering status if ASMS is on
  if (em_asms) {
    if (sensorDisconnected || ebsPressureFail || serviceBrakeLow || m_currentStateEbsInit == EBS_INIT_FAILED){
      m_ebsFault = true;
      std::cout << "[ASS-ERROR] EBS Failure: sensorDisconnected: " << sensorDisconnected 
              << "\n ebsPressureFail: " << ebsPressureFail 
               << "\n serviceBrakeLow: " << serviceBrakeLow 
               << "\n m_ebsInitFail: " << m_currentStateEbsInit << std::endl;        
    } else {
      m_ebsFault = false;
    }

    if((!em_ebsOk || !m_modulesRunning || m_ebsFault || m_steerFault || !em_resStopSignal) && 
                      (m_currentState == asState::AS_READY || m_currentState == asState::AS_DRIVING || m_currentState == asState::AS_FINISHED)){
      m_ebsActivatedTime = timeMillis;
      m_ebsState = ebsState::EBS_ACTIVATED;
      std::cout << "[ASS-Machine] Current state: " << m_currentState 
              << " Values: m_ebsOk: " << em_ebsOk 
              << " m_modulesRunning: " << m_modulesRunning 
              << " m_ebsFault: " << m_ebsFault 
              << " m_steerFault: " << m_steerFault 
              << " m_shutdown: " << m_shutdown
              << " m_finished: " << m_finished
              << " m_prevState: " << m_prevState 
              << " m_resStopSignal: " << em_resStopSignal << std::endl;
    }
  }
}

void StateMachine::stateUpdate()
{
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();

  m_ebsSpeaker = false;
  m_finished = false;
  m_shutdown = false;
  m_torqueReqLeftCan = 0U;
  m_torqueReqRightCan = 0U;
  m_rtd = false;

  if (m_firstCycleAsOff && !em_asms) {
    m_firstCycleAsOff = false;
    stopMission();
  } else if (!m_firstCycleAsOff && em_asms) {
    std::cout << "Running mission: \n";
    m_firstCycleAsOff = true;
    runMission();
  }

  switch(m_currentState) {
    case asState::AS_OFF:
      m_serviceBrake = false;
      em_finishSignal = false;
      em_resGoSignal = false;
      m_brakeState = serviceBrakeState::BRAKE_UNAVAILABLE;

      if (em_currentMission != AMI_NONE && em_currentMission != AMI_MANUAL && m_ebsState == EBS_ARMED && em_asms && em_tsOn) {
        m_prevState = asState::AS_OFF;
        m_currentState = asState::AS_READY;
        m_lastStateTransition = timeMillis;
      } else if (em_currentMission == AMI_MANUAL && m_ebsState == EBS_UNAVAILABLE && !em_asms && em_tsOn && !em_clampExtended) {
        m_prevState = asState::AS_OFF;
        m_currentState = asState::AS_MANUAL;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_READY:
      m_brakeState = serviceBrakeState::BRAKE_ENGAGED;
      m_serviceBrake = true;

      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_READY;
        m_currentState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (!em_asms && m_brakesReleased) {
        m_prevState = asState::AS_READY;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_currentStateEbsInit = EBS_INIT_ENTRY;
        m_ebsState = EBS_UNAVAILABLE;
      } else if ((timeMillis - m_lastStateTransition > 5000) && em_resGoSignal && em_clampExtended) {
        m_prevState = asState::AS_READY;
        m_currentState = asState::AS_DRIVING;
        m_lastStateTransition = timeMillis;
      } else {
        em_resGoSignal = false;
      }
      break;

    case asState::AS_DRIVING:
      m_brakeState = serviceBrakeState::BRAKE_AVAILABLE;
      m_torqueReqLeftCan = em_torqueReqLeft;
      m_torqueReqRightCan = em_torqueReqRight;
      m_rtd = true;
      
      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_DRIVING;
        m_currentState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (em_finishSignal && em_vehicleSpeed < 0.01f) {
        m_prevState = asState::AS_DRIVING;
        m_currentState = asState::AS_FINISHED;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_FINISHED:
      m_brakeState = serviceBrakeState::BRAKE_ENGAGED;
      m_ebsState = ebsState::EBS_ACTIVATED;
      m_finished = true;
      
      if (!em_resStopSignal) {
        m_prevState = asState::AS_FINISHED;
        m_currentState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (!em_asms && m_brakesReleased) {
        m_prevState = asState::AS_FINISHED;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_currentStateEbsInit = ebsInitState::EBS_INIT_ENTRY;
        m_ebsState = ebsState::EBS_UNAVAILABLE;
      }
      break;

    case asState::AS_EMERGENCY:
      m_ebsSpeaker = ((m_ebsActivatedTime+9000) >= timeMillis);
      m_finished = false;
      m_shutdown = true;

      if (!m_ebsSpeaker && !em_asms && m_brakesReleased) {
        m_prevState = asState::AS_EMERGENCY;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_currentStateEbsInit = ebsInitState::EBS_INIT_ENTRY;
        m_ebsState = ebsState::EBS_UNAVAILABLE;
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

  if (m_verbose) {
    std::cout << "[AS-state] Current AS state: " << m_currentState 
              << "\nebsOk: " << em_ebsOk
              << "\nASMS: " << em_asms
              << "\nMission: " << em_currentMission
              << "\nEBS state: " << m_ebsState
              << "\nBrake state: " << m_brakeState 
              << "\nCompressor: " << m_compressor 
              << "\nSteerClamp: " << em_clampExtended
              << "\nEBS pressure line: " << em_pressureEbsLine
              << "\nEBS pressure act: " << em_pressureEbsAct
              << "\nService pressure tank: " << em_pressureServiceTank
              << "\nService pressure reg: " << em_pressureServiceReg 
              << "\nbrakeDuty: " << m_brakeDuty
              << "\nbrakeDutyRequest: " << em_brakeDutyRequest
              << "\n" << std::endl;
  }

}

// TODO: Also check hydraulic brake pressure during this startup procedure
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
    if (em_asms)
    {
      m_currentStateEbsInit = ebsInitState::EBS_INIT_CHARGING;
      m_lastEbsInitTransition = timeMillis;
    }
    break;

  case ebsInitState::EBS_INIT_CHARGING:
    m_brakeDuty = 20000U;
    if (em_pressureEbsAct >= 5 && em_pressureEbsLine >= 5 && em_pressureServiceTank >= 6)
    {
      m_compressor = false;
      m_currentStateEbsInit = ebsInitState::EBS_INIT_COMPRESSOR;
      m_lastEbsInitTransition = timeMillis;
    }
    else if (((m_lastEbsInitTransition + 5000) <= timeMillis) && (em_pressureEbsAct <= 0.5)) // TODO: check EBS_line too
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

  if (m_verbose)
  {
    std::cout << "[EBS-Init] Current EBS Init state: " << m_currentStateEbsInit << std::endl;
  }
}

void StateMachine::setAssi()
{
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();

  if (m_nextFlashTime <= timeMillis){
    m_flash2Hz  = !m_flash2Hz;
    m_nextFlashTime = timeMillis + 500U; // TODO: Make better solution
  }

  if (false){
    std::cout << "[ASSI-Time] Time: " << timeMillis << "ms \t2Hz toogle:" << m_flash2Hz << std::endl;
  } 


  switch(m_currentState){
    case asState::AS_OFF:
      m_blueDuty = 0;
      m_greenDuty = 0;
      m_redDuty = 0;
      break;
    case asState::AS_READY:
      m_blueDuty = 0;
      m_greenDuty = 1000000000;
      m_redDuty = 1000000000;
      break;
    case asState::AS_DRIVING:
      m_blueDuty = 0;
      m_greenDuty = 1000000000*m_flash2Hz;
      m_redDuty = 1000000000*m_flash2Hz;
      break;
    case asState::AS_FINISHED:
      m_blueDuty = 1000000000;
      m_greenDuty = 0;
      m_redDuty = 0;
      break;
    case asState::AS_EMERGENCY:
      m_blueDuty = 1000000000*m_flash2Hz;
      m_greenDuty = 0;
      m_redDuty = 0;
      break;
    case asState::AS_MANUAL:
      m_blueDuty = 0;
      m_greenDuty = 0;
      m_redDuty = 0;
      break;
    default:
      m_blueDuty = 0;
      m_greenDuty = 0;
      m_redDuty = 0;
      break;
  }
}

void StateMachine::sendMessages()
{
  // Sending std messages
  cluon::data::TimeStamp sampleTime = cluon::time::now();
  int16_t senderStamp = 0;

  // GPIO Msg
  opendlv::proxy::SwitchStateRequest msgGpio;

  //Heartbeat Msg
  m_heartbeat = !m_heartbeat;
  senderStamp = m_gpioStampHeartbeat + m_senderStampOffsetGpio;
  msgGpio.state(m_heartbeat);
  m_od4Gpio.send(msgGpio, sampleTime, senderStamp);

  if (false) {
    std::cout << "[ASS-Machine] Current outputs: m_finished: " << m_finished << "\t m_shutdown: " << m_shutdown << std::endl;
  }

  // m_ebsSpeaker Msg
  if (m_ebsSpeaker != m_ebsSpeakerOld || m_refreshMsg) {
    senderStamp = m_gpioStampEbsSpeaker + m_senderStampOffsetGpio;
    msgGpio.state(m_ebsSpeaker);
    m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
    m_ebsSpeakerOld = m_ebsSpeaker;
  }

  // m_compressor Msg
  if (m_compressor != m_compressorOld || m_refreshMsg) {
    senderStamp = m_gpioStampCompressor + m_senderStampOffsetGpio;
    msgGpio.state(m_compressor);
    m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
    m_compressorOld = m_compressor;
  }

  // TODO: Find out if m_ebsTest was used
  /* m_ebsTest Msg
  if (m_ebsTest != m_ebsTestOld || m_refreshMsg) {
    senderStamp = m_gpioStampEbsRelief + m_senderStampOffsetGpio;
    msgGpio.state(m_ebsTest);
    m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
    m_ebsTestOld = m_ebsTest;
  }
  */

  // m_finished Msg
  if (m_finished != m_finishedOld || m_refreshMsg) {
    senderStamp = m_gpioStampFinished + m_senderStampOffsetGpio;
    msgGpio.state(m_finished);
    m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
    m_finishedOld = m_finished;
  }

/*
  // m_shutdown Msg
  if (m_shutdown != m_shutdownOld || m_refreshMsg) {
    senderStamp = m_gpioStampShutdown + m_senderStampOffsetGpio;
    msgGpio.state(m_shutdown);
    m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
    m_shutdownOld = m_shutdown;
  }
*/
  // m_serviceBrake
  if (m_serviceBrake != m_serviceBrakeOld || m_refreshMsg) {
    senderStamp = m_gpioStampServiceBrake + m_senderStampOffsetGpio;
    msgGpio.state(m_serviceBrake);
    m_od4Gpio.send(msgGpio, sampleTime, senderStamp);
    m_serviceBrakeOld = m_serviceBrake;
  }

  // Send pwm Requests
  opendlv::proxy::PulseWidthModulationRequest msgPwm;

  if (m_redDuty != m_redDutyOld || m_refreshMsg) {
    senderStamp = m_pwmStampAssiRed + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_redDuty);
    m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
    m_redDutyOld = m_redDuty;
  }
  if (m_greenDuty != m_greenDutyOld || m_refreshMsg) {
    senderStamp = m_pwmStampAssiGreen + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_greenDuty);
    m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
    m_greenDutyOld = m_greenDuty;
  }
  if (m_blueDuty != m_blueDutyOld || m_refreshMsg) {
    senderStamp = m_pwmStampAssiBlue + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_blueDuty);
    m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
    m_blueDutyOld = m_blueDuty;
  }

  if (m_brakeDuty != m_brakeDutyOld || m_refreshMsg) {
    senderStamp = m_pwmStampBrake + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_brakeDuty);
    m_od4Pwm.send(msgPwm, sampleTime, senderStamp);
    m_brakeDutyOld = m_brakeDuty;
  }

  //Send Current state of state machine
  opendlv::proxy::SwitchStateReading msgGpioRead;

  senderStamp = m_senderStampCurrentState;
  msgGpioRead.state((uint16_t)m_currentState);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);

  senderStamp = m_senderStampRTD;
  msgGpioRead.state((uint16_t)m_rtd);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);

  senderStamp = m_senderStampEBSFault;
  msgGpioRead.state((uint16_t)m_ebsFault);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);

  /*
  senderStamp = m_senderStampSteeringState;
  msgGpioRead.state((uint16_t)em_clampExtended); // TODO: Add separate state for steering available/unavailable (m_steeringState)
  m_od4.send(msgGpioRead, sampleTime, senderStamp);
  */

  senderStamp = m_senderStampEbsState;
  msgGpioRead.state((uint16_t)m_ebsState);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);


  opendlv::proxy::TorqueRequest msgTorqueReq;

  senderStamp = m_senderStampTorqueLeft;
  msgTorqueReq.torque(m_torqueReqLeftCan);
  m_od4.send(msgTorqueReq, sampleTime, senderStamp);

  senderStamp = m_senderStampTorqueRight;
  msgTorqueReq.torque(m_torqueReqRightCan);
  m_od4.send(msgTorqueReq, sampleTime, senderStamp);

  opendlv::proxy::PressureReading pressureReadingMsg;

  senderStamp = m_senderStampBrakeTarget;
  pressureReadingMsg.pressure(m_brakeTarget);
  m_od4.send(pressureReadingMsg, sampleTime, senderStamp);

  senderStamp = m_senderStampBrakeActual;
  pressureReadingMsg.pressure(m_brakeActual);
  m_od4.send(pressureReadingMsg, sampleTime, senderStamp);

  m_refreshMsg = m_flash2Hz;
}

void StateMachine::runMission(){
  switch (em_currentMission)
  {
  case asMission::AMI_NONE:
    break;
  case asMission::AMI_ACCELERATION:
    std::cout << "Starting Acceleration mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/acceleration-up.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_SKIDPAD:
    std::cout << "Starting Skid pad mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/skidpad-up.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_AUTOCROSS:
    std::cout << "Starting Autocross mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/trackdrive-up.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_TRACKDRIVE:
    std::cout << "Starting Trackdrive mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/trackdrive-up.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_BRAKETEST:
    std::cout << "Starting Brake test mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/braketest-up.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_INSPECTION:
    std::cout << "Starting Inspection mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/inspection-up.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_MANUAL:
    std::cout << "Starting Test mission... "
              << system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/state-machine-test/test-up.sh\" &")
              << std::endl; // Docker compose up.
    break;
  default:
    break;
  }
}
void StateMachine::stopMission()
{
  switch (em_currentMission)
  {
  case asMission::AMI_NONE:
    break;

  case asMission::AMI_ACCELERATION:
    std::cout << "Stopping Acceleration mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/acceleration-down.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_SKIDPAD:
    std::cout << "Stopping Skid pad mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/skidpad-down.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_AUTOCROSS:
    std::cout << "Stopping Autocross mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/autocross-down.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_TRACKDRIVE:
    std::cout << "Stopping Trackdrive mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/trackdrive-down.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_BRAKETEST:
    std::cout << "Stopping Brake test mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/braketest-down.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_INSPECTION:
    std::cout << "Stopping Inspection mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/script/inspection-down.sh\" &")
              << std::endl; // Docker compose up.
    break;
  case asMission::AMI_MANUAL:
    std::cout << "Stopping Test mission... "
              //<< system("sshpass -p cfsd ssh -o StrictHostKeyChecking=no cfsd@127.0.0.1 \"sh /home/cfsd/state-machine-test/test-down.sh\" &")
              << std::endl; // Docker compose up.
    break;
  default:
    break;
  }
}

asState StateMachine::getCurrentState() {return m_currentState;}
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

void StateMachine::setGoSignal(bool goSignal)
{
  em_resGoSignal = goSignal;
}

void StateMachine::setStopSignal(bool stopSignal)
{
  em_resStopSignal = stopSignal;
}

void StateMachine::setFinishSignal(bool finishSignal)
{
  em_finishSignal = finishSignal;
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

void StateMachine::setResStatus(bool status)
{
  em_resStatus = status;

  if (!em_resInitialized) {
    em_resInitialized = status;
  }
}