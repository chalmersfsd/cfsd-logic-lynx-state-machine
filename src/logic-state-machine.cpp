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

StateMachine::StateMachine(cluon::OD4Session &od4, bool verbose)
  : m_od4{od4}
  , m_prevState{asState::AS_OFF}
  , m_asState{asState::AS_OFF}
  , m_ebsState{ebsState::EBS_UNAVAILABLE}
  , m_brakeState{serviceBrakeState::BRAKE_UNAVAILABLE}
  , m_ebsInitState{ebsInitState::EBS_INIT_ENTRY}
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

  , em_lastUpdateAnalog{}
  , em_lastUpdateGpio{}
  , em_mission{asMission::AMI_NONE}
  , em_asms{false}
  , em_tsOn{false}
  , em_resGoSignal{false}
  , em_resStopSignal{true}
  , em_finishSignal{false}
  , em_ebsOk{false}
  , em_clampExtended{false}
  , em_resStatus{false}
  , em_resInitialized{false}
  , em_vehicleSpeed{0.0f}
  , em_prEbsAct{0.0f}
  , em_prEbsLine{0.0f}
  , em_prServiceTank{0.0f}
  , em_prServiceReg{0.0f}
  , em_steerPosAct{0.0f}
  , em_steerPosRack{0.0f}
  , em_brakeDutyRequest{0U}
  , em_torqueReqLeft{0}
  , em_torqueReqRight{0}

  , m_resourceMutex{}
{
  std::cout << "Initializing state machine... ";

  cluon::data::TimeStamp sampleTime = cluon::time::now();
  int32_t senderStamp = m_senderStampResInitialize;

  // First part of handshake with RES CAN service
  opendlv::proxy::SwitchStateRequest msgCan;
  msgCan.state(true);
  m_od4.send(msgCan, sampleTime, senderStamp);

  // Initialize the front node delay trackers
  {
    std::lock_guard<std::mutex> lock(m_resourceMutex);

    em_lastUpdateAnalog = cluon::time::now();
    em_lastUpdateGpio = cluon::time::now();
  }

  std::cout << "done." << std::endl;
}

StateMachine::~StateMachine() {}

void StateMachine::step()
{
  // -------------------------- INITIAL COPY OF DATA --------------------------
  uint64_t lastUpdateAnalog, lastUpdateGpio;
  bool asms, ebsOk, resGoSignal, resStopSignal, clampExtended, finishSignal, tsOn;
  float steerPosAct, steerPosRack, vehicleSpeed;
  float prEbsAct, prEbsLine, prServiceTank, prServiceReg;
  uint32_t brakeDutyRequest;
  int16_t torqueReqRight, torqueReqLeft;
  asMission mission;
  {
    std::lock_guard<std::mutex> lock(m_resourceMutex);

    lastUpdateAnalog = cluon::time::toMicroseconds(em_lastUpdateAnalog);
    lastUpdateGpio = cluon::time::toMicroseconds(em_lastUpdateGpio);
    asms = em_asms;
    ebsOk = em_ebsOk;
    resGoSignal = em_resGoSignal;
    resStopSignal = em_resStopSignal;
    clampExtended = em_clampExtended;
    finishSignal = em_finishSignal;
    tsOn = em_tsOn;
    steerPosAct = em_steerPosAct;
    steerPosRack = em_steerPosRack;
    vehicleSpeed = em_vehicleSpeed;
    prEbsAct = em_prEbsAct;
    prEbsLine = em_prEbsLine;
    prServiceTank = em_prServiceTank;
    prServiceReg = em_prServiceReg;
    brakeDutyRequest = em_brakeDutyRequest;
    torqueReqRight = em_torqueReqRight;
    torqueReqLeft = em_torqueReqLeft;
    mission = em_mission;
  }

  // ----------------------------- INITIAL CHECKS -----------------------------

  // Check if we're continuously receiving data from AS node
  bool frontNodeOk = (lastUpdateAnalog != 0) && (lastUpdateGpio != 0);
  // Check if we're receiving data from AS node
  if(!frontNodeOk){
    std::cout << "Front node status: " << (frontNodeOk ? "On\n" : "Off\n") << std::endl;
    return;
  }
  m_modulesRunning = true;

  uint64_t threadTime = cluon::time::toMicroseconds(cluon::time::now());
  uint64_t analogDelay = threadTime - lastUpdateAnalog;
  uint64_t gpioDelay = threadTime - lastUpdateGpio;
  if( ( (analogDelay > 500000) || (gpioDelay > 1000000) ) && m_modulesRunning){
      m_modulesRunning = false;
      std::cout << "[ASS-ERROR] Module has crashed. Last gpio update:" << gpioDelay << "\t Last analog update: " << analogDelay << std::endl;
  }

  if (ebsOk) { // TODO: Remove this when tsOn signal has been added to AS node
    em_tsOn = true;
  } else {
    em_tsOn = false;
  }

  brakeUpdate(asms, ebsOk, resStopSignal, prEbsAct,
              prEbsLine, prServiceTank,
              prServiceReg, brakeDutyRequest);
  stateUpdate(asms, finishSignal, resGoSignal, tsOn, clampExtended,
              resStopSignal, mission, torqueReqLeft, torqueReqRight,
              vehicleSpeed);
  setAssi();
  sendMessages();

  // Check steering implausibility
  bool systemReadyOrDriving = (m_asState == asState::AS_DRIVING || m_asState == asState::AS_READY);
  bool steeringDiffLarge = std::fabs(steerPosAct - steerPosRack) > 10.0f;
  if (systemReadyOrDriving && (!clampExtended || steeringDiffLarge) && resGoSignal) {
    m_steerFault = true;
    std::cout << "[ASS-ERROR] Steering Failure:\n m_clampExtended: " << clampExtended << "\nsteeringDiffLarge: " << steeringDiffLarge << std::endl;        
  } else {
    m_steerFault = false;  
  }

  if (m_verbose && m_refreshMsg) {
  uint64_t threadTimeEnd = cluon::time::toMicroseconds(cluon::time::now());
  std::cout << "[AS-state] Current AS state: " << m_asState 
            << "\nThread time: " << threadTimeEnd << " microseconds"
            << "\nState machine update took: " << (float)(threadTimeEnd - threadTime) / 1000.0f << " ms"
            << "\nebsOk: " << ebsOk
            << "\nASMS: " << asms
            << "\nMission: " << mission
            << "\nEBS state: " << m_ebsState
            << "\nBrake state: " << m_brakeState 
            << "\nCompressor: " << m_compressor 
            << "\nSteerClamp: " << clampExtended
            << "\nEBS pressure line: " << prEbsLine
            << "\nEBS pressure act: " << prEbsAct
            << "\nService pressure tank: " << prServiceTank
            << "\nService pressure reg: " << prServiceReg 
            << "\nbrakeDuty: " << m_brakeDuty
            << "\nbrakeDutyRequest: " << brakeDutyRequest
            << "\nFlash ASSI: " << m_flash2Hz
            << "\n" << std::endl;
  }
  m_refreshMsg = false;

}

// TODO: handle EBS error LED signal
void StateMachine::brakeUpdate(bool asms, bool ebsOk, bool resStopSignal,
                               float prEbsAct, float prEbsLine,
                               float prServiceTank, float prServiceReg,
                               uint32_t brakeDutyRequest)
{
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();

  // ------------------------ EBS INITIALIZATION CHECK ------------------------
  if (m_ebsInitState != ebsInitState::EBS_INIT_INITIALIZED){
    switch (m_ebsInitState)
    {
    case ebsInitState::EBS_INIT_ENTRY:
      if (asms)
      {
        m_ebsInitState = ebsInitState::EBS_INIT_CHARGING;
        m_lastEbsInitTransition = timeMillis;
      }
      break;

    case ebsInitState::EBS_INIT_CHARGING:
      m_brakeDuty = 20000U;
      if (prEbsAct >= 5 && prEbsLine >= 5 && prServiceTank >= 6)
      {
        m_compressor = false;
        m_ebsInitState = ebsInitState::EBS_INIT_COMPRESSOR;
        m_lastEbsInitTransition = timeMillis;
      }
      else if (((m_lastEbsInitTransition + 5000) <= timeMillis) && (prEbsAct <= 0.5)) // TODO: check EBS_line too
      {
        std::cout << "[EBS-Init] Failed to increase pressure above 0.5bar in 5s."
                  << " m_pressureEbsAct: " << prEbsAct
                  << " m_pressureServiceReg: " << prServiceReg << std::endl;
        m_ebsInitState = ebsInitState::EBS_INIT_FAILED;
        m_lastEbsInitTransition = timeMillis;
      }
      else if (((m_lastEbsInitTransition + 60000) <= timeMillis))
      {
        std::cout << "[EBS-Init] Failed to increase pressures 60s." << std::endl;
        m_ebsInitState = ebsInitState::EBS_INIT_FAILED;
        m_lastEbsInitTransition = timeMillis;
      }
      break;

    case ebsInitState::EBS_INIT_COMPRESSOR:
      if (!m_compressor)
      {
        std::cout << "[EBS-Init] Initialisation done" << std::endl;
        m_ebsInitState = ebsInitState::EBS_INIT_INITIALIZED;
        m_lastEbsInitTransition = timeMillis;
      }
      break;

    default:
      break;
    }

    if (m_verbose && m_refreshMsg)
    {
      std::cout << "[EBS-Init] Current EBS Init state: " << m_ebsInitState << std::endl;
    }
  }

  // ------------------------------ BRAKE UPDATE ------------------------------

  m_serviceBrakePressureOk = prServiceTank >= 6.0f;
  m_ebsPressureOk = prEbsLine >= 6.0f;
  m_brakesReleased = (prEbsAct < 0.1f && prServiceReg < 0.1f);
  bool systemReadyOrDriving = (m_asState == asState::AS_DRIVING || m_asState == asState::AS_READY);
  bool systemNotOff = (m_asState != asState::AS_OFF && m_asState != asState::AS_FINISHED);
  bool serviceBrakeLow = (prServiceTank <= 4.0f) && systemReadyOrDriving;
     
  bool sensorDisconnected = (prEbsAct < -0.08f || prEbsLine < -0.06f || prServiceTank < -0.07f);
  bool ebsPressureFail = (!m_ebsPressureOk && systemNotOff);

  // Check if the compressor should be on/off
  // TODO: Tune pressure parameters
  if ((prEbsLine > 6.0f && prServiceTank > 8.0f) || prServiceTank > 9.0f ||
        prServiceTank < -0.05f || m_asState == asState::AS_EMERGENCY) {
    m_compressor = false;
  } else if (asms && (prEbsLine < 5.0f || prServiceTank < 6.0f)) {
    m_compressor = true;
  }

  if (m_asState == AS_OFF && m_ebsInitState == EBS_INIT_INITIALIZED && 
        m_ebsPressureOk && m_serviceBrakePressureOk && ebsOk && !m_compressor) {
    m_ebsState = ebsState::EBS_ARMED;
  }

  if (m_brakeState == BRAKE_AVAILABLE) {
    m_brakeDuty = ((m_lastStateTransition+500U) >= timeMillis) ? 20000U : brakeDutyRequest;
  } else if (m_brakeState == BRAKE_ENGAGED && prEbsAct > 0.1f) {
    m_brakeDuty = 20000U;
  } else {
    m_brakeDuty = 0U;
  }

  // Check the EBS, service and steering status if ASMS is on
  if (asms) {
    if (sensorDisconnected || ebsPressureFail || serviceBrakeLow || m_ebsInitState == EBS_INIT_FAILED){
      m_ebsFault = true;
      std::cout << "[ASS-ERROR] EBS Failure: sensorDisconnected: " << sensorDisconnected 
              << "\n ebsPressureFail: " << ebsPressureFail 
               << "\n serviceBrakeLow: " << serviceBrakeLow 
               << "\n m_ebsInitFail: " << m_ebsInitState << std::endl;        
    } else {
      m_ebsFault = false;
    }

    if((!ebsOk || !m_modulesRunning || m_ebsFault || m_steerFault || !resStopSignal) && 
                      (m_asState == asState::AS_READY || m_asState == asState::AS_DRIVING || m_asState == asState::AS_FINISHED)){
      m_ebsActivatedTime = timeMillis;
      m_ebsState = ebsState::EBS_ACTIVATED;
      std::cout << "[ASS-Machine] EBS ACTIVATED, CURRENT STATE: " << m_asState 
              << " Values: m_ebsOk: " << ebsOk 
              << " m_modulesRunning: " << m_modulesRunning 
              << " m_ebsFault: " << m_ebsFault 
              << " m_steerFault: " << m_steerFault 
              << " m_shutdown: " << m_shutdown
              << " m_finished: " << m_finished
              << " m_prevState: " << m_prevState 
              << " m_resStopSignal: " << resStopSignal << std::endl;
    }
  }
}

void StateMachine::stateUpdate(bool asms, bool finishSignal,
                               bool resGoSignal, bool tsOn, bool clampExtended,
                               bool resStopSignal, asMission mission,
                               int16_t torqueReqLeft, int16_t torqueReqRight,
                               float vehicleSpeed)
{
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();

  // Reset values to make sure nothing gets sent out if in wrong state
  m_ebsSpeaker = false;
  m_finished = false;
  m_shutdown = false;
  m_torqueReqLeftCan = 0U;
  m_torqueReqRightCan = 0U;
  m_rtd = false;

  switch(m_asState) {
    case asState::AS_OFF:
      m_serviceBrake = false;
      m_brakeState = serviceBrakeState::BRAKE_UNAVAILABLE;

      if (mission != AMI_NONE && mission != AMI_MANUAL && m_ebsState == EBS_ARMED && asms && tsOn) {
        m_prevState = asState::AS_OFF;
        m_asState = asState::AS_READY;
        m_lastStateTransition = timeMillis;
      } else if (mission == AMI_MANUAL && m_ebsState == EBS_UNAVAILABLE && !asms && tsOn && !clampExtended) {
        m_prevState = asState::AS_OFF;
        m_asState = asState::AS_MANUAL;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_READY:
      m_brakeState = serviceBrakeState::BRAKE_ENGAGED;
      m_serviceBrake = true;

      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_READY;
        m_asState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (!asms && m_brakesReleased) {
        m_prevState = asState::AS_READY;
        m_asState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_ebsInitState = EBS_INIT_ENTRY;
        m_ebsState = EBS_UNAVAILABLE;
      } else if ((timeMillis - m_lastStateTransition > 5000U) && resGoSignal && clampExtended) {
        m_prevState = asState::AS_READY;
        m_asState = asState::AS_DRIVING;
        m_lastStateTransition = timeMillis;
      } else {
        // If we have pressed and released the go signal too early
        // we reset its state here
        std::lock_guard<std::mutex> lock(m_resourceMutex);
        em_resGoSignal = false;
      }
      break;

    case asState::AS_DRIVING:
      m_brakeState = serviceBrakeState::BRAKE_AVAILABLE;
      m_torqueReqLeftCan = torqueReqLeft;
      m_torqueReqRightCan = torqueReqRight;
      m_rtd = true;
      
      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_DRIVING;
        m_asState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (finishSignal && vehicleSpeed < 0.01f) {
        m_prevState = asState::AS_DRIVING;
        m_asState = asState::AS_FINISHED;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_FINISHED:
      m_brakeState = serviceBrakeState::BRAKE_ENGAGED;
      m_ebsState = ebsState::EBS_ACTIVATED;
      m_finished = true;
      
      if (!resStopSignal) {
        m_prevState = asState::AS_FINISHED;
        m_asState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (!asms && m_brakesReleased) {
        m_prevState = asState::AS_FINISHED;
        m_asState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_ebsInitState = ebsInitState::EBS_INIT_ENTRY;
        m_ebsState = ebsState::EBS_UNAVAILABLE;
      }
      break;

    case asState::AS_EMERGENCY:
      m_ebsSpeaker = ((m_ebsActivatedTime+9000) >= timeMillis);
      m_finished = false;
      m_shutdown = true;

      if (!m_ebsSpeaker && !asms && m_brakesReleased) {
        m_prevState = asState::AS_EMERGENCY;
        m_asState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_ebsInitState = ebsInitState::EBS_INIT_ENTRY;
        m_ebsState = ebsState::EBS_UNAVAILABLE;
      } 
      break;

    case asState::AS_MANUAL:
      if (!tsOn) {
        m_prevState = asState::AS_MANUAL;
        m_asState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      }
      break;

    default:
      break;
  }
}

// TODO: Also check hydraulic brake pressure during this startup procedure


void StateMachine::setAssi()
{
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();

  if (m_nextFlashTime <= timeMillis){
    m_flash2Hz  = !m_flash2Hz;
    m_refreshMsg = true;
    m_nextFlashTime = timeMillis + 500U; // TODO: Make better solution
  }


  switch(m_asState){
    case asState::AS_OFF:
      m_blueDuty  = 0U;
      m_greenDuty = 0U;
      m_redDuty   = 0U;
      break;
    case asState::AS_READY:
      m_blueDuty  = 0U;
      m_greenDuty = 1000000000U;
      m_redDuty   = 1000000000U;
      break;
    case asState::AS_DRIVING:
      m_blueDuty  = 0U;
      m_greenDuty = 1000000000U*m_flash2Hz;
      m_redDuty   = 1000000000U*m_flash2Hz;
      break;
    case asState::AS_FINISHED:
      m_blueDuty  = 1000000000U;
      m_greenDuty = 0U;
      m_redDuty   = 0U;
      break;
    case asState::AS_EMERGENCY:
      m_blueDuty  = 1000000000U*m_flash2Hz;
      m_greenDuty = 0U;
      m_redDuty   = 0U;
      break;
    case asState::AS_MANUAL:
      m_blueDuty  = 0U;
      m_greenDuty = 0U;
      m_redDuty   = 0U;
      break;
    default:
      break;
  }
}

void StateMachine::sendMessages()
{
  // Sending std messages
  cluon::data::TimeStamp sampleTime = cluon::time::now();
  int16_t senderStamp;

  // GPIO Msg
  opendlv::proxy::SwitchStateRequest msgGpio;

  //Heartbeat Msg
  m_heartbeat = !m_heartbeat;
  senderStamp = m_gpioStampHeartbeat + m_senderStampOffsetGpio;
  msgGpio.state(m_heartbeat);
  m_od4.send(msgGpio, sampleTime, senderStamp);

  // m_ebsSpeaker Msg
  if (m_ebsSpeaker != m_ebsSpeakerOld || m_refreshMsg) {
    senderStamp = m_gpioStampEbsSpeaker + m_senderStampOffsetGpio;
    msgGpio.state(m_ebsSpeaker);
    m_od4.send(msgGpio, sampleTime, senderStamp);
    m_ebsSpeakerOld = m_ebsSpeaker;
  }

  // m_compressor Msg
  if (m_compressor != m_compressorOld || m_refreshMsg) {
    senderStamp = m_gpioStampCompressor + m_senderStampOffsetGpio;
    msgGpio.state(m_compressor);
    m_od4.send(msgGpio, sampleTime, senderStamp);
    m_compressorOld = m_compressor;
  }

  // m_finished Msg
  if (m_finished != m_finishedOld || m_refreshMsg) {
    senderStamp = m_gpioStampFinished + m_senderStampOffsetGpio;
    msgGpio.state(m_finished);
    m_od4.send(msgGpio, sampleTime, senderStamp);
    m_finishedOld = m_finished;
  }

  // m_shutdown Msg
  if (m_shutdown != m_shutdownOld || m_refreshMsg) {
    senderStamp = m_gpioStampShutdown + m_senderStampOffsetGpio;
    msgGpio.state(m_shutdown);
    m_od4.send(msgGpio, sampleTime, senderStamp);
    m_shutdownOld = m_shutdown;
  }
  
  // m_serviceBrake
  if (m_serviceBrake != m_serviceBrakeOld || m_refreshMsg) {
    senderStamp = m_gpioStampServiceBrake + m_senderStampOffsetGpio;
    msgGpio.state(m_serviceBrake);
    m_od4.send(msgGpio, sampleTime, senderStamp);
    m_serviceBrakeOld = m_serviceBrake;
  }

  // Send pwm Requests
  opendlv::proxy::PulseWidthModulationRequest msgPwm;

  if (m_redDuty != m_redDutyOld || m_refreshMsg) {
    senderStamp = m_pwmStampAssiRed + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_redDuty);
    m_od4.send(msgPwm, sampleTime, senderStamp);
    m_redDutyOld = m_redDuty;
  }
  if (m_greenDuty != m_greenDutyOld || m_refreshMsg) {
    senderStamp = m_pwmStampAssiGreen + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_greenDuty);
    m_od4.send(msgPwm, sampleTime, senderStamp);
    m_greenDutyOld = m_greenDuty;
  }
  if (m_blueDuty != m_blueDutyOld || m_refreshMsg) {
    senderStamp = m_pwmStampAssiBlue + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_blueDuty);
    m_od4.send(msgPwm, sampleTime, senderStamp);
    m_blueDutyOld = m_blueDuty;
  }

  if (m_brakeDuty != m_brakeDutyOld || m_refreshMsg) {
    senderStamp = m_pwmStampBrake + m_senderStampOffsetPwm;
    msgPwm.dutyCycleNs(m_brakeDuty);
    m_od4.send(msgPwm, sampleTime, senderStamp);
    m_brakeDutyOld = m_brakeDuty;
  }


  //Send Current state of state machine
  opendlv::proxy::SwitchStateReading msgGpioRead;

  senderStamp = m_senderStampAsState;
  msgGpioRead.state((uint16_t)m_asState);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);

  senderStamp = m_senderStampRTD;
  msgGpioRead.state((uint16_t)m_rtd);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);

  senderStamp = m_senderStampEBSFault;
  msgGpioRead.state((uint16_t)m_ebsFault);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);

  senderStamp = m_senderStampEbsState;
  msgGpioRead.state((uint16_t)m_ebsState);
  m_od4.send(msgGpioRead, sampleTime, senderStamp);


  // Torque requests to CAN proxy
  opendlv::proxy::TorqueRequest msgTorqueReq;

  opendlv::cfsdProxy::TorqueRequestDual msgTorqueReqDual;
  senderStamp = m_senderStampTorqueOut;
  msgTorqueReqDual.torqueLeft(m_torqueReqLeftCan);
  msgTorqueReqDual.torqueRight(m_torqueReqRightCan);
  m_od4.send(msgTorqueReqDual, sampleTime, senderStamp);
  
}



asState StateMachine::getAsState() {return m_asState;}

void StateMachine::setLastUpdateAnalog(cluon::data::TimeStamp lastUpdateAnalog)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_lastUpdateAnalog = lastUpdateAnalog;
}

void StateMachine::setLastUpdateGpio(cluon::data::TimeStamp lastUpdateGpio)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_lastUpdateGpio = lastUpdateGpio;
}

void StateMachine::setMission(uint16_t mission)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_mission = static_cast<asMission>(mission);
}

void StateMachine::setAsms(bool asms)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_asms = asms;
}

void StateMachine::setTsOn(bool tsOn)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_tsOn = tsOn;
}

void StateMachine::setGoSignal(bool goSignal)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_resGoSignal = goSignal;
}

void StateMachine::setStopSignal(bool stopSignal)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_resStopSignal = stopSignal;
}

void StateMachine::setFinishSignal(bool finishSignal)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_finishSignal = finishSignal;
}

void StateMachine::setEbsOk(bool ebsOk)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_ebsOk = ebsOk;
}

void StateMachine::setClampExtended(bool clampExtended)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_clampExtended = clampExtended;
}

void StateMachine::setVehicleSpeed(float vehicleSpeed)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_vehicleSpeed = vehicleSpeed;
}

void StateMachine::setPressureEbsAct(float pressure)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_prEbsAct = pressure;
}

void StateMachine::setPressureEbsLine(float pressure)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_prEbsLine = pressure;
}

void StateMachine::setPressureServiceTank(float pressure)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_prServiceTank = pressure;
}

void StateMachine::setPressureServiceReg(float pressure)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_prServiceReg = pressure;
}

void StateMachine::setSteerPosition(float pos)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_steerPosAct = pos;
}

void StateMachine::setSteerPositionRack(float pos)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_steerPosRack = pos;
}

void StateMachine::setBrakeDutyCycle(uint32_t duty)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_brakeDutyRequest = duty;
}

void StateMachine::setTorqueRequest(int16_t torqueRight, int16_t torqueLeft)
{
  std::lock_guard<std::mutex> lock(m_resourceMutex);
  em_torqueReqRight = torqueRight;
  em_torqueReqLeft = torqueLeft;
}

void StateMachine::setResStatus(bool status)
{
  std::lock_guard<std::mutex> lock1(m_resourceMutex);
  em_resStatus = status;

  if (!em_resInitialized) {
    em_resInitialized = status;
  }
}