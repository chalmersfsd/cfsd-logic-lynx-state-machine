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
  , m_modulesCrashed{false}
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
  , m_logPath{}

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
  , em_brakeDutyReq{0U}
  , em_torqueReqLeft{0}
  , em_torqueReqRight{0}

  , m_resourceMutex{}
  , m_od4Mutex{}

  , _heartbeatThread{}
{
  std::cout << "Initializing state machine... ";

  cluon::data::TimeStamp sampleTime = cluon::time::now();
  int32_t senderStamp = m_senderStampResInitialize;

  // First part of handshake with RES CAN service
  opendlv::proxy::SwitchStateRequest msgCan;
  msgCan.state(true);
  m_od4.send(msgCan, sampleTime, senderStamp);

  // Start heartbeat in its own thread
  _heartbeatThread = std::thread(&StateMachine::heartbeat, this);
  _heartbeatThread.detach();

  // Generate log file name with current time
  m_logPath = generateLogPath();

  std::cout << "done." << std::endl;
}

StateMachine::~StateMachine() {}

void StateMachine::step()
{
  // -------------------------- INITIAL COPY OF DATA --------------------------
  uint64_t lastUpdateAnalog, lastUpdateGpio;
  bool asms, ebsOk,clampExtended, finishSignal, tsOn;
  bool resGoSignal, resStopSignal, resInitialized;
  float steerPosAct, steerPosRack, vehicleSpeed;
  float prEbsAct, prEbsLine, prServiceTank, prServiceReg;
  uint32_t brakeDutyReq;
  int16_t torqueReqRight, torqueReqLeft;
  asMission mission;
  {
    std::lock_guard<std::mutex> lock(m_resourceMutex);

    lastUpdateAnalog = cluon::time::toMicroseconds(em_lastUpdateAnalog);
    lastUpdateGpio = cluon::time::toMicroseconds(em_lastUpdateGpio);
    asms = em_asms;
    ebsOk = em_ebsOk;
    clampExtended = em_clampExtended;
    finishSignal = em_finishSignal;
    tsOn = em_tsOn;
    resGoSignal = em_resGoSignal;
    resStopSignal = em_resStopSignal;
    resInitialized = em_resInitialized;
    steerPosAct = em_steerPosAct;
    steerPosRack = em_steerPosRack;
    vehicleSpeed = em_vehicleSpeed;
    prEbsAct = em_prEbsAct;
    prEbsLine = em_prEbsLine;
    prServiceTank = em_prServiceTank;
    prServiceReg = em_prServiceReg;
    brakeDutyReq = em_brakeDutyReq;
    torqueReqRight = em_torqueReqRight;
    torqueReqLeft = em_torqueReqLeft;
    mission = em_mission;
  }

  // ----------------------------- INITIAL CHECKS -----------------------------

  // Check if we have received data from AS node
  bool frontNodeOk = (lastUpdateAnalog != 0) && (lastUpdateGpio != 0);
  if(!frontNodeOk || !resInitialized) {
    std::cout << "Front node status: "   << (frontNodeOk ? "On" : "Off") 
              << "\nRES status:        " << (resInitialized ? "On" : "Off")
              << std::endl;
  } else {
    m_modulesRunning = true;
  }

  uint64_t threadTime = cluon::time::toMicroseconds(cluon::time::now());
  uint64_t analogDelay = threadTime - lastUpdateAnalog;
  uint64_t gpioDelay = threadTime - lastUpdateGpio;
  if( ( (analogDelay > 500000U) || (gpioDelay > 1000000U) ) && m_modulesRunning &&
        !m_modulesCrashed){
      m_modulesRunning = false;
      m_modulesCrashed = true;

      std::ostringstream errorMsg;
      errorMsg << "[AS-ERROR] AS_NODE has crashed"
               << "\nLast gpio update:" << gpioDelay 
               << "\nLast analog update: " << analogDelay;
      std::cout << errorMsg.str() << std::endl;
      logError(errorMsg.str());
  }

  if (ebsOk) { // TODO: Remove this when tsOn signal has been added to AS node
    em_tsOn = true;
  } else {
    em_tsOn = false;
  }

  brakeUpdate(asms, ebsOk, resStopSignal, prEbsAct,
              prEbsLine, prServiceTank,
              prServiceReg);
  stateUpdate(asms, finishSignal, resGoSignal, tsOn, clampExtended,
              resStopSignal, mission, torqueReqLeft, torqueReqRight,
              brakeDutyReq, vehicleSpeed);
  setAssi();
  sendMessages();

  // Check steering implausibility
  bool systemReadyOrDriving = (m_asState == asState::AS_DRIVING ||
                               m_asState == asState::AS_READY);
  bool steeringDiffLarge = std::fabs(steerPosAct - steerPosRack) > 10.0f;
  if (systemReadyOrDriving && (!clampExtended || steeringDiffLarge) && resGoSignal) {
    m_steerFault = true;

    std::ostringstream errorMsg;
    errorMsg << "[AS-ERROR] Steering Failure"
             << "\nm_clampExtended:    " << clampExtended 
             << "\nsteeringDiffLarge:  " << steeringDiffLarge;
    std::cout << errorMsg.str() << std::endl;
    logError(errorMsg.str());
  } else {
    m_steerFault = false;  
  }

  if (m_verbose && m_refreshMsg) {
    uint64_t threadTimeEnd = cluon::time::toMicroseconds(cluon::time::now());
    std::cout << "[AS-STATE-UPDATE]"
            << "\nThread time:           " << threadTimeEnd << " microseconds"
            << "\nCurrent AS state:      " << m_asState
            << "\nEBS Init state:        " << m_ebsInitState
            << "\nEBS state:             " << m_ebsState
            << "\nBrake state:           " << m_brakeState 
            << "\nMission:               " << mission
            << "\nebsOk:                 " << ebsOk
            << "\nASMS:                  " << asms
            << "\nCompressor:            " << m_compressor 
            << "\nSteerClamp:            " << clampExtended
            << "\nRES go signal:         " << resGoSignal
            << "\nEBS speaker:           " << m_ebsSpeaker
            << "\nEBS pressure line:     " << prEbsLine
            << "\nEBS pressure act:      " << prEbsAct
            << "\nPressure service tank: " << prServiceTank
            << "\nPressure service reg:  " << prServiceReg
            << "\nRGB ASSI:              " << m_redDuty << "|" << m_greenDuty << "|" << m_blueDuty
            << "\nbrakeDuty:             " << m_brakeDuty
            << "\nbrakeDutyReq:          " << brakeDutyReq
            << "\nLast analog update:    " << (float)(analogDelay) / 1000.0f << " ms"
            << "\nLast GPIO update:      " << (float)(gpioDelay) / 1000.0f << " ms"
            << "\n" << std::endl;
  }
  m_refreshMsg = false;

}

// TODO: handle EBS error LED signal
void StateMachine::brakeUpdate(bool asms, bool ebsOk, bool resStopSignal,
                               float prEbsAct, float prEbsLine,
                               float prServiceTank, float prServiceReg)
{
  uint64_t timeMillis = msTimeNow();

  // ------------------------ EBS INITIALIZATION CHECK ------------------------
  // TODO: Also check hydraulic brake pressure during this startup procedure
  if (m_ebsInitState != ebsInitState::EBS_INIT_INITIALIZED) {
    switch (m_ebsInitState)
    {
    case ebsInitState::EBS_INIT_ENTRY:
      if (asms)
      {
        m_serviceBrake = true;
        m_ebsInitState = ebsInitState::EBS_INIT_CHARGING;
        m_lastEbsInitTransition = timeMillis;
      }
      break;

    case ebsInitState::EBS_INIT_CHARGING:
      m_brakeDuty = 20000U;
      if (prEbsAct >= 5.0f && prEbsLine >= 5.0f && prServiceTank >= 6.0f)
      {
        m_compressor = false;
        m_ebsInitState = ebsInitState::EBS_INIT_COMPRESSOR;
        m_lastEbsInitTransition = timeMillis;
      }
      else if (((m_lastEbsInitTransition + 5000) <= timeMillis) && (prEbsAct <= 0.5f && prEbsLine < 0.5f)) // TODO: check EBS_line too
      {
        std::ostringstream errorMsg;
        errorMsg << "[EBS-Init] Failed to increase pressure above 0.5bar in 5s."
                 << "\nm_pressureEbsAct:     " << prEbsAct
                 << "\nm_pressureServiceReg: " << prServiceReg
                 << "\nm_ebsInitState: " << m_ebsInitState;
        std::cout << errorMsg.str() << std::endl;
        logError(errorMsg.str());

        m_ebsInitState = ebsInitState::EBS_INIT_FAILED;
        m_lastEbsInitTransition = timeMillis;
      }
      else if (((m_lastEbsInitTransition + 60000) <= timeMillis))
      {
        std::ostringstream errorMsg;
        errorMsg << "[EBS-Init] Failed to increase pressures 60s."
                 << "\nm_pressureEbsAct:     " << prEbsAct
                 << "\nm_pressureServiceReg: " << prServiceReg
                 << "\nm_ebsInitState: " << m_ebsInitState;
        std::cout << errorMsg.str() << std::endl;
        logError(errorMsg.str());

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
  }

  // ------------------------------ BRAKE UPDATE ------------------------------

  m_serviceBrakePressureOk = prServiceTank >= 6.0f;
  m_ebsPressureOk = prEbsLine >= 5.0f;
  m_brakesReleased = (prEbsAct < 0.5f && prServiceReg < 0.5f);
  bool systemReadyOrDriving = (m_asState == asState::AS_DRIVING 
                           || m_asState == asState::AS_READY);
  bool systemNotOff = (m_asState != asState::AS_OFF      &&
                       m_asState != asState::AS_FINISHED &&
                       m_asState != asState::AS_MANUAL);
  bool serviceBrakeLow = (prServiceTank <= 4.0f) && systemReadyOrDriving;
     
  // TODO: Tune sensitivity
  bool sensorDisconnected = (prEbsAct < -0.1f || prEbsLine < -0.1f ||
                             prServiceTank < -0.1f);
  bool ebsPressureFail = (!m_ebsPressureOk && systemNotOff);

  // Check if the compressor should be on/off
  // TODO: Tune pressure parameters
  if ((prEbsLine > 7.0f && prServiceTank > 8.0f) || prServiceTank > 8.3f ||
        prServiceTank < -0.05f || m_asState == asState::AS_EMERGENCY || !m_modulesRunning) {
    m_compressor = false;
  } else if (asms && (prEbsLine < 6.0f || prServiceTank < 6.0f)) {
    m_compressor = true;
  }

  if (m_asState == AS_OFF && m_ebsInitState == EBS_INIT_INITIALIZED && 
        m_ebsPressureOk && m_serviceBrakePressureOk && ebsOk && !m_compressor) {
    m_ebsState = ebsState::EBS_ARMED;
  }

  // Check the EBS, service and steering status if ASMS is on
  if (asms) {
    if ((sensorDisconnected || ebsPressureFail || serviceBrakeLow ||
          m_ebsInitState == EBS_INIT_FAILED) && !m_ebsFault) {
      m_ebsFault = true;

      std::ostringstream errorMsg;
      errorMsg << "[AS-ERROR] EBS failure"
               << "\nsensorDisconnected: " << sensorDisconnected 
               << "\nebsPressureFail:    " << ebsPressureFail 
               << "\nserviceBrakeLow:    " << serviceBrakeLow 
               << "\nm_ebsInitFail:      " << m_ebsInitState;
      std::cout << errorMsg.str() << std::endl;
      logError(errorMsg.str());        
    } else if (!sensorDisconnected && !ebsPressureFail && !serviceBrakeLow &&
          m_ebsInitState != EBS_INIT_FAILED && m_ebsFault) {
      m_ebsFault = false;
    }

    if((!ebsOk || !m_modulesRunning || m_ebsFault || !resStopSignal || m_steerFault) && 
                      (m_asState == asState::AS_READY || m_asState == asState::AS_DRIVING || m_asState == asState::AS_FINISHED)){
      m_ebsActivatedTime = timeMillis;
      m_ebsState = ebsState::EBS_ACTIVATED;

      std::ostringstream errorMsg;
      errorMsg << "[AS-ERROR] EBS activated"
               << "\nCurrent AS state: " << m_asState
               << "\nEBS Init state:   " << m_ebsInitState 
               << "\nm_ebsOk:          " << ebsOk 
               << "\nm_modulesRunning  " << m_modulesRunning 
               << "\nm_ebsFault:       " << m_ebsFault 
               << "\nm_steerFault:     " << m_steerFault 
               << "\nm_shutdown:       " << m_shutdown
               << "\nm_finished:       " << m_finished
               << "\nm_prevState:      " << m_prevState 
               << "\nm_resStopSignal:  " << resStopSignal;
      std::cout << errorMsg.str() << std::endl;
      logError(errorMsg.str());
    }
  }
}

void StateMachine::stateUpdate(bool asms, bool finishSignal,
                               bool resGoSignal, bool tsOn, bool clampExtended,
                               bool resStopSignal, asMission mission,
                               int16_t torqueReqLeft, int16_t torqueReqRight,
                               uint32_t brakeDutyReq, float vehicleSpeed)
{
  uint64_t timeMillis = msTimeNow();

  // Reset values to make sure nothing gets sent out if in wrong state
  m_ebsSpeaker = false;
  m_finished = false;
  m_shutdown = false;
  m_torqueReqLeftCan = 0U;
  m_torqueReqRightCan = 0U;
  m_rtd = false;

  switch(m_asState) {
    case asState::AS_OFF: {
      m_brakeState = serviceBrakeState::BRAKE_UNAVAILABLE;

      bool asMission = (mission != asMission::AMI_NONE && 
                        mission != asMission::AMI_MANUAL);
      bool manualMission = (mission == asMission::AMI_MANUAL);
      bool ebsArmed = (m_ebsState == ebsState::EBS_ARMED);
      bool ebsOff = (m_ebsState == ebsState::EBS_UNAVAILABLE);
      // ------------------------- AS_OFF -> AS_READY -------------------------
      if (asMission && ebsArmed && asms && tsOn) {
        m_prevState = asState::AS_OFF;
        m_asState = asState::AS_READY;
        m_lastStateTransition = timeMillis;

      // ------------------------- AS_OFF -> AS_MANUAL ------------------------
      } else if (manualMission && ebsOff && !asms && tsOn && !clampExtended) {
        m_prevState = asState::AS_OFF;
        m_asState = asState::AS_MANUAL;
        m_lastStateTransition = timeMillis;
      }
    } break;

    case asState::AS_READY: {
      m_brakeState = serviceBrakeState::BRAKE_ENGAGED;
      m_serviceBrake = true;
      m_brakeDuty = asms ? 20000U : 0U;
      
      bool waitToDrive = (timeMillis - m_lastStateTransition > 5000U);
      // ---------------------- AS_READY -> AS_EMERGENCY ----------------------
      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_READY;
        m_asState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;

      // ------------------------- AS_READY -> AS_OFF -------------------------
      } else if (!asms && m_brakesReleased) {
        m_prevState = asState::AS_READY;
        m_asState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_ebsInitState = EBS_INIT_ENTRY;
        m_ebsState = EBS_UNAVAILABLE;

      // ----------------------- AS_READY -> AS_DRIVING -----------------------
      
      } else if (waitToDrive && resGoSignal && clampExtended) {
        m_prevState = asState::AS_READY;
        m_asState = asState::AS_DRIVING;
        m_lastStateTransition = timeMillis;
      }
    } break;

    case asState::AS_DRIVING: {
      m_brakeState = serviceBrakeState::BRAKE_AVAILABLE;
      m_torqueReqLeftCan = torqueReqLeft;
      m_torqueReqRightCan = torqueReqRight;
      m_rtd = true;

      m_brakeDuty = ((m_lastStateTransition+500U) >= timeMillis) ? 20000U : brakeDutyReq;
      
      // --------------------- AS_DRIVING -> AS_EMERGENCY ---------------------
      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_DRIVING;
        m_asState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;

      // ---------------------- AS_DRIVING -> AS_FINISHED ---------------------
      } else if (finishSignal && vehicleSpeed < 0.1f) {
        m_prevState = asState::AS_DRIVING;
        m_asState = asState::AS_FINISHED;
        m_lastStateTransition = timeMillis;
      }
    } break;

    case asState::AS_FINISHED: {
      m_brakeState = serviceBrakeState::BRAKE_ENGAGED;
      m_ebsState = ebsState::EBS_ACTIVATED;
      m_finished = true;
      m_shutdown = true;

      m_brakeDuty = ((m_lastStateTransition+10000U) >= timeMillis) ? 20000U : 0U;
      
      // -------------------- AS_FINISHED -> AS_EMERGENCY ---------------------
      if (!resStopSignal) {
        m_prevState = asState::AS_FINISHED;
        m_asState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;

      // ----------------------- AS_FINISHED -> AS_OFF ------------------------
      } else if (!asms && m_brakesReleased) {
        m_prevState = asState::AS_FINISHED;
        m_asState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_ebsInitState = ebsInitState::EBS_INIT_ENTRY;
        m_ebsState = ebsState::EBS_UNAVAILABLE;
      }
    } break;

    case asState::AS_EMERGENCY:{
      m_ebsSpeaker = ((m_lastStateTransition+9000U) >= timeMillis);
      m_brakeDuty = m_ebsSpeaker ? 50000U : 0U; // Release service brake after 9s
      m_finished = false;
      m_shutdown = true;

      // ----------------------- AS_EMERGENCY -> AS_OFF -----------------------
      if (!m_ebsSpeaker && !asms && m_brakesReleased) {
        m_prevState = asState::AS_EMERGENCY;
        m_asState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;

        m_ebsInitState = ebsInitState::EBS_INIT_ENTRY;
        m_ebsState = ebsState::EBS_UNAVAILABLE;
      } 
    } break;

    case asState::AS_MANUAL: {
      // ------------------------- AS_MANUAL -> AS_OFF ------------------------
      if (!tsOn) {
        m_prevState = asState::AS_MANUAL;
        m_asState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      }
    } break;

    default: {
    } break;
  }
}

void StateMachine::setAssi()
{
  uint64_t timeMillis = msTimeNow();

  if (m_nextFlashTime <= timeMillis){
    m_flash2Hz  = !m_flash2Hz;
    m_refreshMsg = true;
    m_nextFlashTime = timeMillis + 500U;
  }


  switch(m_asState){
    case asState::AS_OFF:
      m_blueDuty  = 0U;
      m_greenDuty = 0U;
      m_redDuty   = 0U;
      break;
    case asState::AS_READY:
      m_blueDuty  = 0U;
      m_greenDuty = 50000U;
      m_redDuty   = 0U;
      break;
    case asState::AS_DRIVING:
      m_blueDuty  = 0U;
      m_greenDuty = 50000U*m_flash2Hz;
      m_redDuty   = 0U;
      break;
    case asState::AS_FINISHED:
      m_blueDuty  = 50000U;
      m_greenDuty = 0U;
      m_redDuty   = 0U;
      break;
    case asState::AS_EMERGENCY:
      m_blueDuty  = 50000U*m_flash2Hz;
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

    
  cluon::data::TimeStamp sampleTime = cluon::time::now();
  int16_t senderStamp;

  // Locking od4 mutex to avoid conflict with heartbeat thread
  {
    std::lock_guard<std::mutex> lock(m_od4Mutex);

    // GPIO Msg
    opendlv::proxy::SwitchStateRequest msgGpio;

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

    senderStamp = m_senderStampRTD;
    msgGpio.state((uint16_t)m_rtd);
    m_od4.send(msgGpio, sampleTime, senderStamp);


    // Send pwm Requests
    opendlv::proxy::PulseWidthModulationRequest msgPwm;

    if (m_redDuty != m_redDutyOld) {
      senderStamp = m_pwmStampAssiRed + m_senderStampOffsetPwm;
      msgPwm.dutyCycleNs(m_redDuty);
      m_od4.send(msgPwm, sampleTime, senderStamp);
      m_redDutyOld = m_redDuty;
    }
    if (m_greenDuty != m_greenDutyOld) {
      senderStamp = m_pwmStampAssiGreen + m_senderStampOffsetPwm;
      msgPwm.dutyCycleNs(m_greenDuty);
      m_od4.send(msgPwm, sampleTime, senderStamp);
      m_greenDutyOld = m_greenDuty;
    }
    if (m_blueDuty != m_blueDutyOld) {
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
}


// ################################ HEARTBEAT #################################
void StateMachine::heartbeat()
{
  // Heartbeat is updated in separate thread at different frequency
  // from main loop
  while(m_od4.isRunning()) {
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(50ms);

    // GPIO Msg
    {
      //Locking od4 mutex to avoid conflict with other message sending
      std::lock_guard<std::mutex> lock(m_od4Mutex);
      opendlv::proxy::SwitchStateRequest msgGpio;

      //Heartbeat Msg
      m_heartbeat = !m_heartbeat;
      uint16_t senderStamp = m_gpioStampHeartbeat + m_senderStampOffsetGpio;
      msgGpio.state(m_heartbeat);
      m_od4.send(msgGpio, cluon::time::now(), senderStamp);
    }
  }
}


// ################################## UTILITY #################################
std::string StateMachine::generateLogPath()
{
  auto time = std::time(nullptr);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S"); // ISO 8601 without timezone information.
  std::string dateString = ss.str();

  std::string logPath = "/opt/stateMachine_" + dateString + ".log";
  return logPath;
}

void StateMachine::logError(std::string errorMsg)
{
  std::ofstream myfile;
  myfile.open(m_logPath, std::ios::app);
  myfile << "\n\n" << errorMsg;
  myfile.close();
}

uint64_t StateMachine::msTimeNow()
{
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  return value.count();
}


// ############################## GETTERS/SETTERS #############################
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
  em_brakeDutyReq = duty;
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