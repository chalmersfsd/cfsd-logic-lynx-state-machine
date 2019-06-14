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

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "opendlv-standard-message-set.hpp"
#include "cfsd-extended-message-set.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <mutex>

enum asState {
    AS_OFF,
    AS_READY, 
    AS_DRIVING, 
    AS_FINISHED,
    AS_EMERGENCY,
    AS_MANUAL
 };

 enum asMission {
    AMI_NONE,
    AMI_ACCELERATION, 
    AMI_SKIDPAD, 
    AMI_TRACKDRIVE, 
    AMI_AUTOCROSS,
    AMI_BRAKETEST,
    AMI_INSPECTION,
    AMI_MANUAL,
    AMI_TEST
};

enum ebsState {
    EBS_UNAVAILABLE,
    EBS_ARMED,
    EBS_ACTIVATED
};

enum serviceBrakeState {
    BRAKE_UNAVAILABLE,
    BRAKE_ENGAGED,
    BRAKE_AVAILABLE
};

enum ebsInitState {
    EBS_INIT_ENTRY,
    EBS_INIT_CHARGING,
    EBS_INIT_COMPRESSOR,
    EBS_INIT_INITIALIZED,
    EBS_INIT_FAILED
};

class StateMachine {
  private:
    StateMachine(const StateMachine &) = delete;
    StateMachine(StateMachine &&)      = delete;
    StateMachine &operator=(const StateMachine &) = delete;
    StateMachine &operator=(StateMachine &&) = delete;

  public:
    StateMachine(cluon::OD4Session &od4, bool verbose);
    ~StateMachine();

  private:
    void setUp();
    void tearDown();
    void brakeUpdate(bool asms, bool ebsOk, bool resStopSignal,
                     float pressureEbsAct, float pressureEbsLine,
                     float pressureServiceTank, float pressureServiceReg,
                     uint32_t brakeDutyRequest);
    void stateUpdate(bool asms, bool finishSignal,
                     bool resGoSignal, bool tsOn, bool clampExtended,
                     bool resStopSignal, asMission mission,
                     int16_t torqueReqLeft, int16_t torqueReqRight,
                     float vehicleSpeed);
    void setAssi();
    void sendMessages();

  public:
    void step();
    void heartbeat();
    asState getAsState();
    void setLastUpdateAnalog(cluon::data::TimeStamp time);
    void setLastUpdateGpio(cluon::data::TimeStamp time);
    void setMission(uint16_t mission);
    void setAsms(bool asms);
    void setTsOn(bool tsOn);
    void setGoSignal(bool goSignal);
    void setStopSignal(bool stopSignal);
    void setFinishSignal(bool finishSignal);
    void setEbsOk(bool ebsOk);
    void setClampExtended(bool clampExtended);
    void setVehicleSpeed(float vehicleSpeed);
    void setPressureEbsAct(float pressure);
    void setPressureEbsLine(float pressure);
    void setPressureServiceTank(float pressure);
    void setPressureServiceReg(float pressure);
    void setSteerPosition(float pos);
    void setSteerPositionRack(float pos);
    void setBrakeDutyCycle(uint32_t duty);
    void setTorqueRequest(int16_t torqueRight, int16_t torqueLeft);
    void setResStatus(bool status);

  private:
    cluon::OD4Session &m_od4;
    asState m_prevState;
    asState m_asState;
    ebsState m_ebsState;
    serviceBrakeState m_brakeState;
    ebsInitState m_ebsInitState;
    uint64_t m_lastStateTransition;
    uint64_t m_lastEbsInitTransition;
    uint64_t m_nextFlashTime;
    uint64_t m_ebsActivatedTime;
    uint32_t m_brakeDuty;
    uint32_t m_brakeDutyOld;
    uint32_t m_blueDuty;
    uint32_t m_blueDutyOld;
    uint32_t m_greenDuty;
    uint32_t m_greenDutyOld;
    uint32_t m_redDuty;
    uint32_t m_redDutyOld;
    int16_t m_torqueReqLeftCan;
    int16_t m_torqueReqRightCan;
    bool m_compressor;
    bool m_compressorOld;
    bool m_modulesRunning;
    bool m_serviceBrake;
    bool m_serviceBrakeOld;
    bool m_serviceBrakePressureOk;
    bool m_ebsPressureOk;
    bool m_steerFault;
    bool m_ebsFault;
    bool m_flash2Hz;
    bool m_rtd;
    bool m_finished;
    bool m_finishedOld;
    bool m_shutdown;
    bool m_shutdownOld;
    bool m_ebsSpeaker;
    bool m_ebsSpeakerOld;
    bool m_heartbeat;
    bool m_refreshMsg;
    bool m_brakesReleased;
    bool m_verbose;

    // Received from other microservices
    cluon::data::TimeStamp em_lastUpdateAnalog;
    cluon::data::TimeStamp em_lastUpdateGpio;
    asMission em_mission;
    bool em_asms;
    bool em_tsOn; // tsOn shares the same signal as ebsOk for now. TODO: Change to other signal when available
    bool em_resGoSignal;
    bool em_resStopSignal;
    bool em_finishSignal;
    bool em_ebsOk;
    bool em_clampExtended;
    bool em_resStatus;
    bool em_resInitialized;
    float em_vehicleSpeed;
    float em_prEbsAct;
    float em_prEbsLine;
    float em_prServiceTank;
    float em_prServiceReg;
    float em_steerPosAct;
    float em_steerPosRack;
    uint32_t em_brakeDutyReq;
    int16_t em_torqueReqLeft;
    int16_t em_torqueReqRight;

    // Resource mutex for all data set by data triggers
    std::mutex m_resourceMutex;

    // Run heartbeat in separate thread
    std::thread _heartbeatThread;

  public:
    // Senderstamps offset
    const uint16_t m_senderStampOffsetGpio = 1000;
    const uint16_t m_senderStampOffsetAnalog = 1200;
    const uint16_t m_senderStampOffsetPwm = 1300;

    // Broadcast
    const uint16_t m_senderStampResInitialize = 1099;
    const uint16_t m_senderStampTorqueOut = 1910;
    const uint16_t m_senderStampAsState = 2101;
    const uint16_t m_senderStampRTD = 2104;
    const uint16_t m_senderStampEBSFault = 2105;
    const uint16_t m_senderStampBrakeReq = 2150;
    const uint16_t m_senderStampSteeringState = 2113;
    const uint16_t m_senderStampEbsState = 2114;
    const uint16_t m_senderStampServiceValveState = 2115;

    // Torque senderstamps
    const uint16_t m_senderStampTorqueIn = 2101;

    // Input from RES proxy
    const uint16_t m_senderStampResStatus = 1801;
    const uint16_t m_senderStampResStop = 1802;
    const uint16_t m_senderStampResButtons = 1804;

    // Input from CAN proxy
    const uint16_t m_senderStampAsMission = 1906;

    // Depends on pin value in opendlv-device-stm32-lynx
    const uint16_t m_gpioStampEbsOk = 49;
    const uint16_t m_gpioStampAsms = 115;
    const uint16_t m_gpioStampClampSensor = 112;

    const uint16_t m_gpioStampHeartbeat = 27;
    const uint16_t m_gpioStampEbsSpeaker = 44;
    const uint16_t m_gpioStampCompressor = 45;
    const uint16_t m_gpioStampEbsRelief = 61;
    const uint16_t m_gpioStampFinished = 66;
    const uint16_t m_gpioStampShutdown = 67;
    const uint16_t m_gpioStampServiceBrake = 69;

    const uint16_t m_pwmStampAssiBlue = 0;
    const uint16_t m_pwmStampAssiRed = 20;
    const uint16_t m_pwmStampAssiGreen = 21;
    const uint16_t m_pwmStampBrake = 41;

    const uint16_t m_analogStampEbsLine = 1;
    const uint16_t m_analogStampServiceTank = 2;
    const uint16_t m_analogStampEbsActuator = 3;
    const uint16_t m_analogStampPressureReg = 5;
    const uint16_t m_analogStampSteerPosition = 0;
    const uint16_t m_analogStampSteerPositionRack = 6;
};
#endif

