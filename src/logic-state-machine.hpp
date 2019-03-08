/*
 * Copyright (C) 2018  Christian Berger
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

#include <chrono>
#include <cstdint>



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

class StateMachine {
  private:
    StateMachine(const StateMachine &) = delete;
    StateMachine(StateMachine &&)      = delete;
    StateMachine &operator=(const StateMachine &) = delete;
    StateMachine &operator=(StateMachine &&) = delete;

  public:
    StateMachine();
    ~StateMachine();

  private:
    void setUp();
    void tearDown();
    void ebsUpdate();
    void stateUpdate();

  public:
    void body();

  public:
    asState m_prevState;
    asState m_currentState;
    asMission m_currentMission;
    ebsState m_ebsState;
    serviceBrakeState m_brakeState;
    bool m_initialized;
    bool m_ebsArmed;
    bool m_asms;
    bool m_tsOn;
    bool m_rtd;
    bool m_finishSignal;
    bool m_ebsSound;
    float m_vehicleSpeed;
    uint64_t m_lastStateTransition;
    
};

#endif

