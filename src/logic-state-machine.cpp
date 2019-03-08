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

StateMachine::StateMachine()
  : m_prevState{}
  , m_currentState{}
  , m_currentMission{}
  , m_ebsState{}
  , m_brakeState{}
  , m_initialized{false}
  , m_ebsArmed{false}
  , m_asms{false}
  , m_tsOn{false}
  , m_rtd{false}
  , m_finishSignal{false}
  , m_ebsSound{false}
  , m_vehicleSpeed{0.0f}
  , m_lastStateTransition{}
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
  m_currentState = asState::AS_OFF;
  m_currentMission = asMission::AMI_NONE;
  m_ebsState = ebsState::EBS_UNAVAILABLE;
  m_brakeState = serviceBrakeState::BRAKE_UNAVAILABLE;
}

void StateMachine::tearDown()
{
}

void StateMachine::body()
{
  stateUpdate();
}

void StateMachine::stateUpdate()
{
  
  std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
  auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
  auto value = tp_ms.time_since_epoch();
  uint64_t timeMillis = value.count();
  
  ebsUpdate();

  switch(m_currentState) {
    case asState::AS_OFF:
      if (m_currentMission && m_ebsState == EBS_ARMED && m_asms && m_tsOn) {
        m_prevState = asState::AS_OFF;
        m_currentState = asState::AS_READY;
        m_lastStateTransition = timeMillis;
      } else if (m_currentMission == AMI_MANUAL && m_ebsState == EBS_UNAVAILABLE && !m_asms && !m_tsOn) {
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
      } else if (!m_asms && m_brakeState == BRAKE_UNAVAILABLE) { //TODO: Check that brake pressure is zero instead
        m_prevState = asState::AS_READY;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      } else if ((timeMillis - m_lastStateTransition > 5000) && m_rtd) {
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
      } else if (m_finishSignal && m_vehicleSpeed < 0.05f) {
        m_prevState = asState::AS_DRIVING;
        m_currentState = asState::AS_FINISHED;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_FINISHED:
      if (m_ebsState == EBS_ACTIVATED) {
        m_prevState = asState::AS_FINISHED;
        m_currentState = asState::AS_EMERGENCY;
        m_lastStateTransition = timeMillis;
      } else if (!m_asms && m_brakeState == BRAKE_UNAVAILABLE) { //TODO: Check that brake pressure is zero instead
        m_prevState = asState::AS_FINISHED;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      }
      break;

    case asState::AS_EMERGENCY:
      if (!m_ebsSound && !m_asms && m_brakeState == BRAKE_UNAVAILABLE) { //TODO: Check that brake pressure is zero instead
        m_prevState = asState::AS_EMERGENCY;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      } 
      break;

    case asState::AS_MANUAL:
      if (!m_tsOn) {
        m_prevState = asState::AS_MANUAL;
        m_currentState = asState::AS_OFF;
        m_lastStateTransition = timeMillis;
      }
      break;

    default:
      std::cerr << "AS state value not defined: " << m_currentState << std::endl;
      break;
  }

}

void StateMachine::ebsUpdate()
{
  // TODO: Write update for m_ebsState here
  //        Call before stateUpdate()
}
