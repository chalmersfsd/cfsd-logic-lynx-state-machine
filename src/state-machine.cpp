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
#include "opendlv-standard-message-set.hpp"

#include "logic-state-machine.hpp"
#include <iostream>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("cid")
        || 0 == commandlineArguments.count("cidGpio")
        || 0 == commandlineArguments.count("cidAnalog")
        || 0 == commandlineArguments.count("cidPwm")
        || 0 == commandlineArguments.count("freq")) {
        std::cerr << argv[0] << "Module running state-machine for Lynx" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --cidgpio=<OpenDaVINCI session> "
                  << "--cidanalog=<OpenDaVINCI session> -- cidpwm=<OpenDaVINCI session> --freq=<State-machine frequency> [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --cidgpio=220 --cidanalog=221 --cidpwm=222 --freq=30 --verbose" << std::endl;
        retCode = 1;
    } else {
        // const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const bool VERBOSE_DATA{commandlineArguments.count("verbosedata") != 0};
        const float FREQ{static_cast<float>(std::stof(commandlineArguments["freq"]))};

        // Interface to a running OpenDaVINCI session.  
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        cluon::OD4Session od4Analog{static_cast<uint16_t>(std::stoi(commandlineArguments["cidAnalog"]))};
        cluon::OD4Session od4Gpio{static_cast<uint16_t>(std::stoi(commandlineArguments["cidGpio"]))};
        cluon::OD4Session od4Pwm{static_cast<uint16_t>(std::stoi(commandlineArguments["cidPwm"]))};

        StateMachine stateMachine(od4, od4Analog, od4Gpio, od4Pwm, VERBOSE);

        auto onPressureReading{[&stateMachine, &VERBOSE, VERBOSE_DATA](cluon::data::Envelope &&envelope)
        {
            uint16_t senderStamp = envelope.senderStamp()-stateMachine.m_senderStampOffsetAnalog;

            if (senderStamp == stateMachine.m_analogStampEbsActuator) {
                auto analogInput = cluon::extractMessage<opendlv::proxy::PressureReading>(std::move(envelope));
                stateMachine.setPressureEbsAct(analogInput.pressure());
                stateMachine.setLastUpdateAnalog(cluon::time::now());
                if(VERBOSE_DATA) {
                    std::cout << "[LOGIC-ASS-PRESSURE-EBS-ACT] Pressure reading:" << analogInput.pressure() << std::endl;
                }
            } else if (senderStamp == stateMachine.m_analogStampEbsLine){
                auto analogInput = cluon::extractMessage<opendlv::proxy::PressureReading>(std::move(envelope));
                stateMachine.setPressureEbsLine(analogInput.pressure());
                stateMachine.setLastUpdateAnalog(cluon::time::now());
                if(VERBOSE_DATA) {
                    std::cout << "[LOGIC-ASS-PRESSURE-EBS-LINE] Pressure reading:" << analogInput.pressure() << std::endl;
                }
            } else if (senderStamp == stateMachine.m_analogStampServiceTank) {
                auto analogInput = cluon::extractMessage<opendlv::proxy::PressureReading>(std::move(envelope));
                stateMachine.setPressureServiceTank(analogInput.pressure());
                stateMachine.setLastUpdateAnalog(cluon::time::now());
                if(VERBOSE_DATA) {
                    std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-TANK] Pressure reading:" << analogInput.pressure() << std::endl;
                }
            } else if (senderStamp == stateMachine.m_analogStampPressureReg) {
                auto analogInput = cluon::extractMessage<opendlv::proxy::PressureReading>(std::move(envelope));
                stateMachine.setPressureServiceReg(analogInput.pressure());
                stateMachine.setLastUpdateAnalog(cluon::time::now());
                if(VERBOSE_DATA) {
                    std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-REGULATOR] Pressure reading:" << analogInput.pressure() << std::endl;
                }
            }
        }};
        od4Analog.dataTrigger(opendlv::proxy::PressureReading::ID(), onPressureReading);

        auto onSwitchStateReadingGpio{[&stateMachine](cluon::data::Envelope &&envelope)
        {
            uint16_t senderStamp = envelope.senderStamp()-stateMachine.m_senderStampOffsetGpio;
            if (senderStamp == stateMachine.m_gpioStampEbsOk) {
                auto gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                stateMachine.setEbsOk(gpioState.state());
                stateMachine.setLastUpdateGpio(cluon::time::now());
            } else if (senderStamp == stateMachine.m_gpioStampAsms) {
                auto gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                stateMachine.setAsms(gpioState.state());
                stateMachine.setLastUpdateGpio(cluon::time::now());
            } else if (senderStamp == stateMachine.m_gpioStampClampSensor) {
                auto gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                stateMachine.setClampExtended(gpioState.state());
                stateMachine.setLastUpdateGpio(cluon::time::now());
            }
        }};
        od4Gpio.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReadingGpio);

        auto onGroundSteeringReading{[&stateMachine, &VERBOSE, &VERBOSE_DATA](cluon::data::Envelope &&envelope)
        {
            uint16_t senderStamp = envelope.senderStamp() - stateMachine.m_senderStampOffsetAnalog;
            if (senderStamp == stateMachine.m_analogStampSteerPosition){
                auto analogInput = cluon::extractMessage<opendlv::proxy::GroundSteeringReading>(std::move(envelope));
                stateMachine.setSteerPosition(analogInput.groundSteering());
                if (VERBOSE_DATA)
                    std::cout << "[LOGIC-STEERING-POSITION-ACT] Position reading:" << analogInput.groundSteering() << std::endl;
            }else if (senderStamp == stateMachine.m_analogStampSteerPositionRack){
                auto analogInput = cluon::extractMessage<opendlv::proxy::GroundSteeringReading>(std::move(envelope));
                stateMachine.setSteerPositionRack(analogInput.groundSteering());
                    if (VERBOSE_DATA)
                        std::cout << "[LOGIC-STEERING-POSITION-RACK] Position reading:" << analogInput.groundSteering() << std::endl;
            }
        }};
        od4Analog.dataTrigger(opendlv::proxy::GroundSteeringReading::ID(), onGroundSteeringReading);

        auto onSwitchStateReading{[&stateMachine, &VERBOSE_DATA](cluon::data::Envelope &&envelope)
        {
            uint16_t senderStamp = envelope.senderStamp();
            if (senderStamp == 1403){
                auto gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                stateMachine.setFinishSignal(gpioState.state());
            }else if (senderStamp == stateMachine.m_senderStampResStatus){
                auto gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                if (VERBOSE_DATA) {
                    std::cout << "[LOGIC-ASS-RES-STATUS] State reading: " << gpioState.state() << std::endl;
                }
                stateMachine.setResStatus(gpioState.state());
            }else if (senderStamp == stateMachine.m_senderStampResStop){
                auto gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                stateMachine.setStopSignal(gpioState.state());
            }else if (senderStamp == stateMachine.m_senderStampResButtons){
                auto gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                stateMachine.setGoSignal(((uint8_t) gpioState.state() & 0x04)); //TODO: check if 5 or 7
            }else if (senderStamp == stateMachine.m_senderStampAsMission){
                auto gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));
                stateMachine.setMission(gpioState.state());
            }
        }};
        od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReading);

        auto onPulseWidthModulationRequest{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
        {
            if (envelope.senderStamp() == 1350){ // TODO: change senderstamp (from 1341)
                auto pwmState = cluon::extractMessage<opendlv::proxy::PulseWidthModulationRequest>(std::move(envelope));
                stateMachine.setBrakeDutyCycle(pwmState.dutyCycleNs());
            }
        }};
        od4.dataTrigger(opendlv::proxy::PulseWidthModulationRequest::ID(), onPulseWidthModulationRequest);

        auto onTorqueRequest{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
        {
            uint16_t senderStamp = envelope.senderStamp();
            if (senderStamp == stateMachine.m_senderStampTorqueIn){
                auto const torqueReq = cluon::extractMessage<opendlv::cfsdProxy::TorqueRequestDual>(std::move(envelope));
                int16_t torqueRight = (int16_t)round(torqueReq.torqueRight());
                int16_t torqueLeft = (int16_t)round(torqueReq.torqueLeft());
                stateMachine.setTorqueRequest(torqueRight, torqueLeft);
            }
        }};
        od4.dataTrigger(opendlv::cfsdProxy::TorqueRequestDual::ID(), onTorqueRequest);

        auto atFrequency{[&od4, &stateMachine, &VERBOSE]() -> bool
        {
            stateMachine.step();
            
            return true;
        }};

        od4.timeTrigger(FREQ, atFrequency);
    }
    return retCode;
}

