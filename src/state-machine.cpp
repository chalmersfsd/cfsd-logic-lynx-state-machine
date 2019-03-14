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

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "logic-state-machine.hpp"
#include <iostream>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("freq")) {
        std::cerr << argv[0] << "Module running state-machine for Lynx" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --freq=<State-machine frequency> "
                  << " [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=111 --cidgpio=220 --cidanalog=221 --cidpwm=222--verbose=1 --freq=30" << std::endl;
        retCode = 1;
    } else {
        // const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const float FREQ{static_cast<float>(std::stof(commandlineArguments["freq"]))};

        // Interface to a running OpenDaVINCI session.  
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        cluon::OD4Session od4Analog{static_cast<uint16_t>(std::stoi(commandlineArguments["cidAnalog"]))};
        cluon::OD4Session od4Gpio{static_cast<uint16_t>(std::stoi(commandlineArguments["cidGpio"]))};
        //cluon::OD4Session od4GPwm{static_cast<uint16_t>(std::stoi(commandlineArguments["cidPwm"]))};

        StateMachine stateMachine(od4, od4Analog, od4Gpio);

        auto onPressureReading{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
        {
            if (!stateMachine.getInitialized()){
                return;
            }
            uint16_t senderStamp = envelope.senderStamp()-stateMachine.getSenderStampOffsetAnalog();
            opendlv::proxy::PressureReading analogInput = cluon::extractMessage<opendlv::proxy::PressureReading>(std::move(envelope));

            if (senderStamp == stateMachine.getAnalogStampEbsActuator()){
                stateMachine.setPressureEbsAct(analogInput.pressure());
                if(VERBOSE)
                    std::cout << "[LOGIC-ASS-PRESSURE-EBS-ACT] Pressure reading:" << analogInput.pressure() << std::endl;
            }else if (senderStamp == stateMachine.getAnalogStampEbsLine()){
                stateMachine.setPressureEbsLine(analogInput.pressure());
                if(VERBOSE)
                    std::cout << "[LOGIC-ASS-PRESSURE-EBS-LINE] Pressure reading:" << analogInput.pressure() << std::endl;
            }else if (senderStamp == stateMachine.getAnalogStampServiceTank()){
                stateMachine.setPressureServiceTank(analogInput.pressure());
                if(VERBOSE)
                    std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-TANK] Pressure reading:" << analogInput.pressure() << std::endl;
            }else if (senderStamp == stateMachine.getAnalogStampPressureReg()){
                stateMachine.setPressureServiceReg(analogInput.pressure());
                if(VERBOSE)
                    std::cout << "[LOGIC-ASS-PRESSURE-SERVICE-REGULATOR] Pressure reading:" << analogInput.pressure() << std::endl;
            }
            stateMachine.setLastUpdateAnalog(cluon::time::now());
        }};
        od4Analog.dataTrigger(opendlv::proxy::PressureReading::ID(), onPressureReading);

        auto onSwitchStateReadingGpio{[&stateMachine](cluon::data::Envelope &&envelope)
        {
            if (!stateMachine.getInitialized()){
                return;
            }
            uint16_t senderStamp = envelope.senderStamp()-stateMachine.getSenderStampOffsetGpio();
            opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));

            if (senderStamp == stateMachine.getGpioStampEbsOk()){
                stateMachine.setEbsOk(gpioState.state());
            }else if (senderStamp == stateMachine.getGpioStampAsms()){
                stateMachine.setAsms(gpioState.state());
            }else if (senderStamp == stateMachine.getGpioStampClampSensor()){
                stateMachine.setClampExtended(gpioState.state());
            }
            stateMachine.setLastUpdateGpio(cluon::time::now());
        }};
        od4Gpio.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReadingGpio);

        auto onGroundSteeringReading{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
        {
            if (!stateMachine.getInitialized()){
                return;
            }
            opendlv::proxy::GroundSteeringReading analogInput = cluon::extractMessage<opendlv::proxy::GroundSteeringReading>(std::move(envelope));

            uint16_t senderStamp = envelope.senderStamp();
            if (senderStamp == 1200){
                stateMachine.setSteerPosition(analogInput.groundSteering());
                if (VERBOSE)
                    std::cout << "[LOGIC-STEERING-POSITION-ACT] Position reading:" << analogInput.groundSteering() << std::endl;
            }else if (senderStamp == 1206){
                stateMachine.setSteerPositionRack(analogInput.groundSteering());
                    if (VERBOSE)
                        std::cout << "[LOGIC-STEERING-POSITION-RACK] Position reading:" << analogInput.groundSteering() << std::endl;
            }
        }};
        od4Analog.dataTrigger(opendlv::proxy::GroundSteeringReading::ID(), onGroundSteeringReading);

        auto onSwitchStateReading{[&stateMachine](cluon::data::Envelope &&envelope)
        {
            if (!stateMachine.getInitialized()){
                return;
            }
            opendlv::proxy::SwitchStateReading gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));

            uint16_t senderStamp = envelope.senderStamp();
            if (senderStamp == 1403){
                stateMachine.setFinishSignal(gpioState.state());
            }else if (senderStamp == 1410){
                stateMachine.setGoSignal(((uint8_t) gpioState.state() & 0x04)); //TODO: check if 5 or 7
            }else if (senderStamp == 1406){
                stateMachine.setMission(gpioState.state());
            }
        }};
        od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReading);

        auto onPulseWidthModulationRequest{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
        {   
            if (!stateMachine.getInitialized()){
                return;
            }

            if (envelope.senderStamp() == 1341){
            auto pwmState = cluon::extractMessage<opendlv::proxy::PulseWidthModulationRequest>(std::move(envelope));
            stateMachine.setBrakeDutyCycle(pwmState.dutyCycleNs());
            }
        }};
        od4.dataTrigger(opendlv::proxy::PulseWidthModulationRequest::ID(), onPulseWidthModulationRequest);

        auto onTorqueRequest{[&stateMachine, &VERBOSE](cluon::data::Envelope &&envelope)
        {   
            if (!stateMachine.getInitialized()){
                return;
            }

            uint16_t senderStamp = envelope.senderStamp();
            if (senderStamp == 1502){
                auto const torqueReq = cluon::extractMessage<opendlv::proxy::TorqueRequest>(std::move(envelope));
                stateMachine.setTorqueReqLeft((int)round(torqueReq.torque()));
            }else if (senderStamp == 1503){
                auto const torqueReq = cluon::extractMessage<opendlv::proxy::TorqueRequest>(std::move(envelope));
                stateMachine.setTorqueReqRight((int)round(torqueReq.torque()));
            }
        }};
        od4.dataTrigger(opendlv::proxy::TorqueRequest::ID(), onTorqueRequest);

        using namespace std::literals::chrono_literals;
        auto atFrequency{[&od4, &stateMachine, &VERBOSE]() -> bool
        {
            stateMachine.body();
            if (VERBOSE) {
                std::cout << stateMachine.getCurrentState() << std::endl;            
            }
            return true;
        }};

        od4.timeTrigger(FREQ, atFrequency);
    }
    return retCode;
}

