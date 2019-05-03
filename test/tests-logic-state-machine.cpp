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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "logic-state-machine.hpp"

// #include <string>
// #include <vector>

TEST_CASE("Test AS state at startup") {
    cluon::OD4Session od4{111};
    cluon::OD4Session od4Analog{111};
    cluon::OD4Session od4Gpio{111};
    cluon::OD4Session od4Pwm{111};

    StateMachine stateMachine(od4, od4Analog, od4Gpio, od4Pwm, 0);
    stateMachine.body();
    asState state = stateMachine.getCurrentState();

    REQUIRE(state == asState::AS_OFF);
}

TEST_CASE("Test AS transitions") {
    cluon::OD4Session od4{111};
    cluon::OD4Session od4Analog{111};
    cluon::OD4Session od4Gpio{111};
    cluon::OD4Session od4Pwm{111};

    StateMachine stateMachine(od4, od4Analog, od4Gpio, od4Pwm, 0);

    SECTION("AS_OFF to AS_READY") {
        stateMachine.setResStatus(true);
        stateMachine.setPressureServiceTank(7.0f);
        stateMachine.setPressureEbsLine(7.0f);
        stateMachine.setPressureServiceReg(0.0f);
        stateMachine.setPressureEbsAct(7.0f);
        stateMachine.setAsms(true);
        stateMachine.setEbsOk(true);
        stateMachine.setLastUpdateAnalog(cluon::time::now());
        stateMachine.setLastUpdateGpio(cluon::time::now());
        stateMachine.setClampExtended(true);
        stateMachine.setSteerPosition(0.0f);
        stateMachine.setSteerPositionRack(0.0f);
        stateMachine.setMission(asMission::AMI_ACCELERATION);

        for (int i = 0; i < 10; i++) {
            stateMachine.body();
        }
        asState state = stateMachine.getCurrentState();

        REQUIRE(state == asState::AS_READY);
    }

/*
    SECTION("AS_READY to AS_DRIVING") {
        std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
        auto tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
        auto value = tp_ms.time_since_epoch();
        uint64_t timeMillis = value.count();
        uint64_t timePoint = timeMillis;

        while (timeMillis - timePoint <= 5000) {
            stateMachine.setPressureServiceTank(7.0f);
            stateMachine.setPressureEbsLine(7.0f);
            stateMachine.setPressureServiceReg(0.0f);
            stateMachine.setPressureEbsAct(7.0f);
            stateMachine.setAsms(true);
            stateMachine.setEbsOk(true);
            usleep(400000);
            
            tp = std::chrono::system_clock::now();
            tp_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(tp);
            value = tp_ms.time_since_epoch();
            timeMillis = value.count();
        }

        stateMachine.setGoSignal(true);
        stateMachine.body();
       
        asState state = stateMachine.getCurrentState();

        REQUIRE(state == asState::AS_DRIVING);
    }

    SECTION("AS_DRIVING to AS_FINISHED") {
        stateMachine.setVehicleSpeed(0.0f);
        stateMachine.setFinishSignal(true);

        stateMachine.body();
        asState state = stateMachine.getCurrentState();

        REQUIRE(state == asState::AS_FINISHED);
    }

    SECTION("AS_FINISHED to AS_OFF") {
        stateMachine.setAsms(false);
        stateMachine.setPressureEbsAct(0.0f);
        stateMachine.setPressureServiceReg(0.0f);

        stateMachine.body();
        asState state = stateMachine.getCurrentState();

        REQUIRE(state == asState::AS_OFF);
    }
    */

}