#include "main.h"
#include "devices.h"
#include "lemlib/chassis/odom.hpp"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/misc/lv_area.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "screen.h"
#include "autons.h"
#include <array>
#include <atomic>
#include <cstdio>
#include <string>
#include <vector>

namespace {
struct AutonOption {
    const char* name;
    void (*run)();
};

// // Change this table to change auton count and mapping for the potentiometer selector.
// const std::array<AutonOption, 6> AUTONS = {{
//     {"Left", left},
//     {"Right", right},
//     {"Skills", skills96},
//     {"Five Inch", fiveInch},
// 	{"Test", test},
// 	{"Solo AWP", soloAWP}
// }};

// std::vector<std::string> autonNamesFromTable() {
//     std::vector<std::string> names;
//     names.reserve(AUTONS.size());
//     for (const auto& auton : AUTONS) names.emplace_back(auton.name);
//     return names;
// }

std::atomic_bool mclPaused{false};

void setMCLPaused(bool pause) {
    if (pause == mclPaused.load()) return;
    lemlib::toggleMCL();
    mclPaused.store(pause);
}
} // namespace


// int turn_divider = 1;
// int right_voltage = 0;
// int left_voltage = 0;
// int top_intake_voltage = 0;
// bool just_lowered_hood = false;




/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	horizontal_rotation.reset();
	// pros::lcd::initialize();
    chassis.calibrate();
    // Keep MCL initialized/running, but paused until autonomous starts.
    setMCLPaused(true);
	controller.clear();
	// controller.set_text(0, 0, "Blub Blub");
	// sc.setAutonNames(autonNamesFromTable());
	sc.initialize();

    static pros::Task screen_task([&]() {
        while (true) {
            if (mclPaused.load()) {
                sc.showInfoLabel("MCL paused (enabled in autonomous)");
                pros::delay(100);
                continue;
            }

			lemlib::Pose poseMCL = chassis.getPose();
			lemlib::Pose poseOLD = lemlib::getOldPose();
			std::uint32_t calculationTime = lemlib::getCalculationTime();
			std::int32_t confidenceFront = lemlib::getConfidence();
            char info[192];
            std::snprintf(
                info,
                sizeof(info),
                "MCL X: %.2f Y: %.2f\nTh: %.2f OldX: %.2f OldY: %.2f\nCalc Time: %u  FrontConf: %d",
                poseMCL.x,
                poseMCL.y,
                poseMCL.theta,
                poseOLD.x,
                poseOLD.y,
                static_cast<unsigned>(calculationTime),
                static_cast<int>(confidenceFront));
            sc.showInfoLabel(info);
            pros::delay(100); 
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    sc.state = RobotState::AUTONOMOUS;
		left_motor_group.set_brake_mode(MOTOR_BRAKE_HOLD);
		right_motor_group.set_brake_mode(MOTOR_BRAKE_HOLD);
    setMCLPaused(false);


	// int idx = sc.selectedAuton;
    // if (idx < 0 || idx >= static_cast<int>(AUTONS.size())) idx = 0;
    // if (AUTONS[idx].run != nullptr) AUTONS[idx].run();
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

 bool wedgePressedLast = false; 
bool clawPressedLast = false;
bool clawOpen = false;
bool wedgeDown = false; 
void opcontrol() {


	left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);


    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) ;

        chassis.arcade(leftY, rightX);



        bool cascadeUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool cascadeDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool toggleReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);


        bool wedgePressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);       

        
        if (wedgePressed && !wedgePressedLast) {

            if (wedgeDown) {
                wedge.set_value(true);


            }
            else {
               wedge.set_value(false);
            }

            wedgeDown = !wedgeDown;
        }        

        wedgePressedLast = wedgePressed;




if (cascadeUp) {
    cascade.move(100);
}
else if (cascadeDown) {
    cascade.move(-100);
}
else {
    cascade.move(0);
}

if (cascadeUp) {
    toggle.move(127);
}
else if (cascadeDown || toggleReverse) {
    toggle.move(-127);
}
else {
    toggle.move(0);
}

       bool clawPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
       

        
        if (clawPressed && !clawPressedLast) {

            if (clawOpen) {
                clawy.set_value(true);


            }
            else {
               clawy.set_value(false);
            }

            clawOpen = !clawOpen;
        }        

        clawPressedLast = clawPressed;
        

    

    //   bool togglePressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    //    // double current = roller.get_current_draw();

        
    //    if (togglePressed) {
    //     toggle.move(127); 
    //    } 
    //    else {
    //     toggle.move(0); 
    //    }

    //     pros::delay(20);
    // }

}
	}
