#include "main.h"
#include "p25/auton_selector.hpp"
#include "p25/timing.hpp"
#include "p25/shmovement_drivetrain.hpp"
#include "p25/intake.hpp"
#include "p25/kicker.hpp"
#include "p25/piston.hpp"

// --- pros devices ---

pros::Imu imu(10);

pros::Motor motor_left1 (12, MOTOR_GEAR_BLUE,  true);
pros::Motor motor_left2 (13, MOTOR_GEAR_BLUE,  true);
pros::Motor motor_left3 (14, MOTOR_GEAR_BLUE,  true);
pros::Motor motor_left4 (19, MOTOR_GEAR_GREEN, false);
pros::Motor motor_right1(15, MOTOR_GEAR_BLUE, false);
pros::Motor motor_right2(16, MOTOR_GEAR_BLUE, false); // back
pros::Motor motor_right3(17, MOTOR_GEAR_BLUE, false);
pros::Motor motor_right4(18, MOTOR_GEAR_GREEN, true);

pros::MotorGroup mg_L ({
    motor_left1,
    motor_left2,
    motor_left3,
    motor_left4
});

pros::MotorGroup mg_R ({
    motor_right1,
    motor_right2,
    motor_right3,
    motor_right4
});

pros::Motor motor_intake(9, MOTOR_GEAR_BLUE, true);
pros::MotorGroup mg_intake({motor_intake});

pros::Controller controller(CONTROLLER_MASTER);

pros::ADIDigitalOut piston_liftClamp('a', false);
pros::ADIDigitalOut piston_pto      ('b', false);
pros::ADIDigitalOut piston_rachet   ('c', false);
pros::ADIDigitalOut piston_leftWing ('d', false);
pros::ADIDigitalOut piston_rightWing('e', false);

// --- p25 subsystems ---
p25::ShmovementDrivetrain drivetrain(
    &mg_L,
    &mg_R,
    &imu
);

p25::Intake intake(&mg_intake);

p25::Piston clamp(&piston_liftClamp);
p25::Piston pto(&piston_pto);
p25::Piston ratchet(&piston_rachet);

p25::Wings wings(&piston_leftWing, &piston_rightWing);

p25::TimingSystem timer;

// --- helper functions ---

constexpr long double operator""_in(long double inches)
{
    return inches * 39.178;
}

constexpr long double operator""_in(unsigned long long int inches)
{
    return inches * 39.178;
}

// --- autons ---

p25::AutonSelector auton_selector(
{
{
    name: "Skills",
    setup_func: [] {
        
    },
    auton_func: [] {

    }
},
{
    name: "Close WP",
    setup_func: [] {

    },
    auton_func: [] {
    }
},
{
    name: "Far WP",
    setup_func: [] {    

    },
    auton_func: [] {

    }
},
{
    name: "Close Bracket",
    setup_func: [] {

    },
    auton_func: [] {

    }
},
{
    name: "Far Bracket",
    setup_func: [] {

    },
    auton_func: [] {

    }
}
},
&controller
);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    auton_selector.autonomous();
}

// --- driver control ---

bool is_elevating = false;

void opcontrol() { 
    auton_selector.opcontrol();
    
    drivetrain.coast();
    
    while (true) {
        if (is_elevating) {
            handleElevationControls();
        } else {
            handleStandardControls();
        }

    pros::delay(5);
    }
}

void handleStandardControls() {
    drivetrain.tankDrive(
        controller.get_analog(ANALOG_LEFT_Y),
        controller.get_analog(ANALOG_RIGHT_Y)
    );

    if (controller.get_digital(DIGITAL_R1)) {
        intake.enable();
    } else if (controller.get_digital(DIGITAL_R2)) {
        intake.reverse();
    } else {
        intake.coast();
    }

    if (controller.get_digital_new_press(DIGITAL_L1)) {
        wings.deployBoth();
    } else if (controller.get_digital_new_press(DIGITAL_L2)) {
        wings.retractBoth();
    }

    if (controller.get_digital_new_press(DIGITAL_A) &&
        controller.get_digital_new_press(DIGITAL_LEFT)) {
        controller.rumble("-");
        is_elevating = true;
        pto.retract();
    }
}

void handleElevationControls() {
    if (controller.get_digital_new_press(DIGITAL_X)) {
        clamp.toggle();
    }

    if (controller.get_digital_new_press(DIGITAL_Y)) {
        ratchet.toggle();
    }

    if (controller.get_digital(DIGITAL_UP)) {
        drivetrain.coast();
        drivetrain.tankDrive(127, 127);
    } else if (controller.get_digital(DIGITAL_DOWN)) {
        drivetrain.coast();
        drivetrain.tankDrive(-127, -127);
    } else {
        drivetrain.brake();
    }
}