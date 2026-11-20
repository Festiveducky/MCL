#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"

namespace {
lemlib::MCLSettings makeMCLSettings() {
    lemlib::MCLSettings cfg(500); // Number of particles tracked by MCL.

    cfg.distanceSensors = {
    {20, {0.0f, 0.25f, 0.0f}},             // Front
    {3, {-4.75f, -0.5f, -M_PI / 2.0f}},   // Left
    {12, {4.75f, -0.5f, M_PI / 2.0f}},     // Right
    {16, {5.125f, 2.5f, M_PI}}              // Back
 };

    cfg.sigma0XY = 0.06f;         // Baseline XY process noise each cycle (in).
    cfg.kDistXY = 0.58f;          // Extra XY noise per inch translated.
    cfg.kTurnXY = 0.24f;          // Extra XY noise per radian turned.
    cfg.maxStartPosErrorIn = 2.0f; // Initial particle spread radius (in).
    cfg.neffResampleThreshold = 0.80f; // Resample when Neff drops below this fraction of particle count.

    cfg.clampDeltaSForNoise = true; // Caps translation before noise scaling.
    cfg.maxDeltaSForNoise = 3.0f;   // Max translation used in noise model (in).
    cfg.clampSigmaXY = true;        // Caps computed sigmaXY to prevent blowups.
    cfg.maxSigmaXY = 1.75f;         // Upper bound for sigmaXY (in).

    cfg.estMsBandwidth = 4.5f;      // Mean-shift kernel radius (in).
    cfg.estMsIters = 6;             // Max mean-shift refinement iterations.
    cfg.estMsEpsStop = 0.1f;        // Mean-shift convergence threshold (in).
    cfg.estUseHuberRefinement = true; // Enables robust Huber refinement pass.
    cfg.estUseEmaSmoothing = true;  // Enables adaptive EMA smoothing on estimated XY pose.
    cfg.estHuberIters = 3;          // Max Huber refinement iterations.
    cfg.estHuberGateMult = 2.0f;    // Huber neighborhood gate as bandwidth multiplier.
    cfg.estHuberDeltaMult = 0.5f;   // Huber delta as bandwidth multiplier.
    cfg.estAlphaMin = 0.10f;        // Min smoothing factor for final pose EMA.
    cfg.estAlphaMax = 0.80f;        // Max smoothing factor for final pose EMA.
    cfg.estSigmaLo = 2.0f;          // Spread where EMA starts using higher smoothing.
    cfg.estSigmaHi = 11.0f;         // Spread where EMA reaches max smoothing.
    cfg.estJumpThresh = 22.0f;      // Innovation threshold for jump suppression (in).
    cfg.estAlphaJump = 0.84f;       // EMA alpha used when a jump is detected.

    cfg.clampOobParticles = true;   // Clamps particles to field bounds if out of bounds.
    cfg.penalizeOobParticles = true; // Downweights particles that leave bounds.
    cfg.oobWeightMult = 1e-3;       // Weight multiplier applied to OOB particles.
    cfg.useFieldMargin = true;      // Shrinks particle-valid region inward from walls.
    cfg.fieldMarginIn = 5.5f;       // Margin size from field walls for particle bounds (in).
    cfg.useSensorConfidence = true; // Uses sensor confidence to blend likelihood strength.
    cfg.sensorConfMax = 63.0f;      // Confidence value mapped to full trust.
    cfg.useNoHitModel = true;       // Treats >zMax as explicit "no wall hit" evidence.
    cfg.noHitPenalty = 0.035f;      // Penalty if particle expected a wall during no-hit.

    cfg.fieldHalf = 70.75f;         // Half field size from center to wall (in).
    cfg.zMin = 0.1f;                // Minimum accepted distance measurement (in).
    cfg.zMax = 85.0f;               // Maximum accepted distance measurement (in).
    cfg.sigmaD = 3.0f;              // Sensor model triangle half-width scale (in).
    cfg.pFloor = 5e-5;              // Minimum per-sensor likelihood floor.
    cfg.wHit = 0.90f;               // Reserved hit-model blend weight (for future model variants).
    cfg.wRand = 1.0f - cfg.wHit;    // Reserved random-model blend weight.

    return cfg;
}
} // namespace

lemlib::MCLSettings settings = makeMCLSettings();


pros::adi::DigitalOut clawy ('G'); 
pros::adi::DigitalOut wedge ('H');
pros::MotorGroup left_motor_group({-7, -6, 8}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({18,17,-19}, pros::MotorGears::blue);


// Lift
pros::MotorGroup lift_motors ({-2,3},pros::v5::MotorGears::green /*to be specified!*/,pros::v5::MotorEncoderUnits::degrees); // the lift has two motors
pros::MotorGroup cascade({13, -10});
pros::Motor toggle(9);//toggle rn

  //bool rollerSpin = false;

   //pros::Task clawTaskHandle(clawTask);


    //   bool togglePressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    //    // double current = roller.get_current_draw();

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              12, // track width
                              lemlib::Omniwheel::NEW_325,
                              450, // drivetrain rpm
                              2 // horizontal drift
);

pros::Imu imu(2);

pros::Rotation horizontal_rotation(-21);
// vertical tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_2, .125);
// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1 &vertical_tracking_wheel
                            nullptr, // vertical tracking wheel 2
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);


lemlib::ControllerSettings
    lateral_controller(5, // proportional gain (kP) //6.5
            0,    // integral gain (kI)
            0.09,    // derivative gain (kD)
            4,    // anti windup
            .5,   // small error range, in inches
            100,  // small error range timeout, in milliseconds
            .7,   // large error range, in inches
            2000, // large error range timeout, in milliseconds
            20    // maximum acceleration (slew)
    );

lemlib::ControllerSettings
    angular_controller(1.2, // proportional gain (kP)
            0.01,     // integral gain (kI)
            0.7,    // derivative gain (kD)
            3,     // anti windup
            .5,    // small error range, in degrees
            500,   // small error range timeout, in milliseconds
            1,     // large error range, in degrees
            800,   // large error range timeout, in milliseconds
            0      // maximum acceleration (slew)
    );
// lateral PID controller


lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
); // 1.03

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
); // 1.03

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        settings,
                        &throttle_curve, 
                        &steer_curve
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);



// pros::adi::DigitalOut descore('C', false);