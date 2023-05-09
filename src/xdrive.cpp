//////////////////////////////////  SCHEMATIC  ///////////////////////////////////////////
//            F                                                                         //
//     LM1          RM1                                                                 //
//     /            \                                                                   //
//                                                                                      //
//            G                                                                         //
//                                                                                      //
//     \            /                                                                   //
//     LM2          RM2                                                                 //
//                                                                                      //
//     * RM1(+), LM2(-) form the axis1 direction                                        //
//     * LM1(+), RM2(-) form the axis2 direction                                        //
//     * LM1 and RM2 belong to motor_group1: positive power yields northeast motion     //
//     * RM1 and LM2 belong to motor_group2: positive power yields northwest motion     //
//     * G is the GPS, facing toward F                                                  //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////// START PROGRAM //////////////////////////////////////////////////////

#include <vex.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include "xdrive.hpp"

/////////////////////////// BEGIN DRIVETRAIN CONSTRUCTOR ///////////////////////////

XDrive::XDrive(vex::motor lm1, vex::motor lm2, vex::motor rm1, vex::motor rm2, vex::gps gps) {
    left_motor1 = lm1;
    left_motor2 = lm2;
    right_motor1 = rm1;
    right_motor2 = rm2;
    Gps = gps;
    reset(false);
}

XDrive::XDrive(int lm1_port, bool lm1_reverse, int lm2_port, bool lm2_reverse, int rm1_port, bool rm1_reverse, int rm2_port, bool rm2_reverse, vex::gearSetting gears, int gps_port, vex::turnType turn_type) {
    left_motor1 = vex::motor(lm1_port, gears, lm1_reverse);
    left_motor2 = vex::motor(lm2_port, gears, lm2_reverse);
    right_motor1 = vex::motor(rm1_port, gears, rm1_reverse);
    right_motor2 = vex::motor(rm2_port, gears, rm2_reverse);
    Gps = vex::gps(gps_port, 0.0, turn_type);
    reset(false);
}

XDrive::XDrive() {
    left_motor1 = vex::motor(LM1, vex::gearSetting::ratio18_1, false);
    left_motor2 = vex::motor(LM2, vex::gearSetting::ratio18_1, false);
    right_motor1 = vex::motor(RM1, vex::gearSetting::ratio18_1, true);
    right_motor2 = vex::motor(RM2, vex::gearSetting::ratio18_1, true);
    Gps = vex::gps(GPS, 0.0, vex::turnType::left);
}

/////////////////////////// CLOSE DRIVETRAIN CONSTRUCTOR ///////////////////////////


/////////////////////////// BEGIN TRIG FUNCTIONS ///////////////////////////

inline double XDrive::dcos(double angle) {
    return cos(angle / 180 * M_PI);
}


inline double XDrive::dsin(double angle) {
    return sin(angle / 180 * M_PI);
}

/////////////////////////// CLOSE TRIG FUNCTIONS ///////////////////////////

/////////////////////////// BEGIN ABS ///////////////////////////

inline double XDrive::abs(double val) {
    return val < 0 ? -val : val;
}

/////////////////////////// CLOSE ABS ///////////////////////////


/////////////////////////// BEGIN TEST ///////////////////////////

inline void XDrive::test() {
    left_motor1.spin(vex::directionType::fwd, 10, vex::percentUnits::pct);
    left_motor2.spin(vex::directionType::fwd, 10, vex::percentUnits::pct);
    right_motor1.spin(vex::directionType::fwd, 10, vex::percentUnits::pct);
    right_motor2.spin(vex::directionType::fwd, 10, vex::percentUnits::pct);
}

/////////////////////////// CLOSE TEST ///////////////////////////


/////////////////////////// BEGIN INTERNAL MOTOR FUNCTIONS ///////////////////////////

inline void XDrive::spin() {
    left_motor1.spin(vex::directionType::fwd, left_motor1_speed, vex::percentUnits::pct);
    right_motor2.spin(vex::directionType::fwd, right_motor2_speed, vex::percentUnits::pct);
    right_motor1.spin(vex::directionType::fwd, right_motor1_speed, vex::percentUnits::pct);
    left_motor2.spin(vex::directionType::fwd, left_motor2_speed, vex::percentUnits::pct);
}


inline void XDrive::brake() {
    left_motor1.stop();
    left_motor2.stop();
    right_motor1.stop();
    right_motor2.stop();
}

/////////////////////////// CLOSE INTERNAL MOTOR FUNCTIONS ///////////////////////////


/////////////////////////// BEGIN STRUCTURE CONSTRUCTORS ///////////////////////////

XDrive::target_data::target_data() {
    x = y = angle = 0.0;
}


XDrive::target_units::target_units() {
    d_u = vex::distanceUnits::mm;
    a_u = vex::rotationUnits::deg;
}


XDrive::target_tolerance::target_tolerance() {
    length_tolerance = 100.0;
    angle_tolerance = 5.0;
}

/////////////////////////// CLOSE STRUCTURE CONSTRUCTORS ///////////////////////////


/////////////////////////// BEGIN MAKE STRUCTURES ///////////////////////////

XDrive::target_data XDrive::make_target(double x, double y, double angle) {
    target_data ret;
    ret.x = x;
    ret.y = y;
    ret.angle = angle;
    return ret;
}


XDrive::target_units XDrive::make_units(vex::distanceUnits d_u, vex::rotationUnits a_u) {
    target_units ret;
    ret.a_u = a_u;
    ret.d_u = d_u;
    return ret;
}

/////////////////////////// BEGIN DEFAULTS ///////////////////////////

XDrive::target_units XDrive::default_units() {
    target_units ret;
    ret.a_u = vex::deg;
    ret.d_u = vex::mm;
    return ret;
}


XDrive::target_tolerance XDrive::default_tolerance() {
    target_tolerance ret;
    ret.length_tolerance = 100.0;
    ret.angle_tolerance = 5.0;
    return ret;
}

/////////////////////////// CLOSE DEFAULTS ///////////////////////////

/////////////////////////// CLOSE MAKE STRUCTURES ///////////////////////////

/////////////////////////// BEGIN CALCULATION ///////////////////////////

void XDrive::adjust_heading(const target_data & target, const target_units & units) {
    // get angle
    heading = Gps.heading(units.a_u);
    // get angle error
    heading_error = target.angle - heading;
    // angle proportional
    heading_proportional = heading_error * kP_angle;
    // angle integral
    heading_error_sum += heading_error;
    heading_integral = heading_error_sum * kI_angle;
    // angle derivative
    heading_derivative = heading_error - last_heading_error;
    // turn speed
    turnspeed = heading_proportional + heading_integral + heading_derivative;
    // get position
    x_position = Gps.xPosition(units.d_u);
    y_position = Gps.yPosition(units.d_u);
    // get position error
    x_error = target.x - x_position;
    y_error = target.y - y_position;
    // calculate axis_error
    sine = dsin(heading + 45);
    cosine = dcos(heading + 45);
    axis1_error = (cosine * x_error) + (sine * y_error);
    axis2_error = (cosine * y_error) - (sine * x_error);
    // axis_proportional
    axis1_proportional = axis1_error * kP;
    axis2_proportional = axis2_error * kP;
    // axis_integral
    axis1_error_sum += axis1_error;
    axis2_error_sum += axis2_error;
    axis1_integral = axis1_error_sum * kI;
    axis2_integral = axis2_error_sum * kI;
    // axis_derivative
    axis1_derivative = axis1_error - last_axis1_error;
    axis2_derivative = axis2_error - last_axis2_error;
    // motor group speeds
    motor_group2_speed = axis1_proportional + axis1_integral + axis1_derivative;
    motor_group1_speed = axis2_proportional + axis2_integral + axis2_derivative;
    // update last error
    last_axis1_error = axis1_error;
    last_axis2_error = axis2_error;
    last_heading_error = heading_error;

    if (debug_display) {
        Brain.Screen.clearScreen();
        Brain.Screen.print("Heading: %.2lf; heading error: %.2lf\n", heading, heading_error);
        Brain.Screen.print("sine: %.2lf, cosine: %.2lf\n", sine, cosine);
        Brain.Screen.print("Heading PID values: P- %.2lf, I- %.2lf, D- %.2lf\n", heading_proportional, heading_integral, heading_derivative);
        Brain.Screen.print("XDrive at (%.2lf, %.2lf)\n", x_position, y_position);
        Brain.Screen.print("Cartesian error: (%.2lf, %.2lf)\n", x_error, y_error);
        Brain.Screen.print("Axis errors: (%.2lf, %.2lf)\n", axis1_error, axis2_error);
        Brain.Screen.print("Axis 1 PID values: P- %.2lf, I- %.2lf, D- %.2lf\n", axis1_proportional, axis1_integral, axis1_derivative);
        Brain.Screen.print("Axis 2 PID values: P- %.2lf, I- %.2lf, D- %.2lf\n", axis2_proportional, axis2_integral, axis2_derivative);
        Brain.Screen.print("Motor group speeds: 1- %.2lf, 2- %.2lf\n", motor_group1_speed, motor_group2_speed);
        Brain.Screen.print("Turning speed: %.2lf\n", turnspeed);


        printf("Heading: %.2lf; heading error: %.2lf\n", heading, heading_error);
        printf("sine: %.2lf, cosine: %.2lf\n", sine, cosine);
        printf("Heading PID values: P- %.2lf, I- %.2lf, D- %.2lf\n", heading_proportional, heading_integral, heading_derivative);
        printf("XDrive at (%.2lf, %.2lf)\n", x_position, y_position);
        printf("Cartesian error: (%.2lf, %.2lf)\n", x_error, y_error);
        printf("Axis errors: (%.2lf, %.2lf)\n", axis1_error, axis2_error);
        printf("Axis 1 PID values: P- %.2lf, I- %.2lf, D- %.2lf\n", axis1_proportional, axis1_integral, axis1_derivative);
        printf("Axis 2 PID values: P- %.2lf, I- %.2lf, D- %.2lf\n", axis2_proportional, axis2_integral, axis2_derivative);
        printf("Motor group speeds: 1- %.2lf, 2- %.2lf\n", motor_group1_speed, motor_group2_speed);
        printf("Turning speed: %.2lf\n", turnspeed);
    }
}

/////////////////////////// CLOSE CALCULATION ///////////////////////////


/////////////////////////// BEGIN MOVE FUNCTIONS ///////////////////////////

void XDrive::move(target_data target, target_tolerance tolerance, target_units units, int tick_length) {
    // set initial conditions
    vex::brain Brain = vex::brain();
    heading = Gps.heading(units.a_u);
    x_position = Gps.xPosition(units.d_u), y_position = Gps.yPosition(units.d_u);
    axis1_error_sum = axis2_error_sum = heading_error_sum = 0.0;
    x_error = target.x - x_position, y_error = target.y - y_position;
    sine = dsin(heading + 45), cosine = dcos(heading + 45);
    last_heading_error = target.angle - heading;
    last_axis1_error = (cosine * x_error) + (sine * y_error), last_axis2_error = (cosine * y_error) - (sine * x_error);    
    
    // run while condition
    bool conditions_not_satisfied =     abs(target.x - Gps.xPosition(units.d_u)) > tolerance.length_tolerance or
                                        abs(target.y - Gps.yPosition(units.d_u)) > tolerance.length_tolerance or
                                        abs(target.angle - Gps.heading(units.a_u)) > tolerance.angle_tolerance;

    // std::cout << Gps.xPosition() << ' ' << Gps.yPosition() << ' ' << Gps.heading() << '\n';
    // std::cout << std::boolalpha << conditions_not_satisfied << '\n';
    Brain.Screen.print("%.2lf %.2lf %.2lf",Gps.xPosition(),Gps.yPosition(),Gps.heading());
    Brain.Screen.newLine();
    Brain.Screen.print(conditions_not_satisfied);
    Brain.Screen.newLine();

    while (conditions_not_satisfied) {
        adjust_heading(make_target(target.x, target.y, target.angle), make_units(units.d_u, units.a_u));
        left_motor1_speed = motor_group1_speed - turnspeed;
        right_motor2_speed = motor_group1_speed + turnspeed;
        right_motor1_speed = motor_group2_speed + turnspeed;
        left_motor2_speed = motor_group2_speed - turnspeed;
        spin();
        vexDelay(tick_length);
        conditions_not_satisfied =      abs(target.x - Gps.xPosition(units.d_u)) > tolerance.length_tolerance or
                                        abs(target.y - Gps.yPosition(units.d_u)) > tolerance.length_tolerance or
                                        abs(target.angle - Gps.heading(units.a_u)) > tolerance.angle_tolerance;
    }
}


void XDrive::turn(double angle, vex::rotationUnits a_u) {
    move(make_target(Gps.xPosition(), Gps.yPosition(), Gps.heading(a_u) + angle), default_tolerance(), default_units(), default_tick_length);
}


void XDrive::set_heading(double angle, vex::rotationUnits a_u) {
    move(make_target(Gps.xPosition(), Gps.yPosition(), angle), default_tolerance(), default_units(), default_tick_length);
}

/////////////////////////// CLOSE MOVE FUNCTIONS ///////////////////////////


/////////////////////////// BEGIN TIME_BASED ///////////////////////////

XDrive::Clock::Clock() {
    clock_birth = std::chrono::high_resolution_clock::now();
}


unsigned long long int XDrive::Clock::now() {
    std::chrono::time_point<std::chrono::high_resolution_clock> current = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds cast = std::chrono::duration_cast<std::chrono::milliseconds> (current - clock_birth);
    return (unsigned long long int) cast.count();
}


void XDrive::do_for(int time, std::function<void()> process, std::function<void()> termination) {
    Clock function_clock;
    unsigned long long int time_based_now = function_clock.now();
    unsigned long long int time_based_end = time_based_now + (unsigned long long int) time * 1000ULL;
    while (time_based_now < time_based_end) {
        process();
        time_based_now = function_clock.now();
    }
    termination();
}


void XDrive::time_move(double speed, int time) {
    motor_group1_speed = motor_group2_speed = speed;
    turnspeed = 0.0;
    do_for(time, std::bind(& XDrive::spin, this), std::bind(& XDrive::brake, this));
}


void XDrive::time_turn(double speed, int time) {
    motor_group1_speed = motor_group2_speed = 0.0;
    turnspeed = speed;
    do_for(time, std::bind(& XDrive::spin, this), std::bind(& XDrive::brake, this));
}

/////////////////////////// CLOSE TIME_BASED ///////////////////////////


/////////////////////////// BEGIN DEBUG ///////////////////////////

void XDrive::print_schematic() {
    std::string output = 
R"(
//////////////////////////////////  SCHEMATIC  ///////////////////////////////////////////
//            F                                                                         //
//     LM1          RM1                                                                 //
//     /            \                                                                   //
//                                                                                      //
//            G                                                                         //
//                                                                                      //
//     \            /                                                                   //
//     LM2          RM2                                                                 //
//                                                                                      //
//     * RM1(+), LM2(-) form the axis1 direction                                        //
//     * LM1(+), RM2(-) form the axis2 direction                                        //
//     * LM1 and RM2 belong to motor_group1: positive power yields northeast motion     //
//     * RM1 and LM2 belong to motor_group2: positive power yields northwest motion     //
//     * G is the GPS, facing toward F                                                  //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
)";
    printf("%s\n", output.c_str());
}


void XDrive::toggle_display_debug_variable_values() {
    debug_display = not debug_display;
}

void XDrive::reset(bool reset_gps) {
    motor_group1_speed = motor_group2_speed = turnspeed = 
    x_position = y_position = 
    x_error = y_error = axis1_error = axis2_error = 
    last_axis1_error = last_axis2_error = 
    axis1_error_sum = axis2_error_sum = 
    axis1_proportional = axis1_integral = axis1_derivative = 
    axis2_proportional = axis2_integral = axis2_derivative = 
    heading = heading_error = last_heading_error = heading_error_sum = 
    heading_proportional = heading_integral = heading_derivative = 
    sine = cosine = 0.0;
    debug_display = false;

    if (reset_gps) 
        Gps.calibrate(), Gps.resetHeading(), Gps.resetRotation(), Gps.calibrate();
}

/////////////////////////// CLOSE DEBUG ///////////////////////////


/////////////////////////// BEGIN CONTROL ///////////////////////////

int XDrive::control(std::function<bool()> break_condition, std::function<void()> exit_exec) {
    vex::controller Controller = vex::controller(vex::controllerType::primary);
    double precisionFactor = 1.0;
    double axis3_value, axis1_value, axis4_value;
    bool lastYState = 0;
    bool togglePrecision = 0;

    while (!break_condition()) {
        axis3_value = 1.0 * Controller.Axis3.position();
        axis1_value = 1.0 * Controller.Axis1.position();
        axis4_value = 1.0 * Controller.Axis4.position();

        if (Controller.ButtonY.pressing() != lastYState and lastYState != 1) { 
            if (togglePrecision) 
                togglePrecision = 0; 
            else 
                togglePrecision = 1; 
        }
        if (togglePrecision)
            precisionFactor = 5.0;
        else 
            precisionFactor = 1.0;
        lastYState = Controller.ButtonY.pressing();

        left_motor1_speed = (axis3_value + axis4_value + axis1_value) / precisionFactor;
        left_motor2_speed = (axis3_value - axis4_value + axis1_value) / precisionFactor;
        right_motor1_speed = (axis3_value - axis4_value - axis1_value) / precisionFactor;
        right_motor2_speed = (axis3_value + axis4_value - axis1_value) / precisionFactor;

        /*
         * insert ASICs (autonomous sequence in control) here
         * conditional followed by function call
         * functions defined in namespace asic in file "auto_in_control.hpp"
         */

        spin();
    }
    exit_exec();
    return 0;
}

/////////////////////////// CLOSE DEBUG ///////////////////////////

////////////////////////////////////////////////////// END PROGRAM //////////////////////////////////////////////////////