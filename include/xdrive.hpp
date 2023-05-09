#ifndef X_DRIVETRAIN
#define X_DRIVETRAIN

#include <vex.h>
#include "abstract_robot.hpp"
#include "ports.hpp"

class XDrive : virtual public Abstract_Robot {

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

    private:

        /////////////////////////// BEGIN DRIVETRAIN VARIABLES ///////////////////////////

        const double kP = 0.12, kI = 0.075, kD = 0.0; // adjust PID constants
        const double kP_angle = 0.1, kI_angle = 0, kD_angle = 0; // adjust PID constants

        const int default_tick_length = 25;

        double motor_group1_speed, motor_group2_speed; // speed of motor groups
        double turnspeed; // turning speed

        double x_position, y_position; // (x_position, y_position) is the current coordinate
        double x_error, y_error; // error = target - position
        double axis1_error, axis2_error; // error in terms of the axes
        double last_axis1_error, last_axis2_error; // derivative: last error
        double axis1_error_sum, axis2_error_sum; // integral: sum of errors
        double axis1_proportional, axis1_integral, axis1_derivative;
        double axis2_proportional, axis2_integral, axis2_derivative;

        double heading; // measure of angle with positive x axis as initial side, GF as terminal side
        double heading_error; // error = target - heading
        double last_heading_error; // derivative
        double heading_error_sum; // integral
        double heading_proportional, heading_integral, heading_derivative;

        double sine, cosine;

        bool debug_display;

        double left_motor1_speed, left_motor2_speed, right_motor1_speed, right_motor2_speed;

        /////////////////////////// CLOSE DRIVETRAIN VARIABLES ///////////////////////////



        /////////////////////////// BEGIN DEVICES ///////////////////////////

        vex::brain Brain;
    	vex::motor left_motor1 = vex::motor(LM1, vex::ratio18_1, true);
	    vex::motor left_motor2 = vex::motor(LM2, vex::ratio18_1, true);
	    vex::motor right_motor1 = vex::motor(RM1, vex::ratio18_1, false);
	    vex::motor right_motor2 = vex::motor(RM2, vex::ratio18_1, false);
        vex::gps Gps = vex::gps(GPS, 0, vex::turnType::left);

        /////////////////////////// CLOSE DEVICES ///////////////////////////



        /////////////////////////// BEGIN FUNCTIONS ///////////////////////////

        inline double dsin(double angle);
        inline double dcos(double angle);
        inline double abs(double val);
        void adjust_heading(const target_data & target, const target_units & units);
        inline void spin();
        inline void test();
        inline void brake();

        /////////////////////////// CLOSE FUNCTIONS ///////////////////////////

    public:

        /////////////////////////// BEGIN NESTED CLASS ///////////////////////////

        class asic {

            public:

                // drive

        };

        /////////////////////////// CLOSE NESTED CLASS ///////////////////////////

        /////////////////////////// BEGIN CONSTRUCTOR ///////////////////////////

        XDrive(vex::motor lm1, vex::motor lm2, vex::motor rm1, vex::motor rm2, vex::gps gps);
        XDrive(int lm1_port, bool lm1_reverse, int lm2_port, bool lm2_reverse, int rm1_port, bool rm1_reverse, int rm2_port, bool rm2_reverse, vex::gearSetting gears, int gps_port, vex::turnType turn_type);
        XDrive();

        /////////////////////////// CLOSE CONSTRUCTOR ///////////////////////////



        /////////////////////////// BEGIN STRUCTURE FUNCTIONS ///////////////////////////

        target_data make_target(double x, double y, double angle);
        target_units make_units(vex::distanceUnits d_u, vex::rotationUnits a_u);
        target_units default_units();
        target_tolerance default_tolerance();

        /////////////////////////// CLOSE STRUCTURE FUNCTIONS ///////////////////////////

        

        /////////////////////////// BEGIN POSITION-BASED AUTONOMOUS FUNCTIONS ///////////////////////////

        void move(target_data target, target_tolerance tolerance, target_units units, int tick_length);
        void turn(double angle, vex::rotationUnits a_u);
        void set_heading(double angle, vex::rotationUnits a_u);

        /////////////////////////// CLOSE POSITION-BASED AUTONOMOUS FUNCTIONS ///////////////////////////



        /////////////////////////// BEGIN TIME-BASED AUTONOMOUS FUNCTIONS ///////////////////////////

        void do_for(int time, std::function<void()> func, std::function<void()> termination);
        void time_move(double speed, int time);
        void time_turn(double speed, int time);

        /////////////////////////// CLOSE TIME-BASED AUTONOMOUS FUNCTIONS ///////////////////////////



        /////////////////////////// BEGIN DEBUG FUNCTIONS ///////////////////////////

        void print_schematic();
        void toggle_display_debug_variable_values();
        void reset(bool reset_gps = false);

        /////////////////////////// CLOSE DEBUG FUNCTIONS ///////////////////////////



        /////////////////////////// BEGIN CONTROL ///////////////////////////

        int control(std::function<bool()> break_condition, std::function<void()> exit_exec);

        /////////////////////////// CLOSE CONTROL ///////////////////////////
};

#endif