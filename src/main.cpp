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

#include <vex.h>
#include <iostream>
#include "xdrive.hpp"
#include "ports.hpp"

bool condition() { return false; }
void nothing() {}

int main() {
    vex::brain Brain = vex::brain();
    // vex::gps Gps = vex::gps(GPS, 0, vex::turnType::right);
    // initializing drivetrain
    // Brain.Screen.print("KSJFKSJSKDJFKSDFJ\n");
    // Brain.Screen.newLine();
    // Brain.Screen.print("%lf.2 %lf.2 %lf.2\n", Gps.xPosition(), Gps.yPosition(), Gps.heading());
    // while (1) {
        // Brain.Screen.newLine();
        // Brain.Screen.print("%lf.2 %lf.2 %lf.2", Gps.xPosition(), Gps.yPosition(), Gps.heading());
        // Brain.Screen.print("fuck you");
        // vexDelay(1000);
        // Brain.Screen.clearScreen();
    // }
    bool gpstest = true;
    if (gpstest) {
        Brain.Screen.print("SDKFJKSDJFKSDJFKSJDKFJSKDFJKSJDKFJSKDJFSKDJKSDJFKSDJFKSJDKJDSF");
        Brain.Screen.newLine();
        vex::gps Gps = vex::gps(GPS, 0, vex::turnType::left);
        std::cout << std::boolalpha << Gps.installed() << '\n';
        Gps.startCalibration();
        Gps.calibrate();
        while (Gps.isCalibrating()) vexDelay(1000);
        printf("DONECAL\n");
        vexDelay(5000);
        Brain.Screen.clearScreen();
        Brain.Screen.print("%.2lf %.2lf %.2lf\n", Gps.heading(), Gps.xPosition(), Gps.yPosition());
        std::cout << Gps.heading() << ' ' << Gps.xPosition() / 1000 << ' ' << Gps.yPosition() / 1000 << '\n';
    }

    // vex::motor left_motor1 = vex::motor(LM1, vex::gearSetting::ratio18_1, false);
    // vex::motor left_motor2 = vex::motor(LM2, vex::gearSetting::ratio18_1, false);
    // vex::motor right_motor1 = vex::motor(RM1, vex::gearSetting::ratio18_1, true);
    // vex::motor right_motor2 = vex::motor(RM2, vex::gearSetting::ratio18_1, true);
    // vex::gps Gps = vex::gps(GPS, 0.0, vex::turnType::left);
    // XDrive base(left_motor1, left_motor2, right_motor1, right_motor2, Gps);
    // XDrive base(LM1, false, LM2, false, RM1, true, RM2, true, vex::gearSetting::ratio18_1, GPS, vex::turnType::left);
    
    if (!gpstest) {
        XDrive base;
        Brain.Screen.print("!"), Brain.Screen.newLine();
        base.toggle_display_debug_variable_values(); // disable for competition
        Brain.Screen.print("!"), Brain.Screen.newLine();
        base.move(base.make_target(0, 0, 0), base.default_tolerance(), base.default_units(), 25);
        Brain.Screen.print("!"), Brain.Screen.newLine();
    }
}