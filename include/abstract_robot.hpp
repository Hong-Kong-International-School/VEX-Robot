#ifndef ABSTRACT_BOT
#define ABSTRACT_BOT

#include <vex.h>
#include <functional>
#include <chrono>

class Abstract_Robot {

    protected:
        
        struct target_data {
            double x;
            double y;
            double angle;
            target_data();
        };
        struct target_units {
            vex::distanceUnits d_u;
            vex::rotationUnits a_u;
            target_units();
        };
        struct target_tolerance {
            double length_tolerance;
            double angle_tolerance;
            target_tolerance();
        };

    public:

        virtual target_data make_target(double x, double y, double angle) = 0;
        virtual target_units make_units(vex::distanceUnits d_u, vex::rotationUnits a_u) = 0;
        virtual target_units default_units() = 0;
        virtual target_tolerance default_tolerance() = 0;


        virtual void move(target_data target, target_tolerance tolerance, target_units units, int tick_length) = 0;
        virtual void turn(double angle, vex::rotationUnits a_u) = 0;
        virtual void set_heading(double angle, vex::rotationUnits a_u) = 0;


        class Clock {
            private:
                std::chrono::time_point<std::chrono::high_resolution_clock> clock_birth;
            public:
                Clock();
                unsigned long long int now();
        };
        virtual void do_for(int time, std::function<void()> func, std::function<void()> termination) = 0;
        virtual void time_move(double speed, int time) = 0;
        virtual void time_turn(double speed, int time) = 0;


        virtual void reset(bool reset_gps = false) = 0;


        virtual int control(std::function<bool()> break_condition, std::function<void()> exit_exec) = 0;

};

#endif