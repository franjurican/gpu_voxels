#ifndef TIMER_WRAPPER_H
#define TIMER_WRAPPER_H

// C++ stl
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>

class TimerWrapper
{
    public:

        /* 
            Init (ros::int) and start (ros::start or create node handle) ROS before using this class! 
            \param name - timers name
            \param useWallTime - if true timer uses WallTime, if false timer uses ros::Time which can use 
            /clock topic (if rosparam /use_sim_time = true, timer uses /clock topic) 
            \param waitForClock - blocks until /clock sends first measage
        */
        TimerWrapper(std::string name, bool useWallTime = true, bool waitForClock = false);

        /* Empty destructor */
        ~TimerWrapper();

        /* Start measuring time. */
        void tic();

        /* 
            Call this method after \method tic().
            Print elapsed time after tic() call in [ms].
            \return - elapsed time
        */
        double tocMili();

        /* 
            Call this method after \method tic().
            Print elapsed time after tic() call in [ms].
            \param printInfo - info that is printed with elapsed time
            \return - elapsed time
        */
        double tocMili(std::string printInfo);

        /* 
            Call this method after \method tic().
            Print elapsed time after tic() call in [us].
            \return - elapsed time
        */
        double tocMikro();

        /* 
            Call this method after \method tic().
            Print elapsed time after tic() call in [us].
            \param printInfo - info that is printed with elapsed time
            \return - elapsed time
        */
        double tocMikro(std::string printInfo);

        /* 
            Call this method after \method tic().
            Print elapsed time after tic() call in [ns].
            \return - elapsed time
        */
        uint64_t tocNano();

        /* 
            Call this method after \method tic().
            Print elapsed time after tic() call in [ns].
            \param printInfo - info that is printed with elapsed time
            \return - elapsed time
        */
        uint64_t tocNano(std::string printInfo);

    private:
        /* Time */
        uint64_t t1, t2;
        bool wallTime; 
        std::string name;
};
#endif //TIMER_WRAPPER_H