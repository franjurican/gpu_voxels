// header for TimerWrapper class
#include <helpers_gpu_voxels/timer/timer_wrapper.h>

// format iostream
#include <iomanip>

TimerWrapper::TimerWrapper(std::string name, bool useWallTime, bool waitForClock) 
{
    this->wallTime = useWallTime;
    this->name = name;

    // timer time description
    if (this->wallTime)
        ROS_INFO_STREAM("Timer \"" << this->name << "\" koristi WallTime.");
    else
        ROS_INFO_STREAM("Timer \"" << this->name << "\" koristi ros::Time koji koristi " << 
                                                        (ros::Time::isSimTime() ? "Sim Time (topic /clock)." : "WallTime."));

    // wait for clock
    if(waitForClock)
    {
        ROS_INFO_STREAM("Timer \"" << this->name << "\" ceka da vrijeme postane razlicito od 0! Timer ceka /clock!");
        while (!ros::Time::isValid()){}
    }

    ROS_INFO_STREAM("Timer \"" << this->name << "\" inicijaliziran.");       
}

TimerWrapper::~TimerWrapper() {}

void TimerWrapper::tic()
{
    this->t1 = (this->wallTime ? ros::WallTime::now().toNSec() : ros::Time::now().toNSec());
}

double TimerWrapper::tocMili()
{  
    this->t2 = (this->wallTime ? ros::WallTime::now().toNSec() : ros::Time::now().toNSec());
    ROS_INFO_STREAM("Timer \"" << this->name << "\": " << std::setprecision(5) << (this->t2 - this->t1)*1e-6 << " [ms].");
    return (this->t2 - this->t1)*1e-6;
}

double TimerWrapper::tocMili(std::string printInfo)
{  
    this->t2 = (this->wallTime ? ros::WallTime::now().toNSec() : ros::Time::now().toNSec());
    ROS_INFO_STREAM("Timer \"" << this->name << "\" - " << printInfo << ": " << std::setprecision(5) << (this->t2 - this->t1)*1e-6 << " [ms].");
    return (this->t2 - this->t1)*1e-6;
}

double TimerWrapper::tocMikro()
{  
    this->t2 = (this->wallTime ? ros::WallTime::now().toNSec() : ros::Time::now().toNSec());
    ROS_INFO_STREAM("Timer \"" << this->name << "\": " << std::setprecision(5) << (this->t2 - this->t1)*1e-3 << " [us].");
    return (this->t2 - this->t1)*1e-3;
}

double TimerWrapper::tocMikro(std::string printInfo)
{  
    this->t2 = (this->wallTime ? ros::WallTime::now().toNSec() : ros::Time::now().toNSec());
    ROS_INFO_STREAM("Timer \"" << this->name << "\" - " << printInfo << ": " << std::setprecision(5) << (this->t2 - this->t1)*1e-3 << " [us].");
    return (this->t2 - this->t1)*1e-3;
}

uint64_t TimerWrapper::tocNano()
{  
    this->t2 = (this->wallTime ? ros::WallTime::now().toNSec() : ros::Time::now().toNSec());
    ROS_INFO_STREAM("Timer \"" << this->name << "\": " << (this->t2 - this->t1) << " [ns].");
    return this->t2 - this->t1;
}

uint64_t TimerWrapper::tocNano(std::string printInfo)
{  
    this->t2 = (this->wallTime ? ros::WallTime::now().toNSec() : ros::Time::now().toNSec());
    ROS_INFO_STREAM("Timer \"" << this->name << "\" - " << printInfo << ": " << (this->t2 - this->t1) << " [ns].");
    return this->t2 - this->t1;
}