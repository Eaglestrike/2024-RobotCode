#include "Util/Mechanism.h"

#include "frc/smartdashboard/SmartDashboard.h"

/**
 * Default Constructor
 * 
 * Sets the name to "Unnamed Mechanism" and does not print to shuffleboard
*/
Mechanism::Mechanism():
    name_("Unnamed Mechanism"),
    enabled_(true),
    shuffleboard_(false)
{

}

/**
 * Constructor
 * 
 * @param name name of mechanism as a string
 * @param enabled enable core code execution
 * @param shuffleboard enable shuffleboard code execution
*/
Mechanism::Mechanism(std::string name, bool enabled, bool shuffleboard):
    name_(name),
    enabled_(enabled),
    shuffleboard_(shuffleboard)
{

}

/**
 * Initial call (should be called once either in another Init or RobotInit)
*/
void Mechanism::Init(){
    if(enabled_){
        CoreInit();
    }
    if(shuffleboard_){
        CoreShuffleboardInit();
    }
};

/**
 * Periodic call (should be called in another Periodic or RobotPeriodic)
*/
void Mechanism::Periodic(){
    if(enabled_){
        CorePeriodic();
    }
    if(shuffleboard_){
        CoreShuffleboardPeriodic();
    }
}


/**
 * Autonomous Initial call (should be called in another AutonomousInit)
*/
void Mechanism::AutonomousInit(){
    if(enabled_){
        CoreAutonomousInit();
    }
}

/**
 * Autonomous Periodic call (should be called in another AutonomousPeriodic)
*/
void Mechanism::AutonomousPeriodic(){
    if(enabled_){
        CoreAutonomousPeriodic();
    }
}

/**
 * Teleoperated Initial call (should be called in another TeleopInit)
*/
void Mechanism::TeleopInit(){
    if(enabled_){
        CoreTeleopInit();
    }
}

/**
 * Teleop Periodic call (should be called in another TeleopPeriodic)
*/
void Mechanism::TeleopPeriodic(){
    if(enabled_){
        CoreTeleopPeriodic();
    }
}

/**
 * Disabled Initial call (should be called in another DisabledInit)
*/
void Mechanism::DisabledInit(){
    if(enabled_){
        CoreDisabledInit();
    }
}

/**
 * Disabled Periodic call (should be called in another DisabledPeriodic)
*/
void Mechanism::DisabledPeriodic(){
    if(enabled_){
        CoreDisabledPeriodic();
    }
}

/**
 * Enables Shuffleboard functions to be executed
 * 
 * Calls ShuffleboardInit
*/
void Mechanism::EnableShuffleboard(){
    CoreShuffleboardInit();
    shuffleboard_ = true;
}

/**
 * Disables Shuffleboard fuctions to be executed
*/
void Mechanism::DisableShuffleboard(){
    shuffleboard_ = false;
}

/**
 * Call to run any Shuffleboard functionality that needs to be executed once
*/
void Mechanism::UpdateShuffleboard(){
    if(enabled_){
        CoreShuffleboardUpdate();
    }
}

/**
 * Disables the mechanism
 * 
 * Prevents Core execution
*/
void Mechanism::Disable(){
    enabled_ = false;
}

/**
 * Enables the mechanism
 * 
 * Enables Core execution
*/
void Mechanism::Enable(){
    enabled_ = true;
}

/**
 * Returns if the mechanism is currently enabled
 * 
 * @returns bool
*/
bool Mechanism::isEnabled(){
    return enabled_;
}

/**
 * Virtual function to implement
*/
void Mechanism::CoreInit(){

}

/**
 * Virtual function to implement
*/
void Mechanism::CorePeriodic(){

}

/**
 * Virtual function to implement
*/
void Mechanism::CoreAutonomousInit(){

}

/**
 * Virtual function to implement
*/
void Mechanism::CoreAutonomousPeriodic(){

}

/**
 * Virtual function to implement
*/
void Mechanism::CoreTeleopInit(){

}

/**
 * Virtual function to implement
*/
void Mechanism::CoreTeleopPeriodic(){

}

/**
 * Virtual function to implement
*/
void Mechanism::CoreDisabledInit(){

}

/**
 * Virtual function to implement
*/
void Mechanism::CoreDisabledPeriodic(){

}

/**
 * Virtual function to implement
*/
void Mechanism::CoreShuffleboardInit(){
    frc::SmartDashboard::PutBoolean(name_ + " enabled", enabled_);
}

/**
 * Virtual function to implement
*/
void Mechanism::CoreShuffleboardPeriodic(){
    //frc::SmartDashboard::PutBoolean(name_ + " enabled", enabled_);
    enabled_ = frc::SmartDashboard::GetBoolean(name_ + " enabled", true);
}

/**
 * Virtual function to implement
*/
void Mechanism::CoreShuffleboardUpdate(){
    enabled_ = frc::SmartDashboard::GetBoolean(name_ + " enabled", true);
}