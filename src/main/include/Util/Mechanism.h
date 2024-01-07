#pragma once

#include <string>

/**
 * Base Mechanism Class
 * 
 * All mechanisms should inherit this parent class
 * Implement needed Core functions
 * Remember to call the public functions elsewhere
 * 
 * https://en.wikipedia.org/wiki/Non-virtual_interface_pattern
*/
class Mechanism{
    public:
        Mechanism();
        Mechanism(std::string name, bool enabled = true, bool shuffleboard = false);

        void Init();
        void Periodic();
        void AutonomousInit();
        void AutonomousPeriodic();
        void TeleopInit();
        void TeleopPeriodic();
        void DisabledInit();
        void DisabledPeriodic();

        void EnableShuffleboard();
        void DisableShuffleboard();
        void UpdateShuffleboard();

        void Disable();
        void Enable();
        bool isEnabled();

    protected:
        virtual void CoreInit();
        virtual void CorePeriodic();
        virtual void CoreAutonomousInit();
        virtual void CoreAutonomousPeriodic();
        virtual void CoreTeleopInit();
        virtual void CoreTeleopPeriodic();
        virtual void CoreDisabledInit();
        virtual void CoreDisabledPeriodic();

        virtual void CoreShuffleboardInit();
        virtual void CoreShuffleboardPeriodic();
        virtual void CoreShuffleboardUpdate();

        std::string name_;
        bool enabled_;
        bool shuffleboard_;
};