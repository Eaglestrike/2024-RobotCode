#pragma once

#include <string>

#include <frc/shuffleboard/Shuffleboard.h>

class ShuffleboardItemInterface{
    public:
        /**
         * Pose Struct:
         * {width, height, x, y}
         * Coordinates start at 0, 0
         * -1 coordinates is default placement
        */
        struct ShuffleboardPose{
            int width = 1;
            int height = 1;
            int positionX = -1;
            int positionY = -1;
        };

        struct ItemData{
            std::string name;
            frc::ShuffleboardTab* tab;
            bool edit = false;
            ShuffleboardPose pose = {1,1,-1,-1};
        };
        
        ShuffleboardItemInterface(ItemData data);

        virtual void update(bool edit) = 0;

        virtual void enable(frc::ShuffleboardTab* tab) = 0;
        virtual void disable() = 0;

        std::string getName();

    protected:
        ItemData data_;
};