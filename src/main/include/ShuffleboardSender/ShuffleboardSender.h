#pragma once

#include <vector>
#include <string>
#include <map>

#include <frc/shuffleboard/Shuffleboard.h>

#include <units/voltage.h>

#include "ShuffleboardItem.h"

/**
 * Class to send many variables to Shuffleboard and edit them
*/
class ShuffleboardSender{
    public:
        /**
         * Creates a tab with name
        */
        ShuffleboardSender(std::string name, bool enable = true);

        /**
         * Pair a value on shuffleboard to the code
        */
        template <typename T> void add(ShuffleboardItem<T>* item){
            items_.push_back(item);
        }

        /**
         * Adds an item to this sender via a pointer
         * 
         * Does not add current element if one already has the same name
         * 
         * @param edit if the value will update in the code on the update call
        */
        template <typename T> void add(std::string name, T* o, bool edit = false){
            for(ShuffleboardItemInterface* item : items_){
                if(item->getName() == name){
                    return;
                }
            }
            items_.push_back(new ShuffleboardItem({name, tab_, edit}, o));
        }
        
        /**
         * Adds an item with the position and size of order {width, height, x, y}
         * Coordinates start at 0, 0
         * 
         * Does not add current element if one already has the same name
         * 
         * @param edit if the value will update in the code on the update call
        */
        template <typename T> void add(std::string name, T* o, ShuffleboardItemInterface::ShuffleboardPose pose, bool edit = false){
            for(ShuffleboardItemInterface* item : items_){
                if(item->getName() == name){
                    return;
                }
            }
            items_.push_back(new ShuffleboardItem({name, tab_, edit, pose}, o));
        }
        
        /**
         * Adds a button that executes some function
        */
        void addButton(std::string name, std::function<void()> callback);
        void addButton(std::string name, std::function<void()> callback, ShuffleboardItemInterface::ShuffleboardPose pose);

        /**
         * Adds a toggle button
        */
        void addToggleButton(std::string name, std::function<void()> callbackTrue, std::function<void()> callbackFalse, bool startVal);
        void addToggleButton(std::string name, std::function<void()> callbackTrue, std::function<void()> callbackFalse, bool startVal, ShuffleboardItemInterface::ShuffleboardPose pose);

        /**
         * Updates variables by reading and configuring, and then sending the data
         * 
         * edit boolean enable editing
        */
        void update(bool edit);

        /**
         * Enable/Disable the shuffleboard, deleting values or creating them
        */
        void enable();
        void disable();

        bool isEnabled();
        
        /**
         * Normal Shuffleboard behavior
        */
        void PutNumber(std::string name, double val);
        void PutNumber(std::string name, double val, ShuffleboardItemInterface::ShuffleboardPose pose);
        void PutBoolean(std::string name, bool val);
        void PutBoolean(std::string name, bool val, ShuffleboardItemInterface::ShuffleboardPose pose);
        void PutInteger(std::string name, int val);
        void PutInteger(std::string name, int val, ShuffleboardItemInterface::ShuffleboardPose pose);
        void PutString(std::string name, std::string val);
        void PutString(std::string name, std::string val, ShuffleboardItemInterface::ShuffleboardPose pose);

        double GetNumber(std::string name, double defaultVal);
        bool GetBoolean(std::string name, bool defaultVal);
        int GetInteger(std::string name, int defaultVal);
        std::string GetString(std::string name, std::string defaultVal);


    private:
        std::string name_;
        
        bool enabled_;
        
        frc::ShuffleboardTab* tab_;
        std::vector<ShuffleboardItemInterface*> items_;

        std::map<std::string, nt::GenericEntry*> keyMap_;
};
