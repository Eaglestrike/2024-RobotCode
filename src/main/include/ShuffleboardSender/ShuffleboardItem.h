#pragma once

#include <vector>

#include <iostream>

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/PIDCommand.h>
#include <units/voltage.h>

#include "ShuffleboardItemInterface.h"
#include "ShuffleboardHelper.h"

/**
 * Class to send objects to shuffleboard
 * 
 * Default only works for frc::units
 * 
 * Included Items:
 * double
 * bool
 * int
 * frc::PIDController
*/

class BaseShuffleboardItem : public ShuffleboardItemInterface{
    public:
        BaseShuffleboardItem(ItemData data):ShuffleboardItemInterface(data){};

        void update(bool canEdit) override{
            if(itemHasChanged()){
                send();
            }
            if(data_.edit && canEdit){
                edit();
            }
        }

        virtual bool itemHasChanged(){
            return true;
        };

    private:
        virtual void send() = 0;
        virtual void edit() = 0;
};

template <typename T>
class ShuffleboardItem : public BaseShuffleboardItem{
    public:
        ShuffleboardItem(ItemData data, T* value):
            ShuffleboardItemInterface(data)
        {
            value_ = value;
            entry_ = ShuffleboardHelper::createItem(data, value->value());
        };

        bool itemHasChanged() override{
            double newVal = value_->value();
            bool hasChanged = (prevVal_ != newVal);
            prevVal_ = newVal;
            return hasChanged;
        } 

        void enable(){
            if(!entry_->Exists()){
                entry_ = ShuffleboardHelper::createItem(data_, value_->value());
            }
        };
        void disable(){
            entry_->Unpublish();
        };

    private:
        void send() override{
            entry_->SetDouble(value_->value());
        }
        void edit() override{
            *value_ = T{entry_->GetDouble(value_->value())};
        }

        double prevVal_;
        T* value_;
        nt::GenericEntry* entry_;
};

#include "ShuffleboardItems/SI_primitives.h"
#include "ShuffleboardItems/SI_PIDController.h"
#include "ShuffleboardItems/SI_units.hpp"