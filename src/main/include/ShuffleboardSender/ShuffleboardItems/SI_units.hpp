#pragma once

#include "units/voltage.h"

#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<units::volt_t>: public BaseShuffleboardItem{
    public:
        ShuffleboardItem(ItemData data, units::volt_t* value):
            BaseShuffleboardItem(data)
        {
            value_ = value;

            if(data_.tab){
                entry_ = ShuffleboardHelper::createItem(data, value_->value());
            }
        }

        bool itemHasChanged() override{
            double newVal = value_->value();
            bool hasChanged = (prevVal_ != newVal);
            prevVal_ = newVal;
            return hasChanged;
        }

        void enable(frc::ShuffleboardTab* tab) override{
            for(auto &component :tab->GetComponents()){
                if(component->GetTitle() == data_.name){
                    return;
                }
            }
            data_.tab = tab;
            entry_ = ShuffleboardHelper::createItem(data_, value_->value());
        }

        void disable() override{

        }
        
    private:
        void send() override{
            entry_->SetDouble(value_->value());
        }

        void edit() override{
            value_ = new units::volt_t{entry_->GetDouble(value_->value())};
        }

        bool prevVal_;
        units::volt_t* value_;
        nt::GenericEntry* entry_;
};