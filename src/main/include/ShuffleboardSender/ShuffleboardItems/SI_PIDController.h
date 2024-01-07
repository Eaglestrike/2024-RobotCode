#pragma once

#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<frc::PIDController>: public BaseShuffleboardItem{
    public:
        ShuffleboardItem(ItemData data, frc::PIDController* value);
        void enable(frc::ShuffleboardTab* tab) override;
        void disable() override;
        
    private:
        void send() override;
        void edit() override;
        frc::PIDController* value_;
        nt::GenericEntry* entry_[3]; //[P, I, D]
};