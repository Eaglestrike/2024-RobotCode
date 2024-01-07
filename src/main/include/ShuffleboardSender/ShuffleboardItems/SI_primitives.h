#pragma once

#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<double> : public BaseShuffleboardItem{
    public:
        ShuffleboardItem(ItemData data, double* value);
        bool itemHasChanged() override;
        void enable(frc::ShuffleboardTab* tab) override;
        void disable() override;
    private:
        void send() override;
        void edit() override;
        double prevVal_;
        double* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<bool>: public BaseShuffleboardItem{
    public:
        ShuffleboardItem(ItemData data, bool* value);
        bool itemHasChanged() override;
        void enable(frc::ShuffleboardTab* tab) override;
        void disable() override;
    private:
        void send() override;
        void edit() override;
        bool prevVal_;
        bool* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<int>: public BaseShuffleboardItem{
    public:
        ShuffleboardItem(ItemData data, int* value);
        bool itemHasChanged() override;
        void enable(frc::ShuffleboardTab* tab) override;
        void disable() override;
    private:
        void send() override;
        void edit() override;
        int prevVal_;
        int* value_;
        nt::GenericEntry* entry_;
};