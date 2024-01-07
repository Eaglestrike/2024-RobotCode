#include "ShuffleboardSender/ShuffleboardToggleButton.h"

/**
 * @param callbackTrue call when set to true
 * @param callbackFalse call when set to false
*/
ShuffleboardToggleButton::ShuffleboardToggleButton(ItemData data, std::function<void()> callbackTrue, std::function<void()> callbackFalse, bool startVal):
    ShuffleboardItemInterface(data),
    callbackTrue_(callbackTrue),
    callbackFalse_(callbackFalse),
    prevVal_(startVal)
{
    if(data_.tab){
        entry_ = ShuffleboardHelper::createItem(data_, startVal, frc::BuiltInWidgets::kToggleSwitch);
    }
}

/**
 * Periodic call, executes code
 * 
 * @param update, adds ability to change code
*/
void ShuffleboardToggleButton::update(bool update){
    if(update){
        bool newVal = entry_->GetBoolean(prevVal_);
        if(newVal != prevVal_){
            if(newVal){
                callbackTrue_();
            }
            else{
                callbackFalse_();
            }
        }
        prevVal_ = newVal;
    }
}

void ShuffleboardToggleButton::enable(frc::ShuffleboardTab* tab){
    if(!entry_->Exists()){
        data_.tab = tab;
        entry_ = ShuffleboardHelper::createItem(data_, prevVal_, frc::BuiltInWidgets::kToggleSwitch);
    }
};

void ShuffleboardToggleButton::disable(){
    entry_->Unpublish();
};