#include "ShuffleboardSender/ShuffleboardButton.h"

ShuffleboardButton::ShuffleboardButton(ItemData data, std::function<void()> callback):
    ShuffleboardItemInterface(data),
    callback_(callback)
{
    if(data_.tab){
        entry_ = ShuffleboardHelper::createItem(data_, false, frc::BuiltInWidgets::kToggleButton);
    }
}

void ShuffleboardButton::update(bool update){
    if(update){
        if(entry_->GetBoolean(false)){
            callback_();
            entry_->SetBoolean(false);
        }
    }
}

void ShuffleboardButton::enable(frc::ShuffleboardTab* tab){
    if(!entry_->Exists()){
        data_.tab = tab;
        entry_ = ShuffleboardHelper::createItem(data_, false, frc::BuiltInWidgets::kToggleButton);
    }
};
void ShuffleboardButton::disable(){
    entry_->Unpublish();
};