#include "ShuffleboardSender/ShuffleboardSender.h"

#include "ShuffleboardSender/ShuffleboardButton.h"
#include "ShuffleboardSender/ShuffleboardToggleButton.h"

ShuffleboardSender::ShuffleboardSender(std::string name, bool enabled):
    name_(name),
    enabled_(false)
{
    if(enabled){
        enable();
    }
}

void ShuffleboardSender::addButton(std::string name, std::function<void()> callback){
    for(ShuffleboardItemInterface* item : items_){
        if(item->getName() == name){
            return;
        }
    }
    items_.push_back(new ShuffleboardButton({name, tab_, true}, callback));
}

void ShuffleboardSender::addButton(std::string name, std::function<void()> callback, ShuffleboardItemInterface::ShuffleboardPose pose){
    for(ShuffleboardItemInterface* item : items_){
        if(item->getName() == name){
            return;
        }
    }
    items_.push_back(new ShuffleboardButton({name, tab_, true, pose}, callback));
}

void ShuffleboardSender::addToggleButton(std::string name, std::function<void()> callbackTrue, std::function<void()> callbackFalse, bool startVal){
    for(ShuffleboardItemInterface* item : items_){
        if(item->getName() == name){
            return;
        }
    }
    items_.push_back(new ShuffleboardToggleButton({name, tab_, true}, callbackTrue, callbackFalse, startVal));
}

void ShuffleboardSender::addToggleButton(std::string name, std::function<void()> callbackTrue, std::function<void()> callbackFalse, bool startVal, ShuffleboardItemInterface::ShuffleboardPose pose){
    for(ShuffleboardItemInterface* item : items_){
        if(item->getName() == name){
            return;
        }
    }
    items_.push_back(new ShuffleboardToggleButton({name, tab_, true, pose}, callbackTrue, callbackFalse, startVal));
}

void ShuffleboardSender::update(bool edit){
    if(enabled_){
        for(ShuffleboardItemInterface* item : items_){
            item->update(edit);
        }
    }
};

void ShuffleboardSender::enable(){
    enabled_ = true;
    tab_ = &frc::Shuffleboard::GetTab(name_);
    for(ShuffleboardItemInterface* item : items_){
        item->enable(tab_);
    }
}

void ShuffleboardSender::disable(){
    enabled_ = false;
    for(ShuffleboardItemInterface* item : items_){
        item->disable();
    }
    // for(std::pair<const std::string, nt::GenericEntry*> pair : keyMap_){
    //     pair.second->Unpublish();
    // }
    // keyMap_.clear();
}

bool ShuffleboardSender::isEnabled(){
    return enabled_;
}

void ShuffleboardSender::PutNumber(std::string name, double val){
    if(keyMap_.contains(name)){
        keyMap_[name]->SetDouble(val);
        return;
    }
    keyMap_[name] = ShuffleboardHelper::createItem({.name = name, .tab = tab_}, val);
}

void ShuffleboardSender::PutNumber(std::string name, double val, ShuffleboardItemInterface::ShuffleboardPose pose){
    if(keyMap_.contains(name)){
        keyMap_[name]->SetDouble(val);
        return;
    }
    keyMap_[name] = ShuffleboardHelper::createItem({.name = name, .tab = tab_, .pose = pose}, val);
}

void ShuffleboardSender::PutBoolean(std::string name, bool val){
    if(keyMap_.contains(name)){
        keyMap_[name]->SetBoolean(val);
        return;
    }
    keyMap_[name] = ShuffleboardHelper::createItem({.name = name, .tab = tab_}, val);
}

void ShuffleboardSender::PutBoolean(std::string name, bool val, ShuffleboardItemInterface::ShuffleboardPose pose){
    if(keyMap_.contains(name)){
        keyMap_[name]->SetBoolean(val);
        return;
    }
    keyMap_[name] = ShuffleboardHelper::createItem({.name = name, .tab = tab_, .pose = pose}, val);
}

void ShuffleboardSender::PutInteger(std::string name, int val){
    if(keyMap_.contains(name)){
        keyMap_[name]->SetInteger(val);
        return;
    }
    keyMap_[name] = ShuffleboardHelper::createItem({.name = name, .tab = tab_}, val);
}

void ShuffleboardSender::PutInteger(std::string name, int val, ShuffleboardItemInterface::ShuffleboardPose pose){
    if(keyMap_.contains(name)){
        keyMap_[name]->SetInteger(val);
        return;
    }
    keyMap_[name] = ShuffleboardHelper::createItem({.name = name, .tab = tab_, .pose = pose}, val);
}

void ShuffleboardSender::PutString(std::string name, std::string val){
    if(keyMap_.contains(name)){
        keyMap_[name]->SetString(val);
        return;
    }
    keyMap_[name] = ShuffleboardHelper::createItem({.name = name, .tab = tab_}, val);
}

void ShuffleboardSender::PutString(std::string name, std::string val, ShuffleboardItemInterface::ShuffleboardPose pose){
    if(keyMap_.contains(name)){
        keyMap_[name]->SetString(val);
        return;
    }
    keyMap_[name] = ShuffleboardHelper::createItem({.name = name, .tab = tab_, .pose = pose}, val);
}

double ShuffleboardSender::GetNumber(std::string name, double defaultVal){
    if(keyMap_.contains(name)){
        return keyMap_[name]->GetDouble(defaultVal);
    }
    return defaultVal;
}
bool ShuffleboardSender::GetBoolean(std::string name, bool defaultVal){
    if(keyMap_.contains(name)){
        return keyMap_[name]->GetBoolean(defaultVal);
    }
    return defaultVal;
}
int ShuffleboardSender::GetInteger(std::string name, int defaultVal){
    if(keyMap_.contains(name)){
        return keyMap_[name]->GetInteger(defaultVal);
    }
    return defaultVal;
}
std::string ShuffleboardSender::GetString(std::string name, std::string defaultVal){
    if(keyMap_.contains(name)){
        return keyMap_[name]->GetString(defaultVal);
    }
    return defaultVal;
}

