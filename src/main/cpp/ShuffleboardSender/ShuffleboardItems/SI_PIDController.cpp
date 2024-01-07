#include "ShuffleboardSender/ShuffleboardItems/SI_PIDController.h"

ShuffleboardItem<frc::PIDController>::ShuffleboardItem(ItemData data, frc::PIDController* value):
    BaseShuffleboardItem(data)
{
    value_ = value;

    if(data_.tab){
        frc::ShuffleboardLayout* pidLayout = ShuffleboardHelper::createList(data);
        entry_[0] = pidLayout->Add("P", value->GetP()).GetEntry();   
        entry_[1] = pidLayout->Add("I", value->GetI()).GetEntry();
        entry_[2] = pidLayout->Add("D", value->GetD()).GetEntry(); 
    }
};

void ShuffleboardItem<frc::PIDController>::send(){
    entry_[0]->SetDouble(value_->GetP());
    entry_[1]->SetDouble(value_->GetI());
    entry_[2]->SetDouble(value_->GetD());
}

void ShuffleboardItem<frc::PIDController>::edit(){
    value_->SetP(entry_[0]->GetDouble(value_->GetP()));
    value_->SetI(entry_[1]->GetDouble(value_->GetI()));
    value_->SetD(entry_[2]->GetDouble(value_->GetD()));
}

void ShuffleboardItem<frc::PIDController>::enable(frc::ShuffleboardTab* tab){
    for(auto &component :tab->GetComponents()){
        if(component->GetTitle() == data_.name){
            return;
        }
    }
    data_.tab = tab;
    frc::ShuffleboardLayout* pidLayout = ShuffleboardHelper::createList(data_);
    entry_[0] = pidLayout->Add("P", value_->GetP()).GetEntry();   
    entry_[1] = pidLayout->Add("I", value_->GetI()).GetEntry();
    entry_[2] = pidLayout->Add("D", value_->GetD()).GetEntry(); 
};

void ShuffleboardItem<frc::PIDController>::disable(){
    // entry_[0]->Unpublish();
    // entry_[1]->Unpublish();
    // entry_[2]->Unpublish();
};