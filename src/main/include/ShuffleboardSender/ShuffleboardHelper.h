#pragma once

#include <frc/shuffleboard/Shuffleboard.h>

#include "ShuffleboardItemInterface.h"

namespace ShuffleboardHelper{
    /**
     * Creates an item on a shuffleboard tab, with a given position/size and type
    */
    template <typename T> nt::GenericEntry* createItem(ShuffleboardItemInterface::ItemData data, T value, frc::BuiltInWidgets type = frc::BuiltInWidgets::kTextView){
        // for(auto &component : data.tab->GetComponents()){
        //     if(component.get()->GetTitle() == data.name){
        //         return component->get()->GetEntry()
        //     }
        // }
        if((data.pose.positionX >= 0) && (data.pose.positionY >= 0)){
            return data.tab->Add(data.name, value)
                                .WithWidget(type)
                                .WithSize(data.pose.width, data.pose.height)
                                .WithPosition(data.pose.positionX, data.pose.positionY)
                                .GetEntry();
        } 
        else{
            return data.tab->Add(data.name, value)
                                .WithWidget(type)
                                .WithSize(data.pose.width, data.pose.height)
                                .GetEntry();
        }
    }

    /**
     * Creates a list with the given position/size
    */
    inline frc::ShuffleboardLayout* createList(ShuffleboardItemInterface::ItemData data){
        // for(auto &component : data.tab->GetComponents()){
        //     if(component.get()->GetTitle() == data.name){
        //         return component->get();
        //     }
        // }
        if((data.pose.positionX >= 0) && (data.pose.positionY >= 0)){
            return &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                .WithSize(data.pose.width, data.pose.height)
                                .WithPosition(data.pose.positionX, data.pose.positionY)
                                .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
        }
        else{
            return &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                .WithSize(data.pose.width, data.pose.height)
                                .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
        }
    }
}