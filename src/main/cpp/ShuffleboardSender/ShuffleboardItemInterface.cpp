#include "ShuffleboardSender/ShuffleboardItemInterface.h"

ShuffleboardItemInterface::ShuffleboardItemInterface(ItemData data):
    data_(data)
{
    
}

std::string ShuffleboardItemInterface::getName(){
    return data_.name;
}