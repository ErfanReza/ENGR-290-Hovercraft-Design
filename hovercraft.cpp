//hovercraft.cpp
#include "hovercraft.h"
Hovercraft::Hovercraft(SerialData* serialdata): serialdata(serialdata) {}
void Hovercraft::sendUpdate()
{
if(this->serialdata == NULL)
return;
// Lift Fan
this->internalBuffer[0] = this->lift_fan_state;
// Right Fan
this->internalBuffer[1] = this->right_fan_state;
this->internalBuffer[2] = this->right_fan_thrust;
this->internalBuffer[3] = this->right_servo;
// Left Fan
this->internalBuffer[4] = this->left_fan_state;
this->internalBuffer[5] = this->left_fan_thrust;
this->internalBuffer[6] = this->left_servo;
this->serialdata->sendData(this->internalBuffer, 7);
}
void Hovercraft::receiveUpdate()
{
if(this->serialdata == NULL)
return;
this->serialdata->parseData();
if(this->serialdata->lenght() < this->input_count)
return;
this->sim_time = this->serialdata->at(0);
this->left_sensor = this->serialdata->at(1);
this->right_sensor = this->serialdata->at(2);
}