//hovercraft.h
#pragma once
#include <Arduino.h>
#include "serialdata.h"
#include "utils.h"
class Hovercraft
{
private:
const unsigned int input_count = 3;

public:
Hovercraft(SerialData* serialdata);
void sendUpdate();
void receiveUpdate();
float getTime()
{ return this->sim_time; }
float getRightSensorDistance()
{ return this->right_sensor; }
float getLeftSensorDistance()
{ return this->left_sensor; }
public:
bool lift_fan_state = false;
bool right_fan_state = false;
bool left_fan_state = false;
float right_fan_thrust = 0.0;
float right_servo = 0.0;
float left_fan_thrust = 0.0;
float left_servo = 0.0;
private:
SerialData* serialdata;
SERIAL_T internalBuffer[8];
float sim_time = 0.0;
float right_sensor = 0.0;
float left_sensor = 0.0;
};