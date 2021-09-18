//controller.ino
#include <stdint.h>
#include "utils.h"
#include "debug.h"
#include "serialdata.h"
#include "hovercraft.h"
#include <string.h>
#include <stdio.h>
// ------------------ OPTIONS -------------------------
#define BAUD_RATE 115200
// Comment if you'd like to disable camera support (will ignore proximity sensors)
#define ENABLE_VISION
// Uncomment if you'd like to disable the sensors (used for debugging)
//#define DISABLE_SENSORS
// ------------------ CONSTANTS -------------------------
#define THRUST_FAN_SPEED 100.0
#define WALL_STOP_DELAY 750
#ifdef DISABLE_SENSORS
#define OPTIMAL_SIDE_DISTANCE 2.0
#define SWITCH_LANE_DELAY 3500
#else
#define OPTIMAL_SIDE_DISTANCE 10.0
#define SWITCH_LANE_DELAY 4350
#endif
#define LEFT true
#define RIGHT false
#define UP_ANGLE 0.0
#define RIGHT_ANGLE -90.0
#define LEFT_ANGLE 90.0
#define ON true
#define OFF false
#define LANES_COUNT 4
#define LANE_1_DURATION 4000
#define LANE_2_DURATION 4500
#define LANE_3_DURATION 4200
#define LANE_4_DURATION 4800

#ifdef ENABLE_VISION
#define SERIAL_DELAY 90 //~2*8*1000000/BAUDRATE
#define IMG_SENSOR_YDIM 32
uint32_t img[IMG_SENSOR_YDIM];
#endif
// ------------------ VARIABLES -------------------------------
bool current_direction = LEFT;
float sensor_reading = 0.0;
float turn_angle = 0.0;
int lane_count = 0;
SerialData serialData;
Hovercraft hovercraft(&serialData);
// -------------------------- SETUP FUNCTION --------------------------
void setup()
{
Serial.begin(BAUD_RATE);
current_direction = LEFT;
#if ! defined(DISABLE_SENSORS) && ! defined(ENABLE_VISION)
// Do this to:
// - Get the initial time for the simulation
// - Make the Arduino wait for the simulator
receive_update();
float ls = hovercraft.getLeftSensorDistance();
if(ls < hovercraft.getRightSensorDistance())
current_direction = LEFT;
else
current_direction = RIGHT;
#endif
#ifdef ENABLE_VISION
Serial.setTimeout(1);
#endif
switch(current_direction)
{
case LEFT:
set_servos(LEFT_ANGLE);
break;
case RIGHT:
set_servos(RIGHT_ANGLE);

break;
}
//turn on fans
set_all_fans_state(ON);
set_thrust_fans_thrust(THRUST_FAN_SPEED);
send_update();
}
// -------------------------- LANE SWITCH --------------------------
void switch_lanes()
{
// Move up to the next lane
set_servos(UP_ANGLE);
send_update();
#ifdef ENABLE_VISION
while(checkDistance() > 0)
{
serialWait();
Serial.println(" ");
}
#else
delay(SWITCH_LANE_DELAY);
#endif
set_servos(turn_angle);
current_direction = !current_direction;
send_update();
lane_count++;
}

// -------------------------- MAIN LOOP --------------------------
void loop()
{
#ifdef ENABLE_VISION
// Wait for an update
serialWait();
#else
// Wait for an update
receive_update();
#endif
#ifndef ENABLE_VISION
#ifndef DISABLE_SENSORS
// -------------------------- WITH SENSORS --------------------------
// Depending on the direction, read the sensor and decide the turn angle
switch(current_direction)
{
case LEFT:
sensor_reading = hovercraft.getLeftSensorDistance();
turn_angle = RIGHT_ANGLE;
break;
case RIGHT:
sensor_reading = hovercraft.getRightSensorDistance();
turn_angle = LEFT_ANGLE;
break;
}
// If you are OPTIMAL_SIDE_DISTANCE cm away from a side wall...
if(sensor_reading > OPTIMAL_SIDE_DISTANCE and not (sensor_reading < 0))
{
delay(WALL_STOP_DELAY);
switch_lanes();
}
#else
// -------------------------- WITHOUT SENSORS --------------------------
// Depending on the direction, read the sensor and decide the turn angle
switch(current_direction)
{
case LEFT:
turn_angle = RIGHT_ANGLE;
break;
case RIGHT:
turn_angle = LEFT_ANGLE;
break;
}
unsigned int timings[LANES_COUNT] = {
LANE_1_DURATION,
LANE_2_DURATION,
LANE_3_DURATION,
LANE_4_DURATION
};
// If we are not on the last track, move in the current one direction
// for timings[lane_count]

delay(timings[lane_count]);
switch_lanes();
#endif
#else
switch(current_direction)
{
case LEFT:
turn_angle = RIGHT_ANGLE;
break;
case RIGHT:
turn_angle = LEFT_ANGLE;
break;
}
int dist = checkDistance();
if(dist > 1 && dist < 10)
switch_lanes();
#endif
// If the last lane has beem reached, stop
if(lane_count >= LANES_COUNT)
{
set_all_fans_state(OFF);
send_update();
while(1)
delay(1000);
}
// Sending an update should guarantee that we'll
// receive one back
send_update();
}
//--------------------- FAN HANDLERS ---------------------
// Set thrust fans state
static inline void set_thrust_fans_state(bool s)
{
hovercraft.right_fan_state = s;
hovercraft.left_fan_state = s;
}
// Set thrust fans thrust
static inline void set_thrust_fans_thrust(float t)
{
hovercraft.right_fan_thrust = t;
hovercraft.left_fan_thrust = t;

}
// Set lift fans state
static inline void set_lift_fan_state(bool s)
{
hovercraft.lift_fan_state = s;
}
// Set all fans state
static inline void set_all_fans_state(bool s)
{
set_thrust_fans_state(s);
set_lift_fan_state(s);
}
// ----------------------- SERVO HELPERS -------------------------------
// Set all servos state
static inline void set_servos(float a)
{
hovercraft.right_servo = a;
hovercraft.left_servo = a;
}
// ---------------------- DATA HANDLERS ---------------------------------
// Sends current hovercraft settings to the simulator
static inline void send_update()
{
hovercraft.sendUpdate();
}
// Receives updates from the simulator (will block until
// updates are received)
static inline void receive_update()
{
hovercraft.receiveUpdate();
}
// ---------------------- CAMERA HANDLERS ---------------------------------
#ifdef ENABLE_VISION
void serialWait()
{
float simTime = Serial.parseFloat();
int result = Serial.parseInt();
// digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
while(Serial.peek() != 's')
{
if (Serial.peek() == 'i')
{ //Read image data coming on the Serial bus
char t = Serial.read(); //discard the i
delayMicroseconds(6*SERIAL_DELAY);
for (int j = IMG_SENSOR_YDIM-1; j >= 0; j--)
{
char longarr[8];
for (int i = 0; i < 8; i++)
{
longarr[i] = Serial.read();
if (Serial.available()< 4)
{
delayMicroseconds(SERIAL_DELAY);
}
}
unsigned long hexval = strtoul((const char*)longarr, 0, 16);
img[j] = hexval;
}
if (Serial.peek() == '@')
{
char t = Serial.read(); //discard the @
//Print out the image, to check if it was received properly
Serial.write(0x23);Serial.write(0x23);Serial.write("\r\n");
Serial.write("printing image...");
Serial.write("\r\n");
for (int b = 0; b < IMG_SENSOR_YDIM; b++)
{
for (int c = 0; c < 32; c++) //for (int c = 31; c >= 0; c--)
{
Serial.print(bitRead(img[b],c));
Serial.write(" ");
}
Serial.write("\r\n");
}
Serial.write(0x40);Serial.write(0x40);Serial.write("\r\n");
}
}
if (Serial.peek() == 's')
{
//char t = Serial.read(); //discard the s
digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
return;// break out of the while loop and continue to the parsing steps.
}
if (Serial.available() > 2)
{
char t = Serial.read();
}
}
// digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
}
void serialEmpty()
{
while(Serial.available())
{
char t = Serial.read();
}
}
short readImage(short imgX, short imgY)
{
return bitRead(img[imgY],imgX);
}
// Made so it will start scanning from line
// two to ignore line at the bottom
int checkDistance()
{
for (int i = 2; i < 29; i++)
{
if (readImage(14,i) == 1)
{
return i;
}
}
return -1;
}
#endif