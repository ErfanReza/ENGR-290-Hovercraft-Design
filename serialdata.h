//serialdata.h
#pragma once
//#define DEBUG
#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include "debug.h"
#define SERIAL_T float
#define MAX_SERIAL_BUFFER_SIZE 100
#define PACKET_START 's'
#define PACKET_SEND_DEL ','
#define PACKET_RECV_DEL ' '
class SerialData
{
private:
static char serial_buffer[MAX_SERIAL_BUFFER_SIZE]; // Buffer for allocated data
public:
//Static functions for data transfer
void sendData(SERIAL_T * data, size_t len); // Sends data in the ###, ###, ... \n\r format
void parseData(char * data, size_t len); // Takes raw serial data from buffer and parses it with the ###, ###, ... \n\r. Allocates results in data
static size_t readIntoBuffer(); // Receives data from the serial buffer and stores it into serial_buffer. Returns the amount of data recevied
static char* getBuffer() {return serial_buffer;} // Returns pointer to serial_buffer
void parseData(); // Receive data from serial port, saves into the buffer, and parses it
public:
//Object memebers
SerialData(); // Constructor.
SERIAL_T at(const size_t &i) {return this->data[i];} // Gets data member at i index
size_t lenght() {return this->len;} // Gets the lenght of data
private:
SERIAL_T data[10]; // Stores parsed data
size_t len = 0; // Stores data size
};
//util.h
#ifndef UTILS_H_
#define UTILS_H_
#define CAP_AT_NUM(val, num) (val < 0.0 ? num : val)
#define MAP_AT_CAP(REF, VAL, CAP) (REF * ((VAL > CAP ? CAP : VAL)/CAP))
#define ABS(x,y) (x < y ? y-x :x-y)
#endif