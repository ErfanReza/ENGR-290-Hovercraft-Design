//serialdata.cpp
#include "serialdata.h"
char SerialData::serial_buffer[MAX_SERIAL_BUFFER_SIZE] = {0};
SerialData::SerialData(): len(0) {}
void SerialData::sendData(SERIAL_T * data, size_t len)
{
while(!Serial)
delay(1);
for(size_t i = 0; i < len; i++)
{
Serial.print(data[i]);
Serial.write(PACKET_SEND_DEL);
}
Serial.write("\n\r");
}
size_t SerialData::readIntoBuffer()
{
while(!Serial)
delay(1);
size_t bytes_read = 0;
char c = 'a';
// Wait for s to arrive. 's' indicates the
// start of a packet
while(c != PACKET_START)
c = Serial.read();
serial_buffer[bytes_read++] = c;
// Read until either buffer is filled up or packet is over
while(c != '\r' && c != '\n' && bytes_read < MAX_SERIAL_BUFFER_SIZE)
{
while(Serial.available() > 0)
c = serial_buffer[bytes_read++] = Serial.read();
}
#ifdef DEBUG
DEBUG_VAR(bytes_read)
#endif
return bytes_read;
}
void SerialData::parseData(char * data, size_t lenght)
{
size_t i = 0;
size_t len = 1;
size_t start = 0;
#ifdef DEBUG
DEBUG_VAR(lenght)
#endif
// Read the amount of numbers received
for(; i < lenght; i++)
{
if(data[i] != PACKET_START && data[i] != PACKET_RECV_DEL)
{
start = i;
break;
}
}
for(; i < lenght && len < 10; i++)
{
if(data[i] == PACKET_RECV_DEL) len++;
else if (data[i] == '\n' || data[i] == '\r') break;
}
#ifdef DEBUG
DEBUG_VAR(len)
#endif
// For every number found, store them into
// this->data
this->len = len;
size_t index = 0;
float buf = 0.0;
float dec_count = 1.0;
bool is_decimal = false;
bool is_negative = false;
for(i = start; i < lenght && index < this->len; i++)
{
int c = data[i];
#ifdef DEBUG
DEBUG_VAR(char(c))
#endif
switch(c)
{
//For negative numbers
case '-':
is_negative = true;
break;
//For decimal numbers
case '.':
is_decimal = true;
break;
//For the numbers
case '0':
case '1':
case '2':
case '3':
case '4':
case '5':
case '6':
case '7':
case '8':
case '9':
c -= 48;
if(is_decimal)
buf += ((float)c)/(pow(10.0,dec_count++));
else
buf = buf * 10.0 + (float)c;
#ifdef DEBUG
DEBUG_VAR(buf)
#endif
break;
//For the endings
case ' ':
case '\n':
case '\r':
#ifdef DEBUG
this->data[index] = (SERIAL_T)(buf*(is_negative ? -1.0 : 1.0));
DEBUG_VAR(this->data[index])
index++;
#else
this->data[index++] = (SERIAL_T)(buf*(is_negative ? -1.0 : 1.0));
#endif
buf = 0.0;
dec_count = 1.0;
is_decimal = false;
is_negative = false;
break;
default:
continue;
}
}
}
void SerialData::parseData()
{
size_t s = this->readIntoBuffer();
char* b = this->getBuffer();
this->parseData(b,s);
}

