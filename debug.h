//debug.h
#pragma once
#include <Arduino.h>
//Debug macros, use when you can but pls no touchy
#define DEBUG_LOG(msg) Serial.print(msg)
#define DEBUG_FILE_INFO() DEBUG_LOG( __FILE__ ":")
#define DEBUG_LINE_INFO() DEBUG_LOG( __LINE__)
#define DEBUG_VAR(var) \
DEBUG_FILE_INFO(); \
DEBUG_LINE_INFO(); \
DEBUG_LOG(": " #var " = " ); \
DEBUG_LOG(var); \
DEBUG_LOG(ENDL);