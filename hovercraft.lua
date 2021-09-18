local USE_ARDUINO = true
local SERIAL_PORT_NAME = "COM3"
local SERIAL_BAUDRATE = 115200
local SENSOR_PRESCISION = 0.001
local CONTACT_PATCH_NAME = "hv"
local LIFT_FAN_NAME = "liftFan"
local RIGTH_FAN_NAME = "ThrustFan#1"
local RIGHT_SERVO_NAME = "SmallServo#1"
local RIGHT_SENSOR_NAME = "MB1030#0"
local LEFT_FAN_NAME = "ThrustFan1"
local LEFT_SERVO_NAME = "SmallServo"
local LEFT_SENSOR_NAME = "MB1030#1"
local DEBUG = false
function coroutineMain()
print(_VERSION)
-- simulator settings
sim.setThreadSwitchTiming(10)
sim.setIntegerSignal('Consoles',0)
sim.setIntegerSignal('lift',0)
keyboard_ctrl = 0
init = true
serial = nil
--Open serial port communication
print(SERIAL_PORT_NAME)
serial = sim.serialOpen(SERIAL_PORT_NAME,SERIAL_BAUDRATE)
-- Print serial info
print(serial)
if serial == -1 then
error("Serial port error. Check port name/path and USB connection to Arduino board")
end
contact_patch_handle = sim.getObjectHandle(CONTACT_PATCH_NAME)
lift_fan_handle = sim.getObjectHandle(LIFT_FAN_NAME)
rigth_servo_handle = sim.getObjectHandle(RIGHT_SERVO_NAME)
rigth_fan_handle = sim.getObjectHandle(RIGTH_FAN_NAME)
rigth_sensor_handle = sim.getObjectHandle(RIGHT_SENSOR_NAME)
left_servo_handle = sim.getObjectHandle(LEFT_SERVO_NAME)
left_fan_handle = sim.getObjectHandle(LEFT_FAN_NAME)
left_sensor_handle = sim.getObjectHandle(LEFT_SENSOR_NAME)
-- initialize sensors
imu_init()
local arduinoData = {}
arduinoData.lift_fan_state = 0
arduinoData.rigth_fan_state = 0
arduinoData.rigth_fan_thrust = 0
arduinoData.rigth_servo = 0
arduinoData.left_fan_state = 0
arduinoData.left_fan_thrust = 0
arduinoData.left_servo = 0
rxData = nil
-- Main simulation loop
while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop do
-- Get sensor data in centimeters
detected, rigth_sensor_reading = read_sensor(rigth_sensor_handle)
rigth_sensor_reading = 100 * rigth_sensor_reading
detected, left_sensor_reading = read_sensor(left_sensor_handle)
left_sensor_reading = 100 * left_sensor_reading
if DEBUG then
print("rigth_sensor_reading: " .. rigth_sensor_reading)
print("left_sensor_reading: " .. left_sensor_reading)
end
-- Get data from arduino
if serial ~= -1 and init == false then
rxData = receive_data()
print(rxData)
end
-- If data is valid, react accordingly
-- Arduino Receive Data Format
-- rxData[1] lift_fan_state
-- rxData[2] rigth_fan_state
-- rxData[3] rigth_fan_thrust
-- rxData[4] rigth_servo
-- rxData[5] left_fan_state
-- rxData[6] left_fan_thrust
-- rxData[7] left_servo
if rxData and table.getn(rxData)>=7 then
-- Get arduino data into arduinoData
-- Lift Fan
arduinoData.lift_fan_state = rxData[1]
-- Right fan
arduinoData.rigth_fan_state = rxData[2]
arduinoData.rigth_fan_thrust = rxData[3]
arduinoData.rigth_servo = angle_to_radiants(rxData[4])
-- Left fan
arduinoData.left_fan_state = rxData[5]
arduinoData.left_fan_thrust = rxData[6]
arduinoData.left_servo = angle_to_radiants(rxData[7])
-- Lift Fan Update
set_lift(lift_fan_handle, arduinoData.lift_fan_state, contact_patch_handle)
-- Right Fan Update
set_thrust(rigth_fan_handle, arduinoData.rigth_fan_state, arduinoData.rigth_fan_thrust/100)
set_servo(rigth_servo_handle, arduinoData.rigth_servo)
-- Left Fan Update
set_thrust(left_fan_handle, arduinoData.left_fan_state, arduinoData.left_fan_thrust/100)
set_servo(left_servo_handle, arduinoData.left_servo)
end
-- Update Arduino with sensor data
-- Arduino Send Data Format
-- txData[1] simTime
-- txData[2] left_sensor_reading
-- txData[3] rigth_sensor_reading
if serial ~= -1 then
sim_time = round(sim.getSimulationTime(), SENSOR_PRESCISION)
rigth_sensor_reading = round(rigth_sensor_reading, SENSOR_PRESCISION)
left_sensor_reading = round(left_sensor_reading, SENSOR_PRESCISION)
send_data(serial, {sim_time, left_sensor_reading, rigth_sensor_reading})
init = false
end
sim.switchThread() -- resume in next simulation step
end
end
function angle_to_radiants(angle)
return angle*3.14*2/180
end
function imu_init()
-- This function will initialize the IMU communication
-- Run only once in the initialization section
-- Usage
-- imu_init()
gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1)
accelSelfCommunicationTube=sim.tubeOpen(0,'accelerometerSelfData'..sim.getNameSuffix(nil),1)
end
function read_imu()
-- This function will read the IMU
-- it will return instantaneous values of angular speed (rad/s) and linear acceleration (m/s^2)
-- Returned values are 2 tables with 3 numbers each
-- {XangVel, YangVel, ZangVel}, {Xaccel, Yaccel, Zaccel}
-- Usage
-- angVel, linAccel = read_imu()
local angularSpeeds = {0,0,0}
local data1=sim.tubeRead(gyroCommunicationTube)
if (data1) then
angularSpeeds=sim.unpackFloatTable(data1)
angularSpeeds = round(angularSpeeds, sensorPrecision)
end
local accelSelf = {0,0,0}
local data2=sim.tubeRead(accelSelfCommunicationTube)
if (data2) then
accelSelf =sim.unpackFloatTable(data2)
accelSelf = round(accelSelf, sensorPrecision)
end
return angularSpeeds, accelSelf
end
function read_sensor(sensorHandle)
-- This function will read one proximity sensor using its Object Handle
-- sensorHandle: Input argument must be the sensor model base's Object Handle
-- Output arguments are result & distance
-- If no object is sensed by the sensor,
-- result = 0, distance = -1
-- If an object is sensed by the sensor,
-- result = 1, distance = distance (m) between sensor and object (rounded to mm)
-- Usage
-- result,distance = read_sensor(sensorHandle)
result,distance = sim.readProximitySensor(sensorHandle)
if distance == nil then
distance = -1
else
distance = round(distance,0.001)
end
return result,distance
end
function send_data(localSerial, localdata)
-- This function sends data to the arduino board
-- nil values will be replaced, as much as possible, with -1
-- it may take the following arguments
-- single number (single proximity sensors)
-- array of numbers (multiple proximity sensors)
-- array of arrays (proximity sensor(s) + IMU)
-- numbers will be sent to the serial port as a string in this format
-- "s #### #### #### ..."
if localdata and table.getn(localdata)>0 then
local data = unroll(localdata)
local data_str = ""
local ind = table.getn(data)
for i = 1,ind do
data_str = data_str.." "..data[i]
end
if localSerial ~= -1 then
print("Sending: ".."s ".." "..data_str)
data_str = "s ".." "..data_str..'\n'
charsSent = sim.serialSend(localSerial, data_str)
else
print("Serial port is not available")
end
end
end
function unroll(localdata, priorData)
-- This recursive function creates a single layer table from multi dimensional tables
-- it may take the following arguments
-- data
--any form of table of table of table (etc) as data
-- priorData
--is optional, is assumed to be a single layer table
result = priorData or {}
if type(localdata) == "number" then
result = {localdata}
elseif type(localdata) == "table" then
ind = table.getn(localdata)
for i = 1,ind do
if type(localdata[i]) == "number" then
table.insert(result,localdata[i])
elseif type(localdata[i]) == "table" then
unroll(localdata[i], result)
elseif type(localdata[i]) == "nil" then
table.insert(result,-1)
print("Attempting to send nil, sending -1 instead")
else
table.insert(result,-1)
print("unknown data type in subtable, sending -1 instead")
end
end
--print("results")
--print(result)
else
print('No new data to unroll?')
end
return result
end
function receive_data()
-- This function receives data from the arduino board
-- The data sent from the arduino MUST BE a comma-separated string
-- terminated by a carriage & line return "\r\n"
-- "###,###,###,###,...\r\n"
-- Those numbers are then returned by this function as a table
-- {###,###,###,###,...}
if serial ~= -1 then
str = sim.serialRead(serial,1000,true,'\n',2)
else
return nil
end
if str ~= nil then
local token
ctrl_val = {}
cpt=0
for token in string.gmatch(str, "[^,]+") do
if type(tonumber(token))=='number' then
--print(token)
cpt = cpt+1
ctrl_val[cpt] = tonumber(token)
end
end
--if ctrl_val == nil or cpt ~= arduino_arg_number then
if ctrl_val == nil then
print('unexpected data length, check arduino_arg_number var')
return nil
end
else
return nil
end
--print(ctrl_val)
return ctrl_val
end
function set_servo(servoHandle,posCmd)
-- This function will actuate the RC Servo to reach the commanded position
-- Only commanded position in the -pi/2 to pi/2 range are allowed
-- servoHandle: Input argument must be the servo model base's Object Handle
-- posCmd: angle in radian from -pi/2 to pi/2
-- Function usage
-- set_servo(servoHandle,servoPosition)
if (sim.getObjectType(servoHandle)==sim.object_forcesensor_type) then
temp = sim.getObjectChild(servoHandle,0)
child1 = sim.getObjectChild(temp,0)
if sim.getObjectType(child1)==sim.object_joint_type then
servoHandle = child1
else
child2 = sim.getObjectChild(temp,1)
if sim.getObjectType(child2)==sim.object_joint_type then
servoHandle = child2
end
end
end
if posCmd > 3.1416/2 then
print('Commanded servo position out of range')
posCmd = 3.141/2
elseif posCmd < -3.1416/2 then
print('Commanded servo position out of range')
posCmd = -3.141/2
end
sim.setJointTargetPosition(servoHandle,posCmd)
end
function set_thrust(fanHandle,state,localThrottle)
-- This function will control the thrust fan with three arguments
-- fanHandle: must be the handle of the component model base
-- state: Activity state of the fan
-- state = 0 (fan is OFF)
-- state = 1 (fan is ON)
-- Throttle: should be a number between 0 and 1.
-- Throttle = 0 (will turn off the fan)
-- Throttle = 0.5 (will give partial thrust from the fan)
-- Throttle = 1 (will give the maximum thrust from the fan)
-- Function usage
-- set_thrust(fanHandle, state, throttle)
if (sim.getObjectType(fanHandle)==sim.object_forcesensor_type) then
fanHandle = sim.getObjectChild(fanHandle,0)
end
if localThrottle > 1 then
print('Throttle out of range')
localThrottle = 1
elseif localThrottle < 0 then
print('Throttle out of range')
localThrottle = 0
state = 0
elseif localThrottle == 0 then
state = 0
elseif state == 0 then
localThrottle = 0
else
state = 1
end
sim.setUserParameter(fanHandle,'lift',0)
sim.setUserParameter(fanHandle,'state',state)
sim.setUserParameter(fanHandle,'throttle',localThrottle)
--add lift = 0
end
function set_lift(liftFan, liftState, localContactPatch)
-- This function will activate and deactivate the lift simulation
-- This is done by setting the appropriate physical properties on the main hovercraft body
-- liftState: Activity state of the fan
-- liftState = 0 (lift OFF)
-- liftState = 1 (lift ON)
-- liftfan: must be the handle of the fan providing lift component model base
-- localContactPatch: must be the handle of the contact patch of your hovercraft
-- friction: returned value for the lift friction coefficient
-- Function usage
-- friction = set_lift(liftFan, liftState, localContactPatch)
if (sim.getObjectType(liftFan)==sim.object_forcesensor_type) then
liftFan = sim.getObjectChild(liftFan,0)
end
sim.setUserParameter(liftFan,'throttle',1)
sim.setUserParameter(liftFan,'lift',1)
if liftState == 1 and sim.getIntegerSignal('lift') == 0 then
print( "Lift is activated")
XYSize = getBoundingBoxXYSize(localContactPatch)
meanRadius = (XYSize[1] + XYSize[2]) / 2 / 2
area = XYSize[1] * XYSize[2]
COMlocation, COMDelta, mass = getCenterOfMass(localContactPatch)
--print("COMlocation X: "..COMlocation[1].."; COMlocation Y: ".. COMlocation[2].."; COMlocation Z: ".. COMlocation[3].."; area: "..area.."; meanRadius: "..meanRadius)
body_pressure = mass * 9.8 / area
fan_pressure = sim.getUserParameter(liftFan,"fanPressure")
if body_pressure >= fan_pressure then
friction = 0.1
elseif body_pressure >= fan_pressure/1.25 then
friction = -0.07/0.25 * (fan_pressure/body_pressure) + 0.38
elseif body_pressure >= fan_pressure/2.5 then
friction = -0.02/1.25 * (fan_pressure/body_pressure) + 0.05
elseif body_pressure < fan_pressure/2.5 then
friction = 0.01
end
print( "body pressure: "..round(body_pressure,0.01).." Pa; "..
"fan pressure: "..round(fan_pressure,0.01).." Pa; ")
if COMDelta <= 0.01*meanRadius then
--Friction coef is untouched
elseif COMDelta <= 0.5*meanRadius then
friction = friction + (0.1-friction)/0.49 * (COMDelta/meanRadius-0.01)
elseif COMDelta > 0.5*meanRadius then
friction = 0.1
end
print( "Distance between CoM and centroid: ".. 1000*round(COMDelta,0.00001).." mm; "..
"meanRadius: "..1000*round(meanRadius,0.00001).." mm; "..
"Contact patch friction: "..round(friction,0.0001))
sim.setUserParameter(liftFan,'state',1)
sim.setIntegerSignal('lift',1)
sim.setEngineFloatParameter(sim.newton_body_staticfriction,localContactPatch,friction)
sim.setEngineFloatParameter(sim.newton_body_kineticfriction,localContactPatch,friction)
sim.resetDynamicObject(localContactPatch)
elseif liftState == 0 and sim.getIntegerSignal('lift') == 1 then
sim.setUserParameter(liftFan,'state',0)
sim.setIntegerSignal('lift',0)
sim.setEngineFloatParameter(sim.newton_body_staticfriction,localContactPatch,0.2)
sim.setEngineFloatParameter(sim.newton_body_kineticfriction,localContactPatch,0.09)
sim.resetDynamicObject(localContactPatch)
end
return round(friction,0.0001)
end
function getBoundingBoxXYSize(obj)
a, size1min = sim.getObjectFloatParameter(obj, 15)
a, size2min = sim.getObjectFloatParameter(obj, 16)
a, size3min = sim.getObjectFloatParameter(obj, 17)
a, size1max = sim.getObjectFloatParameter(obj, 18)
a, size2max = sim.getObjectFloatParameter(obj, 19)
a, size3max = sim.getObjectFloatParameter(obj, 20)
size1 = size1max-size1min
size2 = size2max-size2min
size3 = size3max-size3min
sizes = {size1, size2, size3}
minSize = math.min(size1,size2,size3)
for i = 1,3 do
if sizes[i] == minSize then
min = i
end
end
table.remove(sizes,min)
return sizes
end
function getCenterOfMass(modelBase)
-- reference: https://forum.coppeliarobotics.com/viewtopic.php?t=1719
-- Function returns the CoM for a given model in this format
-- {{CoMX, CoMY, CoMZ},{deltaX, deltaY, deltaZ},totalMass}
-- First find all non-static shapes in our model:
allNonStaticShapes={}
allObjectsToExplore={modelBase}
while (#allObjectsToExplore>0) do
obj=allObjectsToExplore[1]
table.remove(allObjectsToExplore,1)
if (sim.getObjectType(obj)==sim.object_shape_type) then
--print("object# "..obj)
r,v=sim.getObjectInt32Parameter(obj,3003)
if (v==0) then -- is the shape non-static?
table.insert(allNonStaticShapes,obj)
end
end
index=0
while true do
child=sim.getObjectChild(obj,index)
if (child==-1) then
break
end
table.insert(allObjectsToExplore,child)
index=index+1
end
end
-- Now compute the center of mass of our model (in absolute coordinates):
mass,inertia,base_com=sim.getShapeMassAndInertia(modelBase,nil)
miri={0,0,0}
totalMass=0
loc_com = {}
for i=1,#allNonStaticShapes,1 do
--print(sim.getObjectName(allNonStaticShapes[i]))
mass,inertia,com=sim.getShapeMassAndInertia(allNonStaticShapes[i],nil)
miri[1]=miri[1]+mass*com[1]
miri[2]=miri[2]+mass*com[2]
miri[3]=miri[3]+mass*com[3]
totalMass=totalMass+mass
end
final_com = {}
final_com[1]=miri[1]/totalMass
final_com[2]=miri[2]/totalMass
final_com[3]=miri[3]/totalMass
delta = math.sqrt((final_com[1]-base_com[1])^2+(final_com[2]-base_com[2])^2)
return final_com,delta,totalMass
end
function round(exact, quantum)
-- Rounding function
-- https://stackoverflow.com/questions/18313171/lua-rounding-numbers-and-then-truncate
if type(exact) == "number" then
local quant,frac = math.modf(exact/quantum)
return quantum * (quant + (frac > 0.5 and 1 or 0))
elseif type(exact) == "table" then
out = {}
for i = 1,table.getn(exact) do
out[i] = round(exact[i], quantum)
end
return out
else
print("Unexpected type sent to round() function")
end
end
function sysCall_cleanup()
-- Put some clean-up code here
if USE_ARDUINO then
if serial ~= -1 then
sim.serialClose(serial)
else
print('Your serial port was not correctly opened at simulation start')
end
end
end
-- See the user manual or the available code snippets for additional callback functions and details