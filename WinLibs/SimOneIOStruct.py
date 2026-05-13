from ctypes import *
import os
import platform

global HDMapModule, SimoneAPI, SimoneAPI_dll,SimoneStreamingAPI_dll

sys = platform.system()
if sys == "Windows":
	HDMapModule = "HDMapModule.dll"
	SimoneAPI_dll = "SimOneAPI.dll"
	SimoneStreamingAPI_dll = "SimOneStreamingAPI.dll"

elif sys == "Linux":
	HDMapModule = "libHDMapModule.so"
	SimoneAPI_dll = "libSimOneAPI.so"
	SimOneStreamingDep = ["libavutil.so.56","libswresample.so.3",
	"libx264.so.148","libavcodec.so.58","libavformat.so.58","libswscale.so.5"]
	SimoneStreamingAPI_dll = "libSimOneStreamingAPI.so"

SimOneLibPaths = [
"",
"../Build/build/bin/debug/",
"Win64/",
"../Build/build_debug/bin/debug/",
"../../../Build/build/bin/Debug/"
]

# Append full paths
SimOneLibFullPaths = []
SimOneIOStructRoot = os.getcwd()
for path in SimOneLibPaths:
	SimOneLibFullPaths.append(SimOneIOStructRoot+"/"+path)
SimOneLibPaths += SimOneLibFullPaths

LoadDllSuccess = False
for path in SimOneLibPaths:
	if(LoadDllSuccess == True):
		break
	try:# python v3.8+
		CDLL(path+HDMapModule, winmode=DEFAULT_MODE)
		SimoneAPI = CDLL(path + SimoneAPI_dll, winmode=DEFAULT_MODE)
		if sys == "Linux":
			for dep in SimOneStreamingDep:
				CDLL(path + dep)
		SimoneStreamingAPI = CDLL(path + SimoneStreamingAPI_dll, winmode=DEFAULT_MODE)
		LoadDllSuccess = True
	except Exception as e:
		pass
		print(e)

	if LoadDllSuccess:
		print("load libary sucessful")
		break
	try:
		# python 2.7 - 3.7
		CDLL(path+HDMapModule)
		SimoneAPI = CDLL(path + SimoneAPI_dll)
		if sys == "Linux":
			for dep in SimOneStreamingDep:
				CDLL(path + dep)
		SimoneStreamingAPI = CDLL(path + SimoneStreamingAPI_dll)
		LoadDllSuccess = True
	except Exception as e:
		pass
		print(e)


if LoadDllSuccess == False:
	print("[SimOne API ERROR] Load SimOneAPI.dll/ or.so failed. unable to start")
else:
	print("[SimOne API Load Success] Ready to start")

#print(SimoneStreamingAPI)
MAX_MAINVEHICLE_NUM = 10
MAX_MAINVEHICLE_NAME_LEN = 64
TOTAL_LEN = 640


class SimOne_Data_MainVehicle_Info(Structure):
	_pack_ = 1
	_fields_ = [
		('size', c_int),
		('id_list', c_char*MAX_MAINVEHICLE_NAME_LEN*MAX_MAINVEHICLE_NUM),
		('type_list', c_char*MAX_MAINVEHICLE_NAME_LEN*MAX_MAINVEHICLE_NUM)]


class SimOne_Data_MainVehicle_Status(Structure):
	_pack_ = 1
	_fields_ = [
		('mainVehicleId', c_char*MAX_MAINVEHICLE_NAME_LEN),
		('mainVehicleStatus', c_int)]


SOSM_MAP_OD_LENGT = 128
SOSM_MAP_ODURL_LENGT = 256
SOSM_MAP_ODMD5_LENGT = 128
class SimOne_Data_Map(Structure):
	_pack_ = 1
	_fields_ = [
		('openDrive',c_char*SOSM_MAP_OD_LENGT),
		('openDriveUrl',c_char*SOSM_MAP_ODURL_LENGT),
		('opendriveMd5',c_char*SOSM_MAP_ODMD5_LENGT)]

SOSM_CASENAME_LENGT = 256
SOSM_CASEID_LENGT = 256
SOSM_TASKID_LENGT = 256
SOSM_SESSIONID_LENGT = 256
SOSM_EXTRA_STATES_SIZE_MAX = 256
class ESimOne_Case_Status(c_int):
	ESimOne_Case_Status_Unknown = 0
	ESimOne_Case_Status_Stop = 1
	ESimOne_Case_Status_Running = 2
	ESimOne_Case_Status_Pause = 3

class SimOne_Data_CaseInfo(Structure):
	_pack_ = 1
	_fields_ = [
		('caseName', c_char*SOSM_CASENAME_LENGT),
		('caseId', c_char*SOSM_CASEID_LENGT),
		('taskId', c_char*SOSM_TASKID_LENGT)]


class SimOne_Data(Structure):
	_pack_ = 1
	_fields_ = [
	('timestamp', c_longlong),
	('frame', c_int),
	('version', c_int)]
	

class ESimOne_TrafficLight_Status(c_int):
	ESimOne_TrafficLight_Status_Invalid = 0
	ESimOne_TrafficLight_Status_Red = 1
	ESimOne_TrafficLight_Status_Green = 2
	ESimOne_TrafficLight_Status_Yellow = 3
	ESimOne_TrafficLight_Status_RedBlink = 4
	ESimOne_TrafficLight_Status_GreenBlink = 5
	ESimOne_TrafficLight_Status_YellowBlink = 6
	ESimOne_TrafficLight_Status_Black = 7

class ESimOne_Data_Vehicle_State(c_int):
	ESimOne_Data_Vehicle_State_SO_M_SW=0
	ESimOne_Data_Vehicle_State_S0_Vx_SM=1
	ESimOne_Data_Vehicle_State_S0_Vy_SM=2
	ESimOne_Data_Vehicle_State_S0_Vz_SM=3
	ESimOne_Data_Vehicle_State_SO_M_ENGOUT=4
	ESimOne_Data_Vehicle_State_SO_My_DR_L1=5
	ESimOne_Data_Vehicle_State_SO_My_DR_R1=6
	ESimOne_Data_Vehicle_State_SO_My_DR_L2=7
	ESimOne_Data_Vehicle_State_SO_My_DR_R2=8
	ESimOne_Data_Vehicle_State_SO_My_DR_L3=9
	ESimOne_Data_Vehicle_State_SO_My_DR_R3=10

	ESimOne_Data_Vehicle_State_SO_F_Pedal=11    # Brake Pedal Force
	ESimOne_Data_Vehicle_State_SO_Pbk_Con=12    # BrakeMasterCylinder Pressure
	ESimOne_Data_Vehicle_State_SO_My_Bk_L3=13   # Brake Torque at Left Rear Wheel
	ESimOne_Data_Vehicle_State_SO_My_Bk_R3=14   # Brake Torque at Right Rear Wheel

	ESimOne_Data_Vehicle_State_SO_ClutchTr=15   # Transmission clutch control

	ESimOne_Data_Vehicle_State_SO_X_L1=16       # x coordinate, wheel center L1
	ESimOne_Data_Vehicle_State_SO_Y_L1=17
	ESimOne_Data_Vehicle_State_SO_Z_L1=18
	ESimOne_Data_Vehicle_State_SO_X_L2=19
	ESimOne_Data_Vehicle_State_SO_Y_L2=20
	ESimOne_Data_Vehicle_State_SO_Z_L2=21
	ESimOne_Data_Vehicle_State_SO_X_R1=22
	ESimOne_Data_Vehicle_State_SO_Y_R1=23
	ESimOne_Data_Vehicle_State_SO_Z_R1=24
	ESimOne_Data_Vehicle_State_SO_X_R2=25
	ESimOne_Data_Vehicle_State_SO_Y_R2=26
	ESimOne_Data_Vehicle_State_SO_Z_R2=27
	ESimOne_Data_Vehicle_State_SO_X_L3=28       # x coordinate, wheel center L3
	ESimOne_Data_Vehicle_State_SO_Y_L3=29
	ESimOne_Data_Vehicle_State_SO_Z_L3=30
	ESimOne_Data_Vehicle_State_SO_X_R3=31
	ESimOne_Data_Vehicle_State_SO_Y_R3=32
	ESimOne_Data_Vehicle_State_SO_Z_R3=33

	ESimOne_Data_Vehicle_State_SO_Xctc_L1=34      # x coordinate, center of tire contact L1
	ESimOne_Data_Vehicle_State_SO_Yctc_L1=35      # y coordinate, center of tire contact L1
	ESimOne_Data_Vehicle_State_SO_Zctc_L1=36      # z coordinate, center of tire contact L1
	ESimOne_Data_Vehicle_State_SO_Xctc_L2=37
	ESimOne_Data_Vehicle_State_SO_Yctc_L2=38
	ESimOne_Data_Vehicle_State_SO_Zctc_L2=39
	ESimOne_Data_Vehicle_State_SO_Xctc_R1=40
	ESimOne_Data_Vehicle_State_SO_Yctc_R1=41
	ESimOne_Data_Vehicle_State_SO_Zctc_R1=42
	ESimOne_Data_Vehicle_State_SO_Xctc_R2=43
	ESimOne_Data_Vehicle_State_SO_Yctc_R2=44
	ESimOne_Data_Vehicle_State_SO_Zctc_R2=45

	ESimOne_Data_Vehicle_State_SO_AVy_L1=46     # Wheel L1 spin    unit: rpm
	ESimOne_Data_Vehicle_State_SO_Kappa_L1=47   # Longitudinal slip, tire L1   ratio, no unit
	ESimOne_Data_Vehicle_State_SO_Alpha_L1=48   # Lateral slip angle, tire L1  unit: degree
	ESimOne_Data_Vehicle_State_SO_Fz_L1=49      # Vertical force, tire L1   unit: N
	ESimOne_Data_Vehicle_State_SO_My_WC_L1=50   # Tire My at wheel center L1   unit: N.m
	ESimOne_Data_Vehicle_State_SO_Fx_L1=51      # Longitudinal force, tire L1   unit: N
	ESimOne_Data_Vehicle_State_SO_Fy_L1=52      # lateral force, tire L1  unit: N
	ESimOne_Data_Vehicle_State_SO_Jnc_L1=53     # Wheel L1 jounce (compression)   unit: mm
	ESimOne_Data_Vehicle_State_SO_JncR_L1=54    # Wheel L1 jounce rate         unit: mm/s
	ESimOne_Data_Vehicle_State_SO_Mz_L1=55      # Aligning moment, tire L1   unit: Nm

	ESimOne_Data_Vehicle_State_SO_AVy_L2=56     #  Wheel L2 spin    unit: rpm
	ESimOne_Data_Vehicle_State_SO_Kappa_L2=57   # Longitudinal slip, tire L2   ratio, no unit
	ESimOne_Data_Vehicle_State_SO_Alpha_L2=58   # Lateral slip angle, tire L2  unit: rad
	ESimOne_Data_Vehicle_State_SO_Fz_L2=59
	ESimOne_Data_Vehicle_State_SO_My_WC_L2=60
	ESimOne_Data_Vehicle_State_SO_Fx_L2=61       # Longitudinal force, tire L2
	ESimOne_Data_Vehicle_State_SO_Fy_L2=62       # lateral force, tire L2
	ESimOne_Data_Vehicle_State_SO_Jnc_L2=63      # Wheel L2 jounce (compression)  unit: cm
	ESimOne_Data_Vehicle_State_SO_JncR_L2=64     # Wheel L2 jounce rate   unit: cm/s
	ESimOne_Data_Vehicle_State_SO_Mz_L2=65       # Aligning moment, tire L2  unit: Nm

	ESimOne_Data_Vehicle_State_SO_AVy_R1=66
	ESimOne_Data_Vehicle_State_SO_Kappa_R1=67
	ESimOne_Data_Vehicle_State_SO_Alpha_R1=68    # Lateral slip angle, tire R1  unit: degree
	ESimOne_Data_Vehicle_State_SO_Fz_R1=69
	ESimOne_Data_Vehicle_State_SO_My_WC_R1=70
	ESimOne_Data_Vehicle_State_SO_Fx_R1=71       # Longitudinal force, tire R1
	ESimOne_Data_Vehicle_State_SO_Fy_R1=72       # lateral force, tire R1
	ESimOne_Data_Vehicle_State_SO_Jnc_R1=73      # Wheel R1 jounce (compression)
	ESimOne_Data_Vehicle_State_SO_JncR_R1=74     # Wheel R1 jounce rate
	ESimOne_Data_Vehicle_State_SO_Mz_R1=75       # Aligning moment, tire R1

	ESimOne_Data_Vehicle_State_SO_AVy_R2=76
	ESimOne_Data_Vehicle_State_SO_Kappa_R2=77
	ESimOne_Data_Vehicle_State_SO_Alpha_R2=78    # Lateral slip angle, tire R2  unit: degree
	ESimOne_Data_Vehicle_State_SO_Fz_R2=79
	ESimOne_Data_Vehicle_State_SO_My_WC_R2=80
	ESimOne_Data_Vehicle_State_SO_Fx_R2=81       # Longitudinal force, tire R2
	ESimOne_Data_Vehicle_State_SO_Fy_R2=82       # lateral force, tire R2
	ESimOne_Data_Vehicle_State_SO_Jnc_R2=83      # Wheel R2 jounce (compression)
	ESimOne_Data_Vehicle_State_SO_JncR_R2=84     # Wheel R2 jounce rate
	ESimOne_Data_Vehicle_State_SO_Mz_R2=85       # Aligning moment, tire R2

	ESimOne_Data_Vehicle_State_SO_AVy_L3=86     # Wheel L3 spin    unit: rpm
	ESimOne_Data_Vehicle_State_SO_Kappa_L3=87   # Longitudinal slip, tire L3   ratio, no unit
	ESimOne_Data_Vehicle_State_SO_Alpha_L3=88   # Lateral slip angle, tire L3  unit: degree
	ESimOne_Data_Vehicle_State_SO_Fz_L3=89      # Vertical force, tire L3   unit: N
	ESimOne_Data_Vehicle_State_SO_My_WC_L3=90   # Tire My at wheel center L3   unit: N.m
	ESimOne_Data_Vehicle_State_SO_Fx_L3=91      # Longitudinal force, tire L3   unit: N
	ESimOne_Data_Vehicle_State_SO_Fy_L3=92      # lateral force, tire L3  unit: N
	ESimOne_Data_Vehicle_State_SO_Jnc_L3=93     # Wheel L3 jounce (compression)   unit: mm
	ESimOne_Data_Vehicle_State_SO_JncR_L3=94    # Wheel L3 jounce rate         unit: mm/s
	ESimOne_Data_Vehicle_State_SO_Mz_L3=95      # Aligning moment, tire L3   unit: Nm

	ESimOne_Data_Vehicle_State_SO_AVy_R3=96
	ESimOne_Data_Vehicle_State_SO_Kappa_R3=97
	ESimOne_Data_Vehicle_State_SO_Alpha_R3=98    # Lateral slip angle, tire R3  unit: degree
	ESimOne_Data_Vehicle_State_SO_Fz_R3=99
	ESimOne_Data_Vehicle_State_SO_My_WC_R3=100
	ESimOne_Data_Vehicle_State_SO_Fx_R3=101       # Longitudinal force, tire R3
	ESimOne_Data_Vehicle_State_SO_Fy_R3=102       # lateral force, tire R3
	ESimOne_Data_Vehicle_State_SO_Jnc_R3=103      # Wheel R3 jounce (compression)
	ESimOne_Data_Vehicle_State_SO_JncR_R3=104     # Wheel R3 jounce rate
	ESimOne_Data_Vehicle_State_SO_Mz_R3=105       # Aligning moment, tire R3

	ESimOne_Data_Vehicle_State_SO_Steer_L3=106
	ESimOne_Data_Vehicle_State_SO_Steer_R3=107

	ESimOne_Data_Vehicle_State_SO_Steer_SW=108   # Steering wheel angle  unit: deg
	ESimOne_Data_Vehicle_State_SO_TimePassed=109

SOSM_VEHICLE_EXTRA_STATE_MAX_SIZE = 600
class SimOne_Data_Vehicle_Extra(Structure):
	_pack_ = 1
	_fields_ = [
	("dataSize",c_int),
	("extra_states",c_float*SOSM_VEHICLE_EXTRA_STATE_MAX_SIZE)
	]

SOSM_TRAFFICLIGHT_SIZE_MAX = 100
class SimOne_Data_TrafficLight(Structure):
	_pack_ = 1
	_fields_ = [
	('index', c_int),
	('opendriveLightId', c_int),
	('countDown', c_int),
	('status', ESimOne_TrafficLight_Status)
	]


class SimOne_Data_TrafficLights(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('trafficlightNum', c_int),
		('trafficlights', SimOne_Data_TrafficLight*SOSM_TRAFFICLIGHT_SIZE_MAX)]


class ESimOne_Signal_Light(c_int):
	ESimOne_Signal_Light_None = 0
	ESimOne_Signal_Light_RightBlinker = 1
	ESimOne_Signal_Light_LeftBlinker = (1 << 1)
	ESimOne_Signal_Light_DoubleFlash = (1 << 2)
	ESimOne_Signal_Light_BrakeLight = (1 << 3)
	ESimOne_Signal_Light_FrontLight = (1 << 4)
	ESimOne_Signal_Light_HighBeam = (1 << 5)
	ESimOne_Signal_Light_BackDrive = (1 << 6)


class SimOne_Data_Signal_Lights(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('signalLights', c_uint)]


class SimOne_Data_Pose_Control(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('posX', c_float),
	('posY', c_float),
	('posZ', c_float),
	('oriX', c_float),
	('oriY', c_float),
	('oriZ', c_float),
	('autoZ', c_bool)]


class ESimOne_Gear_Mode(c_int):
	ESimOne_Gear_Mode_Neutral = 0
	ESimOne_Gear_Mode_Drive = 1      # forward gear for automatic gear
	ESimOne_Gear_Mode_Reverse = 2
	ESimOne_Gear_Mode_Parking = 3

	ESimOne_Gear_Mode_1 = 4    # forward gear 1 for manual gear
	ESimOne_Gear_Mode_2 = 5
	ESimOne_Gear_Mode_3 = 6
	ESimOne_Gear_Mode_4 = 7
	ESimOne_Gear_Mode_5 = 8
	ESimOne_Gear_Mode_6 = 9
	ESimOne_Gear_Mode_7 = 10
	ESimOne_Gear_Mode_8 = 11

class ESimOne_Throttle_Mode(c_int):
    ESimOne_Throttle_Mode_Percent = 0         # [0, 1]
    ESimOne_Throttle_Mode_Torque = 1          # engine torque, N.m
    ESimOne_Throttle_Mode_Speed = 2           # vehicle speed, m/s,   in this mode, brake input is ignored
    ESimOne_Throttle_Mode_Accel = 3           # vehicle acceleration, m/s^2, in this mode, brake input is ignored
    ESimOne_Throttle_Mode_EngineAV = 4        # engine, rpm
    ESimOne_Throttle_Mode_WheelTorque = 5      # torques applied to each wheel, array, size is the wheel number, N.m

class ESimOne_Brake_Mode(c_int):
    ESimOne_Brake_Mode_Percent = 0
    ESimOne_Brake_Mode_MasterCylinderPressure = 1 # degree
    ESimOne_Brake_Mode_PedalForce = 2
    ESimOne_Brake_Mode_WheelCylinderPressure = 3   # Mpa for each wheel
    ESimOne_Brake_Mode_WheelTorque = 4             # Nm for each wheel
#
class ESimOne_Steering_Mode(c_int):
    ESimOne_Steering_Mode_Percent = 0
    ESimOne_Steering_Mode_SteeringWheelAngle = 1
    ESimOne_Steering_Mode_Torque = 2
    ESimOne_Steering_Mode_AngularSpeed = 3            # steering wheel angualr speed, degree/s
    ESimOne_Steering_Mode_WheelAngle = 4              # degree for each wheel
    ESimOne_Steering_Mode_WheelAnglarSpeed = 5        # degree/s for each wheel

class ESimOne_LogLevel_Type(c_int):
	ESimOne_LogLevel_Type_Debug = 0
	ESimOne_LogLevel_Type_Information = 1
	ESimOne_LogLevel_Type_Warning = 2
	ESimOne_LogLevel_Type_Error = 3
	ESimOne_LogLevel_Type_Fatal = 4

SOSM_MAX_WHEEL_NUM = 20
class SimOne_Data_Control(SimOne_Data):
	_pack_ = 1
	_fields_ = [
    ('EThrottleMode', ESimOne_Throttle_Mode),
	('throttle', c_float),
	('EBrakeMode', ESimOne_Brake_Mode),
	('brake', c_float),
	('ESteeringMode', ESimOne_Steering_Mode),
	('steering', c_float),
	('handbrake', c_bool),
	('isManualGear', c_bool),
	('gear', ESimOne_Gear_Mode), # 0: Neutral; 1: Drive; 2: Reverse; 3: Parking
    ('clutch', c_float),
    ('throttle_input_data', c_float * SOSM_MAX_WHEEL_NUM),
    ('brake_input_data', c_float * SOSM_MAX_WHEEL_NUM),
    ('steering_input_data', c_float * SOSM_MAX_WHEEL_NUM)]

class SimOne_Data_ESP_Control(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('stopDistance', c_int),
	('velocityLimit', c_float),
	('steering', c_float),
	('steerTorque', c_float),
	('accel', c_float),
	('accelUpperLimit', c_float),
	('accelLowerLimit', c_float),
	('accelUpperComfLimit', c_float),
	('accelLowerComfLimit', c_float),
	('standStill', c_bool),
	('driveOff', c_bool),
	('brakeMode', c_int),
	('vlcShutdown', c_int),
	('gearMode', c_int)]

class ESimone_Vehicle_EventInfo_Type(c_int):
	ESimone_Vehicle_EventInfo_Type_Forward_Collision_Warning = 0		# 	front_crash_warning
	ESimone_Vehicle_EventInfo_Type_Backward_Collision_Warning = 1	# 	back_crash_warning
	ESimone_Vehicle_EventInfo_Type_Left_Turn_Decision = 2			# 	turn_left
	ESimone_Vehicle_EventInfo_Type_Left_Turn_Warning = 3				# 	left_warning
	ESimone_Vehicle_EventInfo_Type_Right_Turn_Decision = 4			# 	turn_right
	ESimone_Vehicle_EventInfo_Type_Right_Turn_Warning = 5			# 	right_warning
	ESimone_Vehicle_EventInfo_Type_Forward_Straight_Decision = 6		# 	straight_through
	ESimone_Vehicle_EventInfo_Type_Forward_Straight_Warning = 7		# 	straight_warning
	ESimone_Vehicle_EventInfo_Type_Over_Speed_Warning = 8			# 	overspeeding_warning
	ESimone_Vehicle_EventInfo_Type_Lane_Change_Decision = 9			#  lane_change
	ESimone_Vehicle_EventInfo_Type_Lane_Change_Warning = 10			# 	lane_change_warning
	ESimone_Vehicle_EventInfo_Type_Overtake_Decision = 11			# 	overtake
	ESimone_Vehicle_EventInfo_Type_Emergency_Braking_Decision = 12	# 	emergency_braking
	ESimone_Vehicle_EventInfo_Type_Accelerate_Decision = 13			# 	accelerate

class SimOne_Data_Vehicle_EventInfo(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('type', ESimone_Vehicle_EventInfo_Type)]


SOSM_TRAJECTORY_SIZE_MAX = 100


class SimOne_Data_Trajectory_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('posX', c_float), # Trajectory Position X no Opendrive (by meter)
	('posY', c_float), # Trajectory Position Y no Opendrive (by meter)
	('vel', c_float)]  # Velocity (by meter/second)
	

class SimOne_Data_Trajectory(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('trajectorySize', c_int),
	('trajectory', SimOne_Data_Trajectory_Entry * SOSM_TRAJECTORY_SIZE_MAX)]


class SimOne_Data_IMU(Structure):
	_pack_ = 1
	_fields_ = [
		('accelX', c_float), # Position X no Opendrive (by meter)
		('accelY', c_float), # Position Y no Opendrive (by meter)
		('accelZ', c_float), # Position Z no Opendrive (by meter)
		('velX', c_float), # MainVehicle Velocity X on Opendrive (by meter)
		('velY', c_float), # MainVehicle Velocity Y on Opendrive (by meter)
		('velZ', c_float), # MainVehicle Velocity Z on Opendrive (by meter)
		('angVelX', c_float), # MainVehicle Angular Velocity X on Opendrive (by meter)
		('angVelY', c_float), # MainVehicle Angular Velocity Y on Opendrive (by meter)
		('angVelZ', c_float), # MainVehicle Angular Velocity Z on Opendrive (by meter)
		('rotX', c_float), # Rotation X on Opendrive (by radian)
		('rotY', c_float), # Rotation Y on Opendrive (by radian)
		('rotZ', c_float) # Rotation Z on Opendrive (by radian)
	]

class SimOne_Data_Gps(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('posX', c_float), # Position X no Opendrive (by meter)
	('posY', c_float), # Position Y no Opendrive (by meter)
	('posZ', c_float), # Position Z no Opendrive (by meter)
	('oriX', c_float), # Rotation X no Opendrive (by radian)
	('oriY', c_float), # Rotation Y no Opendrive (by radian)
	('oriZ', c_float), # Rotation Z no Opendrive (by radian)
	('velX', c_float), # MainVehicle Velocity X on Opendrive (by meter)
	('velY', c_float), # MainVehicle Velocity Y on Opendrive (by meter)
	('velZ', c_float), # MainVehicle Velocity Z on Opendrive (by meter)
	('throttle', c_float), #MainVehicle Throttle
	('brake', c_float), #MainVehicle brake
	('steering', c_float), #MainVehicle Steering angle
	('gear', c_int), # MainVehicle gear position
	('accelX', c_float), # MainVehilce Acceleration X on Opendrive (by meter)
	('accelY', c_float), # MainVehilce Acceleration Y on Opendrive (by meter)
	('accelZ', c_float), # MainVehilce Acceleration Z on Opendrive (by meter)
	('angVelX', c_float), # MainVehilce Angular Velocity X on Opendrive (by meter)
	('angVelY', c_float), # MainVehilce Angular Velocity Y on Opendrive (by meter)
	('angVelZ', c_float), # MainVehilce Angular Velocity Z on Opendrive (by meter)
	('wheelSpeedFL', c_float), # Speed of front left wheel (by meter/sec)
	('wheelSpeedFR', c_float), # Speed of front right wheel (by meter/sec)
	('wheelSpeedRL', c_float), # Speed of rear left wheel (by meter/sec)
	('wheelSpeedRR', c_float), # Speed of rear right wheel (by meter/sec)
	('engineRpm', c_float), # Speed of engine (by r/min)
	('odometer', c_float),#  odometer in meter.
	('extraStateSize', c_int),
	('extraStates', c_float*SOSM_EXTRA_STATES_SIZE_MAX), # vehicle states subscripted by MainVehicleExtraDataIndics message
	('isGPSLost', c_bool),
	('imuData', SimOne_Data_IMU)]

SOSM_OBSTACLE_SIZE_MAX = 255

class ESimOne_Obstacle_Type(c_int):
	ESimOne_Obstacle_Type_Unknown = 0
	ESimOne_Obstacle_Type_Pedestrian = 4
	ESimOne_Obstacle_Type_Pole = 5
	ESimOne_Obstacle_Type_Car = 6
	ESimOne_Obstacle_Type_Static = 7
	ESimOne_Obstacle_Type_Bicycle = 8
	ESimOne_Obstacle_Type_Fence = 9
	ESimOne_Obstacle_Type_RoadMark = 12
	ESimOne_Obstacle_Type_TrafficSign = 13
	ESimOne_Obstacle_Type_TrafficLight = 15
	ESimOne_Obstacle_Type_Rider = 17
	ESimOne_Obstacle_Type_Truck = 18
	ESimOne_Obstacle_Type_Bus = 19
	ESimOne_Obstacle_Type_SpecialVehicle = 20
	ESimOne_Obstacle_Type_Motorcycle = 21
	ESimOne_Obstacle_Type_Dynamic = 22
	ESimOne_Obstacle_Type_GuardRail = 23
	ESimOne_Obstacle_Type_SpeedLimitSign = 26
	ESimOne_Obstacle_Type_BicycleStatic = 27
	ESimOne_Obstacle_Type_RoadObstacle = 29


class SimOne_Data_Obstacle_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('id', c_int), # Obstacle ID
	('viewId', c_int),  # Obstacle ID
	('type', ESimOne_Obstacle_Type), # Obstacle Type
	('theta', c_float), # Obstacle vertical rotation (by radian)
	('posX', c_float), # Obstacle Position X no Opendrive (by meter)
	('posY', c_float), # Obstacle Position Y no Opendrive (by meter)
	('posZ', c_float), # Obstacle Position Z no Opendrive (by meter)
	('oriX', c_float), # Obstacle Velocity X no Opendrive (by meter)
	('oriY', c_float), # Obstacle Velocity Y no Opendrive (by meter)
	('oriZ', c_float), # Obstacle Velocity Z no Opendrive (by meter)
	('velX', c_float), # Obstacle Velocity X no Opendrive (by meter)
	('velY', c_float), # Obstacle Velocity Y no Opendrive (by meter)
	('velZ', c_float), # Obstacle Velocity Z no Opendrive (by meter)
	('length', c_float), 	# Obstacle length
	('width', c_float),  	# Obstacle width
	('height', c_float),	# Obstacle height
	('accelX', c_float),	# Obstacle Acceleration X
	('accelY', c_float),	# Obstacle Acceleration Y
	('accelZ', c_float)] 	# Obstacle Acceleration Z


class SimOne_Data_Obstacle(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('obstacleSize', c_int), # Obstacle Size
	('obstacle', SimOne_Data_Obstacle_Entry*SOSM_OBSTACLE_SIZE_MAX)] # Obstacles

SOSM_IMAGE_WIDTH_MAX = 3840
SOSM_IMAGE_HEIGHT_MAX = 2160
SOSM_IMAGE_DATA_SIZE_MAX = SOSM_IMAGE_WIDTH_MAX*SOSM_IMAGE_HEIGHT_MAX*3


class ESimOne_Node_Type(c_int):
	ESimOne_Node_Type_Vehicle = 0
	ESimOne_Node_Type_Camera = 1
	ESimOne_Node_Type_LiDAR = 2
	ESimOne_Node_Type_MMWRadar = 3
	ESimOne_Node_Type_UltrasonicRadar = 4
	ESimOne_Node_Type_AllUltrasonicRadar = 5
	ESimOne_Node_Type_GNSSINS = 6
	ESimOne_Node_Type_PerfectPerception = 7
	ESimOne_Node_Type_V2X = 8
	ESimOne_Node_Type_SensorFusion = 9

SENSOR_IDTYPE_MAX = 64
class SimOne_Data_SensorConfiguration(Structure):
	_pack_ = 1
	_fields_ = [
	('id', c_int),
	('mainVehicle', c_char*SENSOR_IDTYPE_MAX), # 
	('sensorId', c_char*SENSOR_IDTYPE_MAX), # Sensor's ID
	('sensorType', c_char*SENSOR_IDTYPE_MAX),# Sensor's ESimOneNodeType
	('x', c_float),# Obstacle Position X no Opendrive (by meter)
	('y', c_float),# Obstacle Position Y no Opendrive (by meter)
	('z', c_float),# Obstacle Position Z no Opendrive (by meter)
	('roll', c_float),# Sensor's Rotation coordinates in x
	('pitch', c_float),# Sensor's Rotation coordinates in y
	('yaw', c_float),# Sensor's Rotation coordinates in z
	('hz', c_int)]#Sensor's frequency


SOSM_SENSOR_CONFIGURATION_SIZE_MAX = 100


class SimOne_Data_SensorConfigurations(Structure):
	_pack_ = 1
	_fields_ =[
		('dataSize', c_int),  # num of Sensors
		('data', SimOne_Data_SensorConfiguration*SOSM_SENSOR_CONFIGURATION_SIZE_MAX)
	]
    
class ESimOne_Image_Format(c_int):
	ESimOne_Image_Format_RGB = 0
	ESimOne_Image_Format_RLESegmentation = 1
	ESimOne_Image_Format_JPEG = 2

class SimOne_Data_Image(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('width', c_int), # Image resolution width 1920 max
	('height', c_int), # Image resolution height 1080 max
	('format', ESimOne_Image_Format), # Image format. 0: RGB
	('imagedataSize', c_int), # Image data size
	('imagedata', c_char * SOSM_IMAGE_DATA_SIZE_MAX)] #1920 x 1080 x 3 max


SOSM_POINT_DATA_SIZE_MAX = 64*57600


class SimOne_Data_Point_Cloud(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('width', c_int),
	('height', c_int),
	('pointStep', c_int),
	('pointCloudDataSize', c_int),
	('pointClouddata', c_char * SOSM_POINT_DATA_SIZE_MAX)]


SOSM_RADAR_SIZE_MAX = 256


class SimOne_Data_RadarDetection_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('id', c_int), # Obstacle ID
	('subId', c_int), # Obstacle Sub ID
	('type', ESimOne_Obstacle_Type), # Obstacle Type
	('posX', c_float), # Obstacle Position X on Opendrive (by meter)
	('posY', c_float), # Obstacle Position Y on Opendrive (by meter)
	('posZ', c_float), # Obstacle Position Z on Opendrive (by meter)
	('velX', c_float), # Obstacle Velocity X (m/s)
	('velY', c_float), # Obstacle Velocity Y (m/s)
	('velZ', c_float), # Obstacle Velocity Z (m/s)
	('accelX', c_float), # Obstacle Acceleration X (m/s^2)
	('accelY', c_float), # Obstacle Acceleration Y (m/s^2)
	('accelZ', c_float), # Obstacle Acceleration Z (m/s^2)
	('oriX', c_float), # Obstacle Rotation X (by radian)
	('oriY', c_float), # Obstacle Rotation Y (by radian)
	('oriZ', c_float), # Obstacle Rotation Z (by radian)
	('length', c_float), # Obstacle length
	('width', c_float), # Obstacle width 
	('height', c_float), # Obstacle height 
	('range', c_float), # Obstacle relative range in meter 
	('rangeRate', c_float), # Obstacle relative range rate in m/s
	('azimuth', c_float), # Obstacle azimuth angle
	('vertical', c_float), # Obstacle vertical angle
	('snrdb', c_float), # Signal noise ratio
	('rcsdb', c_float), # Obstacle RCS
	('probability', c_float)] # detection probability


class SimOne_Data_RadarDetection(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('detectNum', c_int), # Obstacle Size
	('detections', SimOne_Data_RadarDetection_Entry*SOSM_RADAR_SIZE_MAX)] # Obstacles

class SimOne_Data_UltrasonicRadarDetection_Entry(Structure):
	_pack_ = 1
	_fields_ = [
		('obstacleRanges', c_float),  # Obstacle from Ultrasonic distance
		('x', c_float),	# Obstacle relativelX
		('y', c_float),	# Obstacle relativelY
		('z', c_float)	# Obstacle relativelZ
		]

class SimOne_Data_UltrasonicRadar(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('sensorId', c_char * SENSOR_IDTYPE_MAX),  # Ultrasonic ID
		('obstacleNum', c_int),  # Ultrasonic detect object nums
		('obstacleDetections', SimOne_Data_UltrasonicRadarDetection_Entry * SOSM_OBSTACLE_SIZE_MAX) # object information
		]


SOSM_ULTRASONICRADAR_SIZE_MAX = 100
class SimOne_Data_UltrasonicRadars(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('ultrasonicRadarNum', c_int), # ultrasonic radar count
		('ultrasonicRadars', SimOne_Data_UltrasonicRadar * SOSM_ULTRASONICRADAR_SIZE_MAX)]# ultrasonic radars


SOSM_V2X_MSGFRAME_SIZE_MAX = 20000
class SimOne_Data_V2XNFS(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('V2XMsgFrameSize', c_int),
		('MsgFrameData', c_char*SOSM_V2X_MSGFRAME_SIZE_MAX)]


SOSM_SENSOR_DETECTIONS_OBJECT_SIZE_MAX = 256

class SimOne_Data_SensorDetections_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('id', c_int),			# Detection Object ID
	('type', ESimOne_Obstacle_Type),		# Detection Object Type
	('posX', c_float),		# Detection Object Position X in meter
	('posY', c_float),		# Detection Object Position Y in meter
	('posZ', c_float),		# Detection Object Position Z in meter
	('oriX', c_float),		# Rotation X in radian
	('oriY', c_float),		# Rotation Y in radian
	('oriZ', c_float),		# Rotation Z in radian
	('length', c_float),	# Detection Object Length in meter
	('width', c_float),		# Detection Object Width in meter
	('height', c_float),	# Detection Object Height in meter
	('range', c_float),		# Detection Object nearest range in meter
	('velX', c_float),		# Detection Object Velocity X
	('velY', c_float),		# Detection Object Velocity Y
	('velZ', c_float),		# Detection Object Velocity Z
	('accelX',c_float), 		# Detection Object accel X
	('accelY',c_float), 		# Detection Object accel Y
	('accelZ',c_float), 		# Detection Object accel Z
	('probability', c_float),	# Detection probability
	('relativePosX', c_float),	# Relative position X in sensor space
	('relativePosY', c_float),	# Relative position Y in sensor space
	('relativePosZ', c_float),	# Relative position Z in sensor space
	('relativeRotX', c_float),	# Relative rotation X in sensor space
	('relativeRotY', c_float),	# Relative rotation Y in sensor space
	('relativeRotZ', c_float),	# Relative rotation Z in sensor space
	('relativeVelX', c_float),	# Relative velocity X in sensor space
	('relativeVelY', c_float),	# Relative velocity Y in sensor space
	('relativeVelZ', c_float),	# Relative velocity Z in sensor space
	('bbox2dMinX', c_float),	# bbox2d minX in pixel if have
	('bbox2dMinY', c_float),	# bbox2d minY in pixel if have
	('bbox2dMaxX', c_float),	# bbox2d maxX in pixel if have
	('bbox2dMaxY', c_float)]	# bbox2d maxY in pixel if have


class SimOne_Data_Environment(Structure):
	_pack_ = 1
	_fields_ = [
	('timeOfDay', c_float),		# Environment timeOfDay
	('heightAngle', c_float),		# Environment heightAngle
	('directionalLight', c_float),		# Environment directionalLight
	('ambientLight', c_float),		# Environment ambientLight
	('artificialLight', c_float),		# Environment artificialLight
	('cloudDensity', c_float),		# Environment cloudDensity
	('fogDensity', c_float),	# Environment fogDensity
	('rainDensity', c_float),		# Environment rainDensity
	('snowDensity', c_float),	# Environment snowDensity
	('groundHumidityLevel', c_float),	# Environment groundHumidityLevel
	('groundDirtyLevel', c_float)]	# Environment groundDirtyLevel


class SimOne_Data_SensorDetections(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('objectSize', c_int), # Detection Object Size
	('objects', SimOne_Data_SensorDetections_Entry*SOSM_SENSOR_DETECTIONS_OBJECT_SIZE_MAX)] # Detection Objects 


SOSM_OSI_DATA_SIZE_MAX = 1024*1024*8

SOSM_LANE_NAME_MAX = 128
SOSM_LANE_POINT_MAX = 65535
SOSM_NEAR_LANE_MAX = 128
SOSM_LANE_LINK_MAX = 1024
SOSM_PARKING_SPACE_MAX = 65535
SOSM_PARKING_SPACE_KNOTS_MAX = 4


class ESimOne_Lane_Type(c_int):
	ESimOne_Lane_Type_none = 0
	ESimOne_Lane_Type_driving = 1
	ESimOne_Lane_Type_stop = 2
	ESimOne_Lane_Type_shoulder = 3
	ESimOne_Lane_Type_biking = 4
	ESimOne_Lane_Type_sidewalk = 5
	ESimOne_Lane_Type_border = 6
	ESimOne_Lane_Type_restricted = 7
	ESimOne_Lane_Type_parking = 8
	ESimOne_Lane_Type_bidirectional = 9
	ESimOne_Lane_Type_median = 10
	ESimOne_Lane_Type_special1 = 11
	ESimOne_Lane_Type_special2 = 12
	ESimOne_Lane_Type_special3 = 13
	ESimOne_Lane_Type_roadWorks = 14
	ESimOne_Lane_Type_tram = 15
	ESimOne_Lane_Type_rail = 16
	ESimOne_Lane_Type_entry = 17
	ESimOne_Lane_Type_exit = 18
	ESimOne_Lane_Type_offRamp = 19
	ESimOne_Lane_Type_onRamp = 20
	ESimOne_Lane_Type_mwyEntry = 21
	ESimOne_Lane_Type_mwyExit = 22

class ESimOne_Boundary_Type(c_int):
	ESimOne_Boundary_Type_none = 0
	ESimOne_Boundary_Type_solid = 1
	ESimOne_Boundary_Type_broken = 2
	ESimOne_Boundary_Type_solid_solid = 3
	ESimOne_Boundary_Type_solid_broken = 4
	ESimOne_Boundary_Type_broken_solid = 5
	ESimOne_Boundary_Type_broken_broken = 6
	ESimOne_Boundary_Type_botts_dots = 7
	ESimOne_Boundary_Type_grass = 8
	ESimOne_Boundary_Type_curb = 9

class ESimOne_Boundary_Color(c_int):
	ESimOne_Boundary_Color_standard = 0
	ESimOne_Boundary_Color_blue = 1
	ESimOne_Boundary_Color_green = 2
	ESimOne_Boundary_Color_red = 3
	ESimOne_Boundary_Color_white = 4
	ESimOne_Boundary_Color_yellow = 5

class SimOneData_Vec3f(Structure):
	_pack_ = 1
	_fields_ = [
	('x', c_float),
	('y', c_float),
	('z', c_float)]

class SimOne_LineCurve_Parameter(Structure):
	_pack_ = 1
	_fields_ = [
	('C0', c_float),
	('C1', c_float),
	('C2', c_float),
	('C3', c_float),
	('firstPoints', SimOneData_Vec3f),
	('endPoints', SimOneData_Vec3f),
	('length', c_float)]

SOSM_SENSOR_LANE_OBJECT_SIZE_MAX = 256
SOSM_SENSOR_Boundary_OBJECT_SIZE_MAX = 250
class SimOne_Data_LaneLineInfo(Structure):
	_pack_ = 1
	_fields_ = [
	('lineID', c_int),
	('lineType', ESimOne_Boundary_Type),
	('lineColor', ESimOne_Boundary_Color),
	('linewidth', c_float),
	('linePoints', SimOneData_Vec3f * SOSM_SENSOR_Boundary_OBJECT_SIZE_MAX),
	('linecurveParameter', SimOne_LineCurve_Parameter)]

class SimOne_Data_LaneInfo(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('id', c_int),
		('laneType', ESimOne_Lane_Type),
		('laneLeftID', c_int),
		('laneRightID', c_int),
		('lanePredecessorID', c_int * SOSM_SENSOR_LANE_OBJECT_SIZE_MAX),
		('laneSuccessorID', c_int * SOSM_SENSOR_LANE_OBJECT_SIZE_MAX),
		('l_Line', SimOne_Data_LaneLineInfo),
		('c_Line', SimOne_Data_LaneLineInfo),
		('r_Line', SimOne_Data_LaneLineInfo),
		('ll_Line', SimOne_Data_LaneLineInfo),
		('rr_Line', SimOne_Data_LaneLineInfo)]


SOSM_WAYPOINTS_SIZE_MAX = 100
class SimOne_Data_WayPoints_Entry(Structure):
	_pack_ = 1
	_fields_ = [
	('index', c_int),
	('posX', c_float), # MainVehicle WayPoints X on Opendrive (by meter)
	('posY', c_float),  # MainVehicle WayPoints Y on Opendrive (by meter)
	('heading_x', c_float), # MainVehicle WayPoints heading orientation x in quaternion
	('heading_y', c_float), # MainVehicle WayPoints heading orientation y in quaternion
	('heading_z', c_float), # MainVehicle WayPoints heading orientation z in quaternion
	('heading_w', c_float)] # MainVehicle WayPoints heading orientation w in quaternion

class SimOne_Data_WayPoints(SimOne_Data):
	_pack_ = 1
	_fields_ = [
	('wayPointsSize', c_int),	# MainVehicle WayPoints size
	('wayPoints', SimOne_Data_WayPoints_Entry*SOSM_WAYPOINTS_SIZE_MAX)]  # WayPoints, 300 max

class ESimOne_Driver_Status(c_int):
	ESimOne_Driver_Status_Unknown = 0,
	ESimOne_Driver_Status_Running = 1,
	ESimOne_Driver_Status_Finished = 2

class SimOne_Data_Driver_Status(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('driverStatus', ESimOne_Driver_Status)
	]

class ESimOne_Drive_Mode(c_int):
	ESimOne_Drive_Mode_API = 0,
	ESimOne_Drive_Mode_Driver = 1

class ESimOne_Control_Mode(c_int):
	ESimOne_Control_Mode_Unknown = 0,
	ESimOne_Control_Mode_Auto = 1,
	ESimOne_Control_Mode_Manual = 2
	
class SimOne_Data_Control_Mode(SimOne_Data):
	_pack_ = 1
	_fields_ = [
		('controlMode', ESimOne_Control_Mode)
	]

