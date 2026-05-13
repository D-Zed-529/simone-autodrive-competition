from SimOneIOStruct import *

GpsCbFuncType = CFUNCTYPE(c_void_p, c_char_p, POINTER(SimOne_Data_Gps))
ImuCbFuncType = CFUNCTYPE(c_void_p, c_char_p, POINTER(SimOne_Data_IMU))
GroundTruthCbFuncType = CFUNCTYPE(c_void_p, c_char_p, POINTER(SimOne_Data_Obstacle))
SensorLaneInfoCbFuncType = CFUNCTYPE(c_void_p,c_char_p, c_char_p, POINTER(SimOne_Data_LaneInfo))
UltrasonicsCbFuncType = CFUNCTYPE(c_void_p, c_char_p, POINTER(SimOne_Data_UltrasonicRadars))
RadarDetectionCbFuncType = CFUNCTYPE(c_void_p, c_char_p, c_char_p,POINTER(SimOne_Data_RadarDetection))
SensorDetectionsCbFuncType = CFUNCTYPE(c_void_p, c_char_p, c_char_p,POINTER(SimOne_Data_SensorDetections))

SIMONEAPI_GPS_CB = None
SIMONEAPI_IMU_CB = None
SIMONEAPI_GROUNDTRUTH_CB = None
SIMONEAPI_SENSORLANEINFO_CB = None
SIMONEAPI_ULTRASONICS_CB = None
SIMONEAPI_RADARDETECTION_CB = None
SIMONEAPI_SENSOR_DETECTIONS_CB = None

def _simoneapi_gps_cb(mainVehicleId, data):
	global SIMONEAPI_GPS_CB
	SIMONEAPI_GPS_CB(mainVehicleId, data)

def _simoneapi_imu_cb(mainVehicleId, data):
	global SIMONEAPI_IMU_CB
	SIMONEAPI_IMU_CB(mainVehicleId, data)

def _simoneapi_groundtruth_cb(mainVehicleId, data):
	global SIMONEAPI_GROUNDTRUTH_CB
	SIMONEAPI_GROUNDTRUTH_CB(mainVehicleId, data)

def _simoneapi_sensorlaneinfo_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_SENSORLANEINFO_CB
	SIMONEAPI_SENSORLANEINFO_CB(mainVehicleId, sensorId, data)

def _simoneapi_ultrasonics_cb(mainVehicleId, data):
	global SIMONEAPI_ULTRASONICS_CB
	SIMONEAPI_ULTRASONICS_CB(mainVehicleId, data)

def _simoneapi_radardetection_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_RADARDETECTION_CB
	SIMONEAPI_RADARDETECTION_CB(mainVehicleId, sensorId, data)

def _simoneapi_sensor_detections_cb(mainVehicleId, sensorId, data):
	global SIMONEAPI_SENSOR_DETECTIONS_CB
	SIMONEAPI_SENSOR_DETECTIONS_CB(mainVehicleId, sensorId, data)

simoneapi_gps_cb_func = GpsCbFuncType(_simoneapi_gps_cb)
simoneapi_imu_cb_func = ImuCbFuncType(_simoneapi_imu_cb)
simoneapi_groundtruth_cb_func = GroundTruthCbFuncType(_simoneapi_groundtruth_cb)
simoneapi_sensorlaneinfo_cb_func = SensorLaneInfoCbFuncType(_simoneapi_sensorlaneinfo_cb)
simoneapi_ultrasonics_cb_func = UltrasonicsCbFuncType(_simoneapi_ultrasonics_cb)
simoneapi_radardetection_cb_func = RadarDetectionCbFuncType(_simoneapi_radardetection_cb)
simoneapi_sensor_detections_cb_func = SensorDetectionsCbFuncType(_simoneapi_sensor_detections_cb)

def SoGetGps(mainVehicleId, gpsData):
	"""获取主车GPS信息.

	Get main vehicle GPS

	Parameters
	----------
	dmainVehicleI : 
		Id of the main vehicle
	gpsData : 
		GPS data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetGps.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetGps(_mainVehicleId, pointer(gpsData))

def SoApiSetGpsUpdateCB(cb):
	"""主车GPS更新回调.

	Register the callback func applying for GPS info

	Parameters
	----------
	cb : 
		GPS data update callback function

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_GPS_CB
	SimoneAPI.SetGpsUpdateCB(simoneapi_gps_cb_func)
	SIMONEAPI_GPS_CB = cb

def SoGetImu(mainVehicleId, imuData):
	"""获取主车IMU信息.

	Get main vehicle IMU

	Parameters
	----------
	dmainVehicleI : 
		Id of the main vehicle
	imuData : 
		IMU data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetImu.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetImu(_mainVehicleId, pointer(imuData))

def SoApiSetImuUpdateCB(cb):
	"""主车IMU更新回调.

	Register the callback func applying for IMU info

	Parameters
	----------
	cb : 
		IMU data update callback function

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_IMU_CB
	SimoneAPI.SetImuUpdateCB(simoneapi_imu_cb_func)
	SIMONEAPI_IMU_CB = cb

def SoGetGroundTruth(mainVehicleId, obstacleData):
	"""得到仿真场景中的物体的真值.

	Get Ground Truth Data of Objects(Obstacles) from simulation scene

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	obstacleData : 
		Obstacle data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetGroundTruth.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	ret = SimoneAPI.GetGroundTruth(_mainVehicleId, pointer(obstacleData))
	return ret

def SoApiSetGroundTruthUpdateCB(cb):
	"""得到仿真场景中的物体的真值的更新回调.

	Register the callback func applying for obstacle info

	Parameters
	----------
	cb : 
		Obstacle data update callback function

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_GROUNDTRUTH_CB
	SimoneAPI.SetGroundTruthUpdateCB(simoneapi_groundtruth_cb_func)
	SIMONEAPI_GROUNDTRUTH_CB = cb

def SoGetRadarDetections(mainVehicleId, sensorId, detectionData):
	"""得到毫米波雷达目标信息.

	Get millimeter wave radar detections

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	sensorId : 
		Sensor Index
	detectionData : 
		Radar detections

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetRadarDetections.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	return SimoneAPI.GetRadarDetections(_mainVehicleId, _sensorId, pointer(detectionData))

def SoApiSetRadarDetectionsUpdateCB(cb):
	"""毫米波雷达目标信息回调.

	Register the callback func applying for Millimeter wave radar detections

	Parameters
	----------
	cb : 
		Radar detections update callback function

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_RADARDETECTION_CB
	SimoneAPI.SetRadarDetectionsUpdateCB(simoneapi_radardetection_cb_func)
	SIMONEAPI_RADARDETECTION_CB = cb

def SoGetUltrasonicRadar(mainVehicleId, sensorId, ultrasonics):
	"""获得一个超声波雷达信息.

	Get UltrasonicRadar imformations

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	sensorId : 
		Sensor Index
	ultrasonics : 
		ultrasonic data in SimOne_Data_UltrasonicRadar format

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetUltrasonicRadar.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	return SimoneAPI.GetUltrasonicRadar(_mainVehicleId, _sensorId, pointer(ultrasonics))

def SoGetUltrasonicRadars(mainVehicleId, ultrasonics):
	"""获得所有超声波雷达信息.

	Get UltrasonicRadars imfomations

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	ultrasonics : 
		ultrasonics data in SimOne_Data_UltrasonicRadars format

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetUltrasonicRadar.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetUltrasonicRadars(_mainVehicleId, pointer(ultrasonics))

def SoApiSetUltrasonicRadarsCB(cb):
	"""超生波雷达真值信息更新回调.

	Register the callback func applying for ultrasonics radar detections

	Parameters
	----------
	cb : 
		Ultrasonics data update callback function

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_ULTRASONICS_CB
	SimoneAPI.SetUltrasonicRadarsCB(simoneapi_ultrasonics_cb_func)
	SIMONEAPI_ULTRASONICS_CB = cb

def SoGetSensorDetections(mainVehicleId, sensorId, sensorDetections):
	"""获取传感器检测到物体的对应真值.

	Get Ground Truth objects for current sensor, support camera, lidar and perfect perception sensors

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	sensorId : 
		Sensor Index
	sensorDetections : 
		SimOne_Data_SensorDetections data

	Returns
	-------
	bool
		Success or not

	"""
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	SimoneAPI.GetSensorDetections.restype = c_bool
	return SimoneAPI.GetSensorDetections(_mainVehicleId, _sensorId, pointer(sensorDetections))

def SoApiSetSensorDetectionsUpdateCB(cb):
	"""传感器真值信息更新回调.

	Register the callback func applying for GroundTruth of current sensor, support camera, lidar and perfect perception sensors

	Parameters
	----------
	cb : 
		Groundtruth data fetch callback function

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_SENSOR_DETECTIONS_CB
	SimoneAPI.SetSensorDetectionsUpdateCB(simoneapi_sensor_detections_cb_func)
	SIMONEAPI_SENSOR_DETECTIONS_CB = cb

def SoGetSensorConfigurations(mainVehicleId, sensorConfigurations):
	"""得到所有传感器的配置信息（Id、类型、频率、位置和朝向等）.

	Get Sensor's position information

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	sensorConfigurations : 
		SensorConfigurations data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetSensorConfigurations.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetSensorConfigurations(_mainVehicleId, pointer(sensorConfigurations))

def SoGetEnvironment(pEnvironment):
	"""获取当前环境相关信息（天气、光照、地面等）.

	Get current Environment

	Parameters
	----------
	pEnvironment : 
		SimOne_Data_Environment data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetEnvironment.restype = c_bool
	return SimoneAPI.GetEnvironment(pointer(pEnvironment))

def SoSetEnvironment(pEnvironment):
	"""设置当前环境相关信息（天气、光照、地面等）.

	Set Current Environment

	Parameters
	----------
	pEnvironment : 
		SimOne_Data_Environment data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.SetEnvironment.restype = c_bool
	return SimoneAPI.SetEnvironment(pointer(pEnvironment))

def SoGetTrafficLights(mainVehicleId, opendriveLightId, trafficLight):
	"""得到仿真场景中的交通灯的真值.

	Get traffic lights Data of Objects(light) from simulation scene

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	opendriveLightId : 
		traffic light Id on opendrive
	trafficLight : 
		light data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetTrafficLight.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetTrafficLight(_mainVehicleId, opendriveLightId, pointer(trafficLight))

def SoGetSensorLaneInfo(mainVehicleId, sensorId,pLaneInfo):
	"""获取传感器检测到车道与车道线数据.

	Get LaneInfo for current sensor, support camera and fusion sensor

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	sensorId : 
		Sensor Index
	pLaneInfo : 
		SimOne_Data_LaneInfo data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetSensorLaneInfo.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	_sensorId = create_string_buffer(sensorId.encode(), 256)
	return SimoneAPI.GetSensorLaneInfo(_mainVehicleId, _sensorId, pointer(pLaneInfo))

def SoApiSetSensorLaneInfoCB(cb):
	"""获取传感器检测到车道与车道线数据回调.

	Register the callback func applying for LaneInfo from current sensor, support camera, and fusion sensors

	Parameters
	----------
	cb : 
		Groundtruth data fetch callback function

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_SENSORLANEINFO_CB
	SimoneAPI.SetSensorLaneInfoCB(simoneapi_sensorlaneinfo_cb_func)
	SIMONEAPI_SENSORLANEINFO_CB = cb

