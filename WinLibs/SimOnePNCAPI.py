from SimOneIOStruct import *

SimOne_ScenarioEventCBType = CFUNCTYPE(c_void_p, c_char_p, c_char_p, c_char_p)

G_API_ScenarioEvent_CB = None

def _api_scenarioEvent_cb(mainVehicleId, evt, data):
	global G_API_ScenarioEvent_CB
	if G_API_ScenarioEvent_CB is None:
		return
	G_API_ScenarioEvent_CB(mainVehicleId, evt, data)

api_scenarioEvent_cb = SimOne_ScenarioEventCBType(_api_scenarioEvent_cb)

def SoRegisterVehicleState(mainVehicleId, data, size):
	"""注册SimOne_Data_Gps包含的状态以外的主车状态信息.

	Register states of main vehicle dynamics

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	data : 
		array of state names
	size : 
		state number in data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.RegisterVehicleState.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.RegisterVehicleState(_mainVehicleId,pointer(data),size)

def SoGetVehicleState(mainVehicleId,data):
	"""获取通过RegisterSimOneVehicleState注册的主车状态信息.

	Get states of main vehicle dynamics which are registered by RegisterSimOneVehicleState

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	data : 
		states of main vehicle dynamics

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetVehicleState.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetVehicleState(_mainVehicleId,pointer(data))

def SoSetPose(mainVehicleId, poseControl):
	"""设置主车位置.

	Set main vehicle pose

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	poseControl : 
		Pose to set(input)

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.SetPose.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetPose(_mainVehicleId, pointer(poseControl))

def SoSetDrive(mainVehicleId, driveControl):
	"""主车控制.

	Set vehicle drive control

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	driveControl : 
		vehicle control data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.SetDrive.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetDrive(_mainVehicleId, pointer(driveControl))
	
def SoSetDriveTrajectory(mainVehicleId, controlTrajectory):
	"""主车控制.

	Set vehicle drive control by planning trajectory

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	controlTrajectory : 
		vehicle planning trajectory

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.SetDriveTrajectory.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetDriveTrajectory(_mainVehicleId, pointer(controlTrajectory))

def SoSetDriverName(mainVehicleId, driverName):
	"""设置主车控制器的名字.

	Set vehicle driver's name

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	driverName : 
		vehicle driver name, max length is 8

	Returns
	-------
	None

	"""
	SimoneAPI.SetDriverName.restype = c_bool
	_driverName = create_string_buffer(driverName.encode(), 256)
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetDriverName(_mainVehicleId, _driverName)

def SoSetDriveMode(mainVehicleId, driverMode):
	"""设置主车控制模式.

	Set vehicle drive control mode for vehicle dynamics.

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	driverMode : 
		ESimOne_Drive_Mode_API for control signal from API,
		ESimOne_Drive_Mode_Driver for control signal from SimOneDriver.

	Returns
	-------
	None

	"""
	SimoneAPI.SetDriveMode.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetDriveMode(_mainVehicleId, driverMode)

def SoSetVehicleEvent(mainVehicleId, vehicleEventInfo):
	"""设置主车预警消息.

	Set vehicle event information

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	vehicleEventInfo : 
		vehicle event information data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.SetVehicleEvent.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetVehicleEvent(_mainVehicleId, pointer(vehicleEventInfo))

def SoSetSignalLights(mainVehicleId, pSignalLight):
	"""设置车辆信号灯状态.

	Set signal lights

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	pSignalLight : 
		SimOne_Data_Turn_Signal data (output)

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.SetSignalLights.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.SetSignalLights(_mainVehicleId, pointer(pSignalLight))

def SoGetDriverStatus(mainVehicleId, driverStatusData):
	"""获取SimOneDriver运行状态.

	Get SimOneDriver status

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	driverStatusData : 
		SimOneDriver status data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetDriverStatus.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetDriverStatus(_mainVehicleId, pointer(driverStatusData))

def SoGetDriverControl(mainVehicleId, driverControlData):
	"""获取SimOneDriver控制信号.

	Get SimOneDriver drive control

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	driverControlData : 
		vehicle control data from SimOneDriver

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetDriverControl.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetDriverControl(_mainVehicleId, pointer(driverControlData))

def SoGetControlMode(mainVehicleId, controlModeData):
	"""获取车辆控制模式.

	Get vehicle control mode

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	controlModeData : 
		Vehicle control mode

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetControlMode.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetControlMode(_mainVehicleId, pointer(controlModeData))

def SoGetWayPoints(mainVehicleId, wayPointsData):
	"""获取案例主车路径点.

	Get MainVehile WayPoints

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	wayPointsData : 
		MainVehicle WayPoints data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetWayPoints.restype = c_bool
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	return SimoneAPI.GetWayPoints(_mainVehicleId,pointer(wayPointsData))

def SoAPISetScenarioEventCB(cb):
	"""场景事件回调.

	Register the callback func applying for setting scenario event

	Parameters
	----------
	cb : 
		scenario event callback function

	Returns
	-------
	bool
		Success or not

	"""
	if cb == 0:
		cb = None
	global G_API_ScenarioEvent_CB
	G_API_ScenarioEvent_CB = cb
	SimoneAPI.SetScenarioEventCB.restype = c_bool
	ret = SimoneAPI.SetScenarioEventCB(api_scenarioEvent_cb)
	return ret
