from SimOneIOStruct import *


SimOne_StartCaseFuncType = CFUNCTYPE(c_void_p)
SimOne_StopCaseFuncType = CFUNCTYPE(c_void_p)
SimOne_MainVehicleStatusUpdateFuncType = CFUNCTYPE(c_void_p, POINTER(SimOne_Data_MainVehicle_Status))
SimOne_FrameStartFuncType = CFUNCTYPE(c_void_p, c_int)
SimOne_FrameEndFuncType = CFUNCTYPE(c_void_p, c_int)

G_API_StartCase_CB = None
G_API_StopCase_CB = None
G_API_MainVehicleChangeStatusCB =None
G_API_FrameStart_CB = None
G_API_FrameStop_CB = None

def _api_startcase_cb():
	global G_API_StartCase_CB
	if G_API_StartCase_CB is None:
		return
	G_API_StartCase_CB()

def _api_stopcase_cb():
	global G_API_StopCase_CB
	if G_API_StopCase_CB is None:
		return
	G_API_StopCase_CB()

def _api_mainvehiclestatusupdate_cb(mainVehicleId, data):
	global G_API_MainVehicleChangeStatusCB
	if G_API_MainVehicleChangeStatusCB is None:
		return
	G_API_MainVehicleChangeStatusCB(mainVehicleId, data)

def _api_framestart_cb(frame):
	global G_API_FrameStart_CB
	if G_API_FrameStart_CB is None:
		return
	G_API_FrameStart_CB(frame)

def _api_framestop_cb(frame):
	global G_API_FrameStop_CB
	if G_API_FrameStop_CB is None:
		return
	G_API_FrameStop_CB(frame)

api_startcase_cb = SimOne_StartCaseFuncType(_api_startcase_cb)
api_stopcase_cb = SimOne_StopCaseFuncType(_api_stopcase_cb)
api_mainvehiclestatusupdate_cb = SimOne_MainVehicleStatusUpdateFuncType(_api_mainvehiclestatusupdate_cb)
api_framestart_cb = SimOne_FrameStartFuncType(_api_framestart_cb)
api_framestop_cb = SimOne_FrameEndFuncType(_api_framestop_cb)

def SoAPIGetVersion():
	"""获取当前库的版本号.

	Get the version number of the current library

        Parameters
        ----------
	None

        Returns
        -------
	c_char_p
            version number

	"""
	SimoneAPI.GetVersion.restype = c_char_p
	return SimoneAPI.GetVersion()

def SoSetLogOut(logLevel,*args):
	"""日志设置接口.

	Set log interface

	Parameters
	----------
	logLevel : 
		warning,error,info flag:true/false
	args : 
		output format

	Returns
	-------
	bool
		Success or not

	"""
	print(logLevel)
	list = ""
	for arg in args:
		list+=arg
	logStr = bytes(list,encoding='utf-8')
	return SimoneAPI.SetLogOut(logLevel,logStr)

def SoInitSimOneAPI(mainVehicleId='0',isFrameSync = 0,serverIP = '127.0.0.1',port=23789,startcase = 0, stopcase= 0,registerNodeId=0):
	"""初始化SimOne API.

	Initialize SimOneAPI for autonomous driving algorithm

	Parameters
	----------
	mainVehicleId : 
		host vehicle ID(from 0 to 9)
	isFrameSync : 
		synchronize frame or not
	serverIP : 
		BridgeIO server ip
	port : 
		BridgeIO server port
	startcase : 
		callback func which being called before case start
	stopcase : 
		callback func which being called after case end
	registerNodeId : 
		not in use

	Returns
	-------
	None

	"""
	_input = create_string_buffer(serverIP.encode(), 256)
	_mainVehicleId = create_string_buffer(mainVehicleId.encode(), 256)
	global G_API_StartCase_CB
	global G_API_StopCase_CB
	if startcase == 0:
		startcase = None
	if stopcase == 0:
		stopcase = None
	G_API_StartCase_CB = startcase
	G_API_StopCase_CB = stopcase
	ret = SimoneAPI.InitSimOneAPI(_mainVehicleId,isFrameSync,_input,port,startcase,stopcase,registerNodeId)
	return ret

def SoTerminateSimOneAPI():
	"""退出API node.

	Stop SimOne API node

	Parameters
	----------
	None

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.TerminateSimOneAPI.restype = c_bool
	return SimoneAPI.TerminateSimOneAPI()

def SoAPIGetCaseInfo(data):
	"""获取案例详情.

	Get case information

	Parameters
	----------
	data : 
		caseName,caseId,taskId,sessionId

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetCaseInfo.restype = c_bool
	return SimoneAPI.GetCaseInfo(pointer(data))

def SoGetCaseRunStatus():
	"""获取案例运行情况（运行中，停止）.

	Get case running status

	Parameters
	----------
	None

	Returns
	-------
	bool
		Stop,Running

	"""
	SimoneAPI.GetCaseRunStatus.restype = c_int
	return SimoneAPI.GetCaseRunStatus()

def SoGetMainVehicleList(data):
	"""获取主车信息列表，只需要获取一次.

	Get the main vehicle information list

	Parameters
	----------
	data : 
		mainvehicle id/num/type data(output)

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetMainVehicleList.restype = c_bool
	return SimoneAPI.GetMainVehicleList(pointer(data))

def SoAPIWait():
	"""获取当前帧值.

	Get the current frame value

	Parameters
	----------
	None

	Returns
	-------
	bool
		frame value

	"""
	SimoneAPI.Wait.restype = c_int
	return SimoneAPI.Wait()

def SoAPINextFrame(frame):
	"""进行下一帧.

	Go to the next frame

	Parameters
	----------
	frame : 
		current frame value

	Returns
	-------
	bool
		None

	"""
	SimoneAPI.NextFrame.restype = c_void_p
	return SimoneAPI.NextFrame(frame)

def SoAPISetFrameCB(startcb, stopcb):
	"""仿真场景中每帧的回调,每帧开始和结束时调用回调函数.

	Register the callback func which being called at the beginning and end of each frame

	Parameters
	----------
	startcb : 
		callback func which being called at the beginning of the frame
	stopcb : 
		callback func which being called at the end of the frame

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.SetFrameCB.restype = c_bool
	global G_API_FrameStart_CB
	global G_API_FrameStop_CB
	if startcb == 0:
		startcb = None
	if stopcb == 0:
		stopcb = None
	G_API_FrameStart_CB = startcb
	G_API_FrameStop_CB = stopcb
	ret = SimoneAPI.SetFrameCB(api_framestart_cb, api_framestop_cb)
	return ret

def SoGetMainVehicleStatus(mainVehicleId, data):
	"""获取主车状态信息.

	Get the status information of the mainvehicle

	Parameters
	----------
	mainVehicleId : 
		Id of the main vehicle
	data : 
		status data which applied for

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetMainVehicleStatus.restype = c_bool
	return SimoneAPI.GetMainVehicleStatus(mainVehicleId, pointer(data))

def SoAPISetMainVehicleStatusUpdateCB(cb):
	"""获取主车状态信息回调.

	Register the callback func applying for status info of the mainvehicle

	Parameters
	----------
	cb : 
		callback func applying for status info of the mainvehicle

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.SetMainVehicleStatusUpdateCB.restype = c_bool
	global G_API_MainVehicleChangeStatusCB
	if cb == 0:
		cb = None
	G_API_MainVehicleChangeStatusCB = cb
	return SimoneAPI.SetMainVehicleStatusUpdateCB(api_mainvehiclestatusupdate_cb)

def SoGetHDMapData(hdMap):
	"""获取高精度地图标识.

	Get the hdmap data which is designated by configuring on SimOne web app

	Parameters
	----------
	hdMap : 
		SimOne_Data_Map data

	Returns
	-------
	bool
		Success or not

	"""
	SimoneAPI.GetHDMapData.restype = c_bool
	return SimoneAPI.GetHDMapData(pointer(hdMap))
