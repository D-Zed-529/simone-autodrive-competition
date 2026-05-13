from SimOneIOStruct import *

StreamingImageCbFuncType = CFUNCTYPE(c_void_p,POINTER(SimOne_Data_Image))
StreamingPointCloudCbFuncType = CFUNCTYPE(c_void_p,POINTER(SimOne_Data_Point_Cloud))

SIMONEAPI_StreamingPOINTCLOUD_CB = None
SIMONEAPI_StreamingIMAGE_CB = None

def _simoneapi_streamingpointcloud_cb(data):
	global SIMONEAPI_StreamingPOINTCLOUD_CB
	SIMONEAPI_StreamingPOINTCLOUD_CB(data)
	
def _simoneapi_streamingimage_cb(data):
	global SIMONEAPI_StreamingIMAGE_CB
	SIMONEAPI_StreamingIMAGE_CB(data)

simoneapi_streamingpointcloud_cb_func = StreamingPointCloudCbFuncType(_simoneapi_streamingpointcloud_cb)
simoneapi_streamingimage_cb_func = StreamingImageCbFuncType(_simoneapi_streamingimage_cb)

def SoGetStreamingImage(ip, port, imageData):
	"""获取图像数据.

	Get Streaming Image Data

	Parameters
	----------
	ip : 
		UDP Streaming Image ip
	port : 
		UDP Streaming Image port
	imageData : 
		Image data(output)

	Returns
	-------
	bool
		Success or not

	"""
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingAPI.GetStreamingImage.restype = c_bool
	return SimoneStreamingAPI.GetStreamingImage(_input, port, pointer(imageData))

def SoApiSetStreamingImageUpdateCB(ip, port, cb):
	"""获取图像数据回调.

	Streaming Image Data update callback

	Parameters
	----------
	ip : 
		UDP Streaming Image ip
	port : 
		UDP Streaming Image port
	cb : 
		callback func applying for image data 

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_StreamingIMAGE_CB
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingAPI.SetStreamingImageUpdateCB(_input, port, simoneapi_streamingimage_cb_func)
	SIMONEAPI_StreamingIMAGE_CB = cb

def SoGetStreamingPointCloud(ip, port,infoPort, pointCloudData):
	"""获取点云数据.

	Get Streaming PointCloud Data

	Parameters
	----------
	ip : 
		UDP Streaming PointCloud ip
	port : 
		UDP Streaming PointCloud port
	infoPort : 
		UDP Streaming device info port
	pointCloudData : 
		PointCloud data(output)

	Returns
	-------
	bool
		Success or not

	"""
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingAPI.GetStreamingPointCloud.restype = c_bool
	return SimoneStreamingAPI.GetStreamingPointCloud(_input, port, infoPort,pointer(pointCloudData))

def SoApiSetStreamingPointCloudUpdateCB(ip, port,infoPort, cb):
	"""获取点云数据回调.

	Streaming PointCloud Data update callback

	Parameters
	----------
	ip : 
		UDP Streaming PointCloud ip
	port : 
		UDP Streaming PointCloud port
	infoPort : 
		UDP Streaming device info port
	cb : 
		callback func applying for point cloud data 

	Returns
	-------
	bool
		Success or not

	"""
	global SIMONEAPI_StreamingPOINTCLOUD_CB
	_input = create_string_buffer(ip.encode(), 256)
	SimoneStreamingAPI.SetStreamingPointCloudUpdateCB(_input, port,infoPort, simoneapi_streamingpointcloud_cb_func)
	SIMONEAPI_StreamingPOINTCLOUD_CB = cb
