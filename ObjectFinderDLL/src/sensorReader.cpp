#include "stdafx.h"
#include "sensorReader.h"

#include "pcl\io\png_io.h"

int MySensor::initSesor(){
	std::cout << "INITIALIZING READER.." << std::endl;
	_stopThread = false;
	_sensorInitResult = -1;

#ifdef USE_KINECT
	_hResult = GetDefaultKinectSensor(&_sensor);
	if (FAILED(_hResult)){
		std::cerr << "Error: GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	_hResult = _sensor->Open();
	if (FAILED(_hResult)){
		std::cerr << "Error: IKinectSensor::Open" << std::endl;
		return -1;
	}

	_hResult = _sensor->get_CoordinateMapper(&_mapper);
	if (FAILED(_hResult)){
		std::cerr << "Error: IKinectSensor::get_CoordinateMapper" << std::endl;
		return -1;
	}

	_hResult = _sensor->get_ColorFrameSource(&_colorSource);
	if (FAILED(_hResult)){
		std::cerr << "Error: IKinectSensor::get_ColorFrameSource" << std::endl;
		return -1;
	}

	_hResult = _sensor->get_DepthFrameSource(&_depthSource);
	if (FAILED(_hResult)){
		std::cerr << "Error: IKinectSensor::get_DepthFrameSource" << std::endl;
		return -1;
	}

	_hResult = _colorSource->OpenReader(&_colorReader);
	if (FAILED(_hResult)){
		std::cerr << "Error: IColorFrameSource::OpenReader" << std::endl;
		return -1;
	}

	_hResult = _depthSource->OpenReader(&_depthReader);
	if (FAILED(_hResult)){
		std::cerr << "Error: IDepthFrameSource::OpenReader" << std::endl;
		return -1;
	}

	_hResult = _colorSource->get_FrameDescription(&_colorFrameDescription);
	if (FAILED(_hResult)){
		std::cerr << "Error: IColorFrameSource::get_FrameDescription" << std::endl;
		return -1;
	}

	_hResult = _depthSource->get_FrameDescription(&_depthFrameDescription);
	if (FAILED(_hResult)){
		std::cerr << "Error: IDepthFrameSource::get_FrameDescription" << std::endl;
		return -1;
	}
#else
	g_context = DepthSense::Context::create("localhost");

	g_context.deviceAddedEvent().connect(this, &MySensor::onDeviceConnected);
	g_context.deviceRemovedEvent().connect(this, &MySensor::onDeviceDisconnected);

	// Get the list of currently connected devices
	std::vector<DepthSense::Device> da = g_context.getDevices();

	// We are only interested in the first device
	if (da.size() >= 1){
		g_bDeviceFound = true;

		da[0].nodeAddedEvent().connect(this, &MySensor::onNodeConnected);
		da[0].nodeRemovedEvent().connect(this, &MySensor::onNodeDisconnected);

		std::vector<DepthSense::Node> na = da[0].getNodes();

		printf("Found %u nodes\n", na.size());

		for (int n = 0; n < (int)na.size(); n++)
			configureNode(na[n]);
	}else{
		return -1;
	}
	g_context.startNodes();
#endif

	_sensorInitResult = 0;
	return _sensorInitResult;
};

void MySensor::startReading(){
#ifdef USE_KINECT
	//int colorHeight, colorWidth;
	//_colorFrameDescription->get_Height(&colorHeight);
	//_colorFrameDescription->get_Width(&colorWidth);
	//IColorFrame* colorFrame;
	//RGBQUAD* colorBuffer = new RGBQUAD[colorWidth * colorHeight];
	//ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };

	int depthHeight, depthWidth;
	_depthFrameDescription->get_Height(&depthHeight);
	_depthFrameDescription->get_Width(&depthWidth);
	IDepthFrame* depthFrame;
	UINT16* depthBuffer = new UINT16[depthWidth * depthHeight];
	UINT16* depthBufferPointer;
	UINT16 depth;
	DepthSpacePoint depthSpacePoint = { 0.0f, 0.0f };

	CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };

	pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
	pointCloud.resize(depthHeight*depthWidth);
	pointCloud.is_dense = false;
	pointCloud.width = static_cast<uint32_t>(depthWidth);
	pointCloud.height = static_cast<uint32_t>(depthHeight);

	int pcPointer;

	pcl::PointXYZRGB point;

	// Start counting
	pcl::ScopeTime timer("Reading");

	while (!_stopThread){
		timer.reset();

		//// Read color frame
		//_hResult = _colorReader->AcquireLatestFrame(&colorFrame);
		//if (SUCCEEDED(_hResult)){
		//	_hResult = colorFrame->CopyConvertedFrameDataToArray(colorWidth * colorHeight * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(colorBuffer), ColorImageFormat::ColorImageFormat_Bgra);
		//	if (FAILED(_hResult)){
		//		std::cerr << "Error: IColorFrame::CopyConvertedFrameDataToArray" << std::endl;
		//	}
		//}
		//SafeRelease(colorFrame);

		// Read depth frame
		_hResult = _depthReader->AcquireLatestFrame(&depthFrame);
		if (SUCCEEDED(_hResult)){
			_hResult = depthFrame->CopyFrameDataToArray(depthWidth * depthHeight, depthBuffer);
			if (FAILED(_hResult)){
				std::cerr << "Error: IDepthFrame::CopyFrameDataToArray" << std::endl;
			}
		}
		SafeRelease(depthFrame);

		depthBufferPointer = depthBuffer;
		pcPointer = 0;

		// Fill point cloud
		for (int y = 0; y < depthHeight; y++){
			for (int x = 0; x < depthWidth; x++){
				depthSpacePoint.X = x;
				depthSpacePoint.Y = y;
				depth = *depthBufferPointer;

				if (0 < depth){
					_mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				}
				else{
					cameraSpacePoint.X = NAN;
					cameraSpacePoint.Y = NAN;
					cameraSpacePoint.Z = NAN;
				}

				//_mapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				//int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				//int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
				//if (colorX >= 0 && colorX < colorWidth && colorY >= 0 && colorY < colorHeight){
				//	RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
				//  
				//} else{
				//
				//}

				//if (_finitef(cameraSpacePoint.X) && _finitef(cameraSpacePoint.Y) && _finitef(cameraSpacePoint.Z)
				//	&&  -FLT_MAX <= cameraSpacePoint.X && cameraSpacePoint.X <= FLT_MAX
				//	&&  -FLT_MAX <= cameraSpacePoint.Y && cameraSpacePoint.Y <= FLT_MAX
				//	&&  -FLT_MAX <= cameraSpacePoint.Z && cameraSpacePoint.Z <= FLT_MAX){
					point.r = 0.0f;
					point.g = 0.0f;
					point.b = 0.0f;
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
				//}
				//else{
				//	point.x = 0.0f;
				//	point.y = 0.0f;
				//	point.z = 0.0;
				//}
			
				pointCloud[pcPointer] = point;

				depthBufferPointer++;
				pcPointer++;
			}
		}

		updatePC(pointCloud);

		#ifdef CHECK_READING_TIME
			std::cout << red << "Reading Time: " << timer.getTime() << "ms" << white << std::endl;
		#endif
	}

	//delete colorBuffer;
	delete depthBuffer;
	//delete depthBufferPointer;
	SafeRelease(_depthFrameDescription);
	SafeRelease(_colorFrameDescription);
	SafeRelease(_depthReader);
	SafeRelease(_colorReader);
	SafeRelease(_depthSource);
	SafeRelease(_colorSource);
	SafeRelease(_mapper);
	if (_sensor){
		_sensor->Close();
	}
	SafeRelease(_sensor);
#else
	g_context.run();

	g_context.stopNodes();

	if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
	if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
	if (g_anode.isSet()) g_context.unregisterNode(g_anode);
#endif
}

inline void MySensor::updatePC(pcl::PointCloud<pcl::PointXYZRGB> pc){
	std::lock_guard<std::mutex> lock(_key);
	_currentPC = pc;
}

pcl::PointCloud<pcl::PointXYZRGB> MySensor::getCurrentPC(){
	std::lock_guard<std::mutex> lock(_key);
	return std::ref(_currentPC);
}

void MySensor::stop(){
	_stopThread = true;
}

// DepthSense helper functions

void MySensor::onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data){
	int depthWidth, depthHeight;
	DepthSense::FrameFormat_toResolution(data.captureConfiguration.frameFormat, &depthWidth, &depthHeight);

	pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
	pointCloud.resize(depthHeight*depthWidth);
	pointCloud.is_dense = false;
	pointCloud.width = static_cast<uint32_t>(depthWidth);
	pointCloud.height = static_cast<uint32_t>(depthHeight);

	DepthSense::FPVertex p3DPoint;
	pcl::PointXYZRGB point;

	int pcPointer = 0;

	for (int y = 0; y < depthHeight; y++){
		for (int x = 0; x < depthWidth; x++){
			p3DPoint = data.verticesFloatingPoint[pcPointer];

			if (p3DPoint.z <= 0.0){
				p3DPoint = DepthSense::FPVertex(NAN, NAN, NAN);
			}

			point.r = 0.0f;
			point.g = 0.0f;
			point.b = 0.0f;
			point.x = p3DPoint.x;
			point.y = p3DPoint.y;
			point.z = p3DPoint.z;
			pointCloud[pcPointer] = point;
			pcPointer++;
		}
	}

	//pcl::io::savePNGFile("model.png", pointCloud, "z");

	//FILE *outfile = fopen("image.pgm", "w");
	//fprintf(outfile, "P2\n%d %d\n%d\n", depthWidth, depthHeight, 31999);
	//for (int y = 0; y < depthHeight; y++){
	//	for (int x = 0; x < depthWidth; x++){
	//		fprintf(outfile, "%d ", data.depthMap[y*depthWidth+x]);
	//	}
	//	fprintf(outfile, "\n");
	//}
	//std::this_thread::sleep_for(std::chrono::seconds(10));

	updatePC(pointCloud);
}

void MySensor::configureDepthNode(){
	g_dnode.newSampleReceivedEvent().connect(this, &MySensor::onNewDepthSample);

	DepthSense::DepthNode::Configuration config = g_dnode.getConfiguration();
	config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
	config.framerate = 25;
	config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
	config.saturation = true;

	g_dnode.setEnableVertices(true);
	g_dnode.setEnableVerticesFloatingPoint(true);
	g_dnode.setEnableDepthMap(true);
	
	if (!g_dnode.confidenceThresholdIsReadOnly()){
		g_dnode.setConfidenceThreshold(100);
	}

	try
	{
		g_context.requestControl(g_dnode, 0);

		g_dnode.setConfiguration(config);
	}
	catch (DepthSense::ArgumentException& e)
	{
		printf("Argument Exception: %s\n", e.what());
	}
	catch (DepthSense::UnauthorizedAccessException& e)
	{
		printf("Unauthorized Access Exception: %s\n", e.what());
	}
	catch (DepthSense::IOException& e)
	{
		printf("IO Exception: %s\n", e.what());
	}
	catch (DepthSense::InvalidOperationException& e)
	{
		printf("Invalid Operation Exception: %s\n", e.what());
	}
	catch (DepthSense::ConfigurationException& e)
	{
		printf("Configuration Exception: %s\n", e.what());
	}
	catch (DepthSense::StreamingException& e)
	{
		printf("Streaming Exception: %s\n", e.what());
	}
	catch (DepthSense::TimeoutException&)
	{
		printf("TimeoutException\n");
	}

}

void MySensor::configureNode(DepthSense::Node node){
	if ((node.is<DepthSense::DepthNode>()) && (!g_dnode.isSet()))
	{
		g_dnode = node.as<DepthSense::DepthNode>();
		configureDepthNode();
		g_context.registerNode(node);
	}

	if ((node.is<DepthSense::ColorNode>()) && (!g_cnode.isSet()))
	{
		g_cnode = node.as<DepthSense::ColorNode>();
		//configureColorNode();
		g_context.registerNode(node);
	}

	if ((node.is<DepthSense::AudioNode>()) && (!g_anode.isSet()))
	{
		g_anode = node.as<DepthSense::AudioNode>();
		//configureAudioNode();
		g_context.registerNode(node);
	}
}

void MySensor::onNodeConnected(DepthSense::Device device, DepthSense::Device::NodeAddedData data){
	configureNode(data.node);
}

void MySensor::onNodeDisconnected(DepthSense::Device device, DepthSense::Device::NodeRemovedData data){
	if (data.node.is<DepthSense::AudioNode>() && (data.node.as<DepthSense::AudioNode>() == g_anode))
		g_anode.unset();
	if (data.node.is<DepthSense::ColorNode>() && (data.node.as<DepthSense::ColorNode>() == g_cnode))
		g_cnode.unset();
	if (data.node.is<DepthSense::DepthNode>() && (data.node.as<DepthSense::DepthNode>() == g_dnode))
		g_dnode.unset();
	printf("Node disconnected\n");
}

void MySensor::onDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data){
	if (!g_bDeviceFound)
	{
		data.device.nodeAddedEvent().connect(this, &MySensor::onNodeConnected);
		data.device.nodeRemovedEvent().connect(this, &MySensor::onNodeDisconnected);
		g_bDeviceFound = true;
	}
}

void MySensor::onDeviceDisconnected(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data){
	g_bDeviceFound = false;
	printf("Device disconnected\n");
}

//int main(){
//#pragma omp parallel
//	std::cout << "Hello" << std::endl;
//
//	return 0;
//}