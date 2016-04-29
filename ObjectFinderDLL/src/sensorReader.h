#pragma once

#include "stdafx.h"

#define USE_KINECT

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class MySensor{
public:
	MySensor(){ _sensorInitResult = -1; g_bDeviceFound = false;};
	int initSesor();
	void startReading();
	pcl::PointCloud<pcl::PointXYZRGB> getCurrentPC();
	void stop();

private:
	inline void updatePC(pcl::PointCloud<pcl::PointXYZRGB> pc);

	void onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);
	void configureDepthNode();
	void configureNode(DepthSense::Node node);
	void onNodeConnected(DepthSense::Device device, DepthSense::Device::NodeAddedData data);
	void onNodeDisconnected(DepthSense::Device device, DepthSense::Device::NodeRemovedData data);
	void onDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data);
	void onDeviceDisconnected(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data);
	
	pcl::PointCloud<pcl::PointXYZRGB>_currentPC;

	IKinectSensor* _sensor;
	ICoordinateMapper* _mapper;
	IColorFrameSource* _colorSource;
	IDepthFrameSource* _depthSource;
	IColorFrameReader* _colorReader;
	IDepthFrameReader* _depthReader;
	IFrameDescription* _colorFrameDescription;
	IFrameDescription* _depthFrameDescription;

	DepthSense::Context g_context;
	DepthSense::DepthNode g_dnode;
	DepthSense::ColorNode g_cnode;
	DepthSense::AudioNode g_anode;

	bool g_bDeviceFound;

	bool _stopThread;

	HRESULT _hResult;
	int _sensorInitResult;
	std::mutex _key; //allowed to be aquired only through update and get methods
};
