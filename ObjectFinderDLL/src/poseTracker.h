#pragma once

#include "stdafx.h"
#include "modelSegmenter.h"
#include "initialAligner.h"

#include <pcl\registration\icp_nl.h>


#define MIN_CLOUD_SIZE 100
#define WAIT_TO_INVALIDATE 100
#define TRACKER_GOOD_FIT_THRESHOLD 0.0001
#define RESQUEST_MODEL_THRESHOLD 0.0001

struct TrackingResults{
	TrackingResults() :
		trackingSucceeded(false),
		fitnessScore(1.0),
		finalTransformation(Eigen::Matrix4f::Identity())
	{};
	bool trackingSucceeded;
	float fitnessScore;
	Eigen::Matrix4f finalTransformation;
	float translation[3];
	float quaternion[4];
	pcl::PointCloud<pcl::PointXYZ> output;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class IcpTracker{
public:
	IcpTracker();
	void setTarget(ModelSegmentor::SegmentedCloud target); // For outside access
	void setTargetStream(ModelSegmentor* frameStream);
	void setInitAligner(SacPrerejectiveAligner* aligner);
	void setSources(pcl::PointCloud<pcl::PointXYZ> source, Eigen::Matrix4f transformation); // For outside access
	void addSource(pcl::PointCloud<pcl::PointXYZ> source);
	void startTracking();
	void track();
	inline void updateCenter();
	void updateResults();
	TrackingResults getResults();
	void pause();
	void resume();
	void stop();

private:
	pcl::IterativeClosestPoint <pcl::PointXYZ, pcl::PointXYZ> _icp;
	int _maxIerations;
	float _maxCorrespondenceDistance;
	bool _lostTracking;

	ModelSegmentor* _targetSegmentedFrameStream; // shared pointer?
	SacPrerejectiveAligner* _aligner;
	pcl::PointCloud<pcl::PointXYZ> _targetCloud;
	std::vector<int> _targetCloudIndices;
	pcl::PointCloud<pcl::PointXYZ> _sourceModel;

	pcl::PointCloud<pcl::PointXYZ> _currentOutput;
	Eigen::Matrix4f _currentTransformation;
	float _currentFitnessScore;

	pcl::ScopeTime _timer;
	int _waitCount;

	vrpn_OneEuroFilterQuat _qFilter;
	vrpn_OneEuroFilterVec _vFilter;

	TrackingResults _results;

	bool _starting;
	bool _pauseThread;
	bool _stopThread;
	std::condition_variable _condPauseRequest;

	std::mutex _getResultsKey; //allowed to be aquired only through update and get methods
};
