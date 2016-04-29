#pragma once

#include "stdafx.h"
#include "featureCalculator.h"

#define WAIT_TO_INVALIDATE 5
#define MIN_NUM_OF_INTEREST_POINTS 20
#define MIN_GOOD_FIT_ALIGNER 0.0003

struct AlignResults{
	float fitnessScore;
	Eigen::Matrix4f finalTransformation;
	pcl::PointCloud<pcl::PointXYZ> output;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SacInitialAligner{
public:
	SacInitialAligner();
	void setTarget(FeatureCalculator::FeatureCloud target); // For outside access
	void setTargetStream(FeatureCalculator* frameStream);
	void setSources(FeatureCalculator::FeatureCloud source); // Vector of models later
	void addSource(FeatureCalculator::FeatureCloud source);
	void startAligning();
	void align();
	void updateResults();
	AlignResults getResults();
	bool hasNewResults();
	void invalidateResults();

private:
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> _sacia;
	float _minSampleDistance;
	float _maxCorrespondenceDistance;
	int _maxIerations;

	FeatureCalculator* _targetProcessedFrameStream; // shared pointer?
	FeatureCalculator::FeatureCloud _targetCloud; // For outside access
	FeatureCalculator::FeatureCloud _sourceModel; // Vector of models later
	pcl::PointCloud<pcl::PointXYZ> _output;

	AlignResults _results; // Optimize ; pointer

	std::mutex key; //allowed to be aquired only through update and get methods
};

class SacPrerejectiveAligner{
public:
	SacPrerejectiveAligner();
	void setTarget(FeatureCalculator::FeatureCloud target);
	void setTargetStream(FeatureCalculator* frameStream);
	void setSources(FeatureCalculator::FeatureCloud source);
	void addSource(FeatureCalculator::FeatureCloud source);
	void startAligning();
	void align();
	void updateResults();
	AlignResults getResults();
	bool hasNewCenter();
	void invalidateCenter();
	bool hasNewResults();
	void invalidateResults();
	void pause();
	void resume();
	void stop();

	std::condition_variable condNewResults;

private:
	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, LocalFeature> _sacpr;
	int _maxIterations;
	int	_numOfSamples;
	int	_correspondenceRandomness;
	float _similarityThreshold;
	float _maxCorrespondenceDistance;
	float _inlierFraction;
	int _waitCount;

	pcl::PointCloud<pcl::PointXYZ> pc;

	FeatureCalculator* _targetProcessedFrameStream;
	FeatureCalculator::FeatureCloud _targetScene;
	pcl::PointCloud<pcl::PointXYZ> _targetCloud;
	FeatureCalculator::FeatureCloud _sourceModel;
	pcl::PointCloud<pcl::PointXYZ> _sourceCloud;
	pcl::PointCloud<pcl::PointXYZ> _output;

	AlignResults _results;
	bool _newCenter;
	bool _newResults;

	bool _pauseThread;
	bool _stopThread;
	std::condition_variable _condPauseRequest;

	std::mutex _setDataKey; // aquired only through startAligning and setSources methods
	std::mutex _getResultsKey; //aquired only through updateResults and getResults methods
	std::mutex _checkResultsKey; //aquired only through updateResults, hasNewResults and invalidateResults methods
	std::mutex _checkCenterKey; //aquired only through updateResults, hasNewResults and invalidateResults methods
};