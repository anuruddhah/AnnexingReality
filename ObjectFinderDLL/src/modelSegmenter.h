#pragma once

#include "stdafx.h"
#include "sensorReader.h"
#include "initialAligner.h"

class ModelSegmentor{
public:
	struct SegmentedCloud{
		pcl::PointCloud<pcl::PointXYZ> originalPc;
		std::vector<int> segmentedPcIndices;
	};

	ModelSegmentor();
	void startSegmenting();
	inline void setInputCloud(pcl::PointCloud<pcl::PointXYZ> pc); // For outside access remove inline and different one)
	void setInputCloudStream(MySensor* myKinect);
	void setInitAligner(SacPrerejectiveAligner* aligner);
	void setModelDimensions(std::vector<float> dimensions);
	void setModelOrigin(pcl::PointCloud<pcl::PointXYZ> origin);
	inline void segment(); // For outside access remove inline and have a different func
	SegmentedCloud getCurrentSegmentedCloud();
	void updateCenterTransform(Eigen::Matrix4f center);
	void updateTrackingStatus(bool lost);
	void pause();
	void resume();
	void stop();

private:
	inline void filter();
	inline void downSample();
	inline void updateCurrentSegmentedCloud();
	inline void filterAndSegment();

	MySensor* _myKinect;
	SacPrerejectiveAligner* _aligner;
	//pcl::PointCloud<pcl::PointXYZ> _currentSegmentedPc;
	SegmentedCloud _currentSegmentedCloud;

	pcl::PointCloud<pcl::PointXYZ> _origin;
	pcl::PointCloud<pcl::PointXYZ> _originTranformed;
	Eigen::Matrix4f _centerTransformFromTracker;
	Eigen::Vector3f _center;

	bool _lostTracking;

	float _offsetScale;
	float _distX;
	float _distY;
	float _distZ;
	float _radius;
	pcl::PointCloud<pcl::PointXYZ> p;
	float _sorNumOfNeibours;
	float _sorStdDevMultThreshold;
	float _radRadius;
	float _radMinNeibours;
	float _bilateralSigmaR;
	float _bilateralSigmaS;
	float _voxelGridSize;
	float _usRadius;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr _searchMethod;
	pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr _organizedSearchMethod;

	pcl::PassThrough<pcl::PointXYZ> passThroughX;
	pcl::PassThrough<pcl::PointXYZ> passThroughY;
	pcl::PassThrough<pcl::PointXYZ> passThroughZ;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statOutlierRemovel;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> radOutlierRemovel;
	pcl::FastBilateralFilter<pcl::PointXYZ> fastBilateral;
	pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
	pcl::UniformSampling<pcl::PointXYZ> uniformSampling;

	pcl::PointCloud<pcl::PointXYZ>::Ptr _originalPc;
	pcl::IndicesPtr _segmentedPcIndicesX;
	pcl::IndicesPtr _segmentedPcIndicesY;
	pcl::IndicesPtr _segmentedPcIndices;
	pcl::IndicesPtr _filteredPcIndices;
	pcl::IndicesPtr _segmentedSampledPcIndices;

	bool _pauseThread;
	bool _stopThread;
	std::condition_variable _condPauseRequest;

	std::mutex _setDataKey; // aquired only through startSegmenting, setModelDimensions and setModelOrigin methods
	std::mutex _getResultsKey; //aquired only through updateCurrentSegmentedCloud and getCurrentSegmentedCloud methods
	std::mutex _updateCenterKey; //allowed to be aquired only through segment and updateCentrer methods
};
