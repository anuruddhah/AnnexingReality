#pragma once

#include "stdafx.h"
#include "sensorReader.h"

typedef pcl::FPFHSignature33 LocalFeature;
typedef pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, LocalFeature> FeatureEstimation;

class FeatureCalculator{
public:
	struct FeatureCloud{
		pcl::PointCloud<pcl::PointXYZ> originalPc;
		std::vector<int> sampledPcIndices;
		pcl::PointCloud<pcl::PointXYZ> keyPoints;
		pcl::PointCloud<LocalFeature> localFeatures;
	};

	FeatureCalculator();
	void startProcessing();
	void setInputCloud(pcl::PointCloud<pcl::PointXYZ> pc); // For outside access
	void setInputCloudStream(MySensor* myKinect);
	void filterAndFeatureCalc(); // For outside access
	void featureCalcOnly(); // For outside access, for already filtered and sampled clouds
	void filterOnly();
	void stop();
	FeatureCloud getCurrentFeatureCloud();

private:
	inline void segment();
	inline void filter(); // Do the passthrough when reading, this function is unnecessary
	inline void downSample();
	inline void extractKeyPoints();
	inline void calcNormals();
	inline void calcFeatures();
	inline void updateCurrentProcessedCloud();

	MySensor* _myKinect;
	FeatureCloud _currentFeatureCloud;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr _searchMethod;
	pcl::search::OrganizedNeighbor<pcl::PointXYZ>::Ptr _organizedSearchMethod;

	float _depthLlimit;
	float _sorNumOfNeibours; //
	float _sorStdDevMultThreshold; //
	float _voxelGridSize;
	float _usRadius;
	float _normalRadius;
	float _featureRadius;

	pcl::IndicesPtr _segmentedPcIndicesX; //
	pcl::IndicesPtr _segmentedPcIndicesY; //
	pcl::IndicesConstPtr _segmentedPcIndicesZ; //

	pcl::PassThrough<pcl::PointXYZ> passThrough;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statOutlierRemovel; //
	pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
	pcl::UniformSampling<pcl::PointXYZ> uniformSampling;
	//pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> kpExt;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> integralNormEst;
	FeatureEstimation fpfhEst;

	pcl::PointCloud<pcl::PointXYZ>::Ptr _pc;
	pcl::IndicesPtr _segmentedPcIndices;
	pcl::IndicesPtr _filteredPcIndices;
	pcl::IndicesPtr _sampledPcIndices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _keyPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _sampledPc;

	pcl::PointCloud<pcl::Normal>::Ptr _normals;
	pcl::PointCloud<LocalFeature>::Ptr _localFeatures;

	bool _stopThread;

	std::mutex _getResultsKey; //allowed to be aquired only through update and get methods
};
