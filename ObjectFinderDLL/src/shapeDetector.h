#pragma once

#include "stdafx.h"
#include "sensorReader.h"
#include "featureCalculator.h"
#include "initialAligner.h"
#include "poseTracker.h"

//#define SHAPEVERBOSE
#define SIMILARITY_THRESHOLD 150

enum PrimShape{
	PLANE,
	SPHERE,
	CYLINDER,
	CONE,
	TORUS
};

struct BaseTypeParams{
	virtual void dummy();
	BaseTypeParams(){}
	float primitivePref;
};

struct PlaneParams : BaseTypeParams{
	PlaneParams(float _width, float _length,
		float _primitivePref = 1.0,
		float _widthPref = 0.3, float _lengthPref = 0.4, float _ratioPref = 0.3) :
		width(_width),
		length(_length),
		widthPref(_widthPref),
		lengthPref(_lengthPref),
		ratioPref(_ratioPref)
	{
		primitivePref = _primitivePref;
	};
	float width;
	float length;

	Eigen::Vector3f center;
	Eigen::Vector3f normal;

	float widthPref;
	float lengthPref;
	float ratioPref;
};

struct SphereParams : BaseTypeParams{
	SphereParams(float _radius,
		float _primitivePref = 1.0,
		float _radiusPref = 1.0) :
		radius(_radius),
		radiusPref(_radiusPref)
	{
		primitivePref = _primitivePref;
	};
	float radius;

	Eigen::Vector3f center;

	float radiusPref;
};

struct CylinderParams : BaseTypeParams{
	CylinderParams(float _radius, float _height,
		float _primitivePref = 1.0,
		float _radiusPref = 0.1, float _heightPref = 0.6, float _ratioPref = 0.3) :
		radius(_radius),
		height(_height),
		radiusPref(_radiusPref),
		heightPref(_heightPref),
		ratioPref(_ratioPref)
	{
		primitivePref = _primitivePref;
	};
	float radius;
	float height;

	Eigen::Vector3f axisPosition;
	Eigen::Vector3f axisDirection;

	float radiusPref;
	float heightPref;
	float ratioPref;
};

struct ConeParams : BaseTypeParams{
	ConeParams(float _angle, float _height,
		float _primitivePref = 1.0,
		float _anglePref = 0.2, float _heightPref = 0.7, float _ratioPref = 0.1) :
		angle(_angle),
		height(_height),
		anglePref(_anglePref),
		heightPref(_heightPref),
		ratioPref(_ratioPref)
	{
		primitivePref = _primitivePref;
	};
	float angle;
	float height;

	Eigen::Vector3f center; // Apex?
	Eigen::Vector3f axisDirection;

	float anglePref;
	float heightPref;
	float ratioPref;
};

struct TorusParams : BaseTypeParams{
	TorusParams(float _minRadius, float _maxRadius,
		float _primitivePref = 1.0,
		float _minRadiusPref = 0.1, float _maxRadiusPref = 0.9, float _ratioPref = 0) :
		minRadius(_minRadius),
		maxRadius(_maxRadius),
		minRadiusPref(_minRadiusPref),
		maxRadiusPref(_maxRadiusPref),
		ratioPref(_ratioPref)
	{
		primitivePref = _primitivePref;
	};
	float minRadius;
	float maxRadius;

	Eigen::Vector3f center;
	Eigen::Vector3f axisDirection;

	float minRadiusPref;
	float maxRadiusPref;
	float ratioPref;
};

struct Primitive{
	Primitive(PrimShape _type, BaseTypeParams *_params) : type(_type), params(_params){}
	PrimShape type;
	BaseTypeParams *params;
};

struct Pose{
	// Translation
	float x;
	float y;
	float z;

	// Rotation as quaternion
	float qX;
	float qY;
	float qZ;
	float qW;
};


struct ShapeBasic{
	ShapeBasic(std::vector<Primitive> _prims) :
		primitives(_prims)
	{};

	std::vector<Primitive> primitives;
};

struct Shape{
	Shape(std::vector<Primitive> _primitives, int _objPref = 100, bool _jump = false) :
		preferredPrimitives(_primitives),
		objectPref(_objPref),
		jumpAllowed(_jump),

		hasNewModel(false),
		tracker(new IcpTracker),
		lostTrackCount(0)
	{};

	void resetModel(){
		hasNewModel = false;
		model = pcl::PointCloud<pcl::PointXYZ>();
		initTrans = Eigen::Matrix4f::Identity();
		rot = Eigen::Matrix4f::Identity();
		objToPrimOrigin = Eigen::Vector4f::Zero();
		matchedPrimitives = std::vector<Primitive>();
		lostTrackCount = 0;
	}

	std::vector<Primitive> preferredPrimitives;
	int objectPref;
	bool jumpAllowed;

	// Updated after detection and matching
	bool hasNewModel;
	pcl::PointCloud<pcl::PointXYZ> model;
	Eigen::Matrix4f initTrans;
	Eigen::Matrix4f rot;
	Eigen::Vector4f objToPrimOrigin;
	std::vector<Primitive> matchedPrimitives;

	std::shared_ptr<IcpTracker> tracker;
	int lostTrackCount;
};



//struct MatchingResults{
//	std::vector<Shape> desiredShapes;
//	std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
//	std::vector<Shape> detectedShapes;
//	std::vector<int> shapeToClusterMapping;
//};


class ShapeDetector{
public:

	ShapeDetector();
	void setInputCloudStream(MySensor* myKinect);
	void setInputCloud(pcl::PointCloud<pcl::PointXYZ> pc); // outside access
	void setShapes(std::vector<Shape> shapes);
	void startMatching();
	std::vector<Shape> getResults();
	int getNoOfDetections();
	void pause();
	void resume();
	void wake();
	void stop();

	std::condition_variable condNewResults;

private:
	inline void cluster();
	inline void detectShapes();
	inline void vote(ShapeBasic shape, int clusterNo);
	inline void match();
	inline void updateresults();
	inline void initVoteMatrix();
	inline double quickAlign(pcl::PointCloud<pcl::PointXYZ> model, pcl::PointCloud<pcl::PointXYZ> cluster);

	//MatchingResults _results;
	std::vector<Shape> _resultShapes; // Replica of _desiredShapes updated with matching info
	bool _resultsFetched;
	bool _resultsFetchedTemp;
	int _noOfDetections;

	bool _allShapesDetected;

	dlib::matrix<int> _voteMatrix;
	int _voteMatrixSize;

	MySensor* _myKinect;
	pcl::PointCloud<pcl::PointXYZ> _pc;

	std::vector<Shape> _desiredShapes;
	std::vector<Shape> _desiredShapesTemp;
	std::vector<pcl::PointCloud<pcl::PointXYZ>> _clusters;
	std::vector<ShapeBasic> _detectedShapes;
	std::vector<int> _shapeToClusterMapping;

	//pcl::visualization::PCLVisualizer* _viewer;

	bool _pauseThread;
	bool _stopThread;
	std::condition_variable _condPauseRequest;

	std::mutex _setShapesKey; // aquired only through setShapes and startMatching methods
	std::mutex _getResultsKey; //aquired only through updateResults and getResults methods
	std::mutex _getNoOfDetectionsKey; //aquired only through getNoOfDetections and match methods

	std::condition_variable _condShapesFound;
	std::mutex _dummyKey; //used by condition variable
};