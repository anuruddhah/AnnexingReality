#include "stdafx.h"
#include "poseTracker.h"

IcpTracker::IcpTracker() :
_maxIerations(20),
_maxCorrespondenceDistance(0.015f),
_starting(true),
_lostTracking(true),
_pauseThread(false),
_stopThread(false),

_targetSegmentedFrameStream(new ModelSegmentor),
_aligner(new SacPrerejectiveAligner),

_currentFitnessScore(1),

_waitCount(0)
{
	std::cout << "CREATING TRACKER.." << std::endl;
	_icp.setMaximumIterations(_maxIerations);
	_icp.setMaxCorrespondenceDistance(_maxCorrespondenceDistance);

	_qFilter.setBeta(1.0);
	_qFilter.setMinCutoff(0.1);

	_vFilter.setBeta(5.0);
	_vFilter.setMinCutoff(0.5);
}

void IcpTracker::startTracking(){
	ModelSegmentor::SegmentedCloud currentSegmentedCloud;
	AlignResults alignResults;
	pcl::ScopeTime timer("Tracker");
	while (!_stopThread){
		timer.reset();
		//Wait for Initial Aligner when (re)starting the thread
		if (_starting){
			_aligner->condNewResults.wait(std::unique_lock<std::mutex>(std::mutex()), 
				[=](){return _aligner->hasNewResults(); });
			_timer.reset();
		}
		_starting = false;
		if (_currentFitnessScore > RESQUEST_MODEL_THRESHOLD && _aligner->hasNewResults()){
			alignResults = _aligner->getResults();
			_sourceModel = alignResults.output;
			if (!_lostTracking){
				_lostTracking = true;
				_targetSegmentedFrameStream->updateTrackingStatus(true);
			}
			if (!_sourceModel.size()){
				#ifdef VERBOSE
				std::cerr << "Tracker: Empty model" << std::endl;
				#endif
				continue;
			}
			setSources(_sourceModel, alignResults.finalTransformation);
			if (_waitCount < WAIT_TO_INVALIDATE){
				_waitCount++;
			}else{
				_aligner->invalidateResults();
				_waitCount = 0;
			}
			#ifdef VERBOSE
			std::cout << "Tracker: Getting model from Aligner.." << std::endl;
			#endif
		}else{
			_waitCount = 0;
			if (_lostTracking){
				_lostTracking = false;
				_targetSegmentedFrameStream->updateTrackingStatus(false);
			}
			setSources(_currentOutput, _currentTransformation);
		}
		currentSegmentedCloud = _targetSegmentedFrameStream->getCurrentSegmentedCloud();
		if (currentSegmentedCloud.originalPc.size() < MIN_CLOUD_SIZE ||
			currentSegmentedCloud.segmentedPcIndices.size() < MIN_CLOUD_SIZE){
			#ifdef VERBOSE
			std::cerr << "Tracker: No enough points" << std::endl;
			#endif
			if (!_lostTracking){
				_lostTracking = true;
				_targetSegmentedFrameStream->updateTrackingStatus(true);
			}
			continue;
		}
		setTarget(currentSegmentedCloud);
		track();
		#ifdef CHECK_TRACKING_TIME
			std::cout << white << "Tracking Time: " << _timer.getTime() << "ms" << std::endl;
			//std::cout << "---------------- TRACKING ---------------- running time: " << _timer.getTime() << " ms" << std::endl;
		#endif
		_timer.reset();
		if (_pauseThread){
			_currentFitnessScore = 1;
			_waitCount = 0;
			_starting = true;
			_lostTracking = true;
			_getResultsKey.lock(); // Speed is not a concern. Using Mutex directly
			_results.trackingSucceeded = false;
			_getResultsKey.unlock();
			_condPauseRequest.wait(std::unique_lock<std::mutex>(std::mutex()), [=](){return !_pauseThread; });
		}
	}
}

void IcpTracker::setTarget(ModelSegmentor::SegmentedCloud target){
	_targetCloudIndices = target.segmentedPcIndices;
	pcl::copyPointCloud(target.originalPc, target.segmentedPcIndices, _targetCloud);
	_icp.setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(_targetCloud)));
	//pcl::PointCloud<pcl::PointXYZ> pc;
	//pcl::io::loadPCDFile("objects\\source.pcd", pc);
	//_icp.setInputTarget(pc.makeShared());
	//_icp.setIndices(pcl::IndicesConstPtr(new std::vector<int>(_targetCloudIndices)));
}

void IcpTracker::setTargetStream(ModelSegmentor* frameStream){
	_targetSegmentedFrameStream = frameStream;
}

void IcpTracker::setInitAligner(SacPrerejectiveAligner* aligner){
	_aligner = aligner;
}

void IcpTracker::setSources(pcl::PointCloud<pcl::PointXYZ> source, Eigen::Matrix4f transformation){
	_sourceModel = source;
	_currentTransformation = transformation;
	_icp.setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(_sourceModel)));
	//pcl::PointCloud<pcl::PointXYZ> pc;
	//pcl::io::loadPCDFile("objects\\source.pcd", pc);
	//_icp.setInputSource(pc.makeShared());
}

void IcpTracker::track(){
	_icp.align(_currentOutput);
	_currentTransformation = _icp.getFinalTransformation() * _currentTransformation;  // concatenating transformations
	_currentFitnessScore = _icp.getFitnessScore();
	if (_currentFitnessScore <= TRACKER_GOOD_FIT_THRESHOLD){
		updateResults();
	}else{
		#ifdef VERBOSE
		std::cout << "Tracker: Reverting back to previous iteration.." << std::endl;
		#endif
		_currentOutput = _results.output;
		_currentTransformation = _results.finalTransformation;
		std::lock_guard<std::mutex> lock(_getResultsKey);
		_results.fitnessScore = _currentFitnessScore;
		_results.trackingSucceeded = false;
	}
}

void IcpTracker::updateResults(){
	_targetSegmentedFrameStream->updateCenterTransform(_currentTransformation);
	std::lock_guard<std::mutex> lock(_getResultsKey);
	_results.output = _currentOutput;
	_results.fitnessScore = _currentFitnessScore;
	_results.finalTransformation = _currentTransformation;

	Eigen::Quaternionf quat(_currentTransformation.topLeftCorner<3, 3>());
	quat.normalize();
	double rot[4] = { quat.x(), quat.y(), quat.z(), quat.w() };
	const double *rotFilt = _qFilter.filter(_timer.getTimeSeconds(), rot);
	std::copy(rotFilt, rotFilt + 4, _results.quaternion);

	Eigen::Vector4f vTrans;
	pcl::compute3DCentroid(_currentOutput, vTrans);
	double trans[3] = { vTrans.x(), vTrans.y(), vTrans.z() };
	const double *transFilt = _vFilter.filter(_timer.getTimeSeconds(), trans);
	std::copy(transFilt, transFilt + 3, _results.translation);

	_results.trackingSucceeded = true;
}

TrackingResults IcpTracker::getResults(){
	std::lock_guard<std::mutex> lock(_getResultsKey);
	TrackingResults __results = _results;
	return __results;
}

void IcpTracker::pause(){
	_pauseThread = true;
}

void IcpTracker::resume(){
	_pauseThread = false;
	_condPauseRequest.notify_one();
}

void IcpTracker::stop(){
	_stopThread = true;
}
