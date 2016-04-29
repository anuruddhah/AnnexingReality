#include "stdafx.h"
#include "initialAligner.h"

#if 0
SacInitialAligner::SacInitialAligner():
_minSampleDistance(0.05f),
_maxCorrespondenceDistance(0.01f*0.01f),
_maxIerations(500),

_targetProcessedFrameStream(new FeatureCalculator)
{
	_sacia.setMinSampleDistance(_minSampleDistance);
	_sacia.setMaxCorrespondenceDistance(_maxCorrespondenceDistance);
	_sacia.setMaximumIterations(_maxIerations);
}

void SacInitialAligner::startAligning(){
	// Start counting
	pcl::ScopeTime timer("Initial Aligner");
	while (true){
		timer.reset();
		_targetCloud = _targetProcessedFrameStream->getCurrentFeatureCloud();
		if (_targetCloud.originalPc.size() < 100){
			std::cerr << "Empty sample" << std::endl;
			continue;
		}
		setTarget(_targetCloud);
		align();
		#ifdef CHECK_ALIGNMENT_TIME
				std::cout << "SacInitialAligner Time: " << timer.getTime() << "ms" << std::endl;
		#endif
	}
}

void SacInitialAligner::setTarget(FeatureCalculator::FeatureCloud target){
	_targetCloud = target;
	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::copyPointCloud(target.originalPc, target.sampledPcIndices, pc);
	_sacia.setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(pc)));
	_sacia.setTargetFeatures(pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>(_targetCloud.localFeatures)));
}

void SacInitialAligner::setTargetStream(FeatureCalculator* frameStream){
	_targetProcessedFrameStream = frameStream;
}

void SacInitialAligner::setSources(FeatureCalculator::FeatureCloud source){
	_sourceModel = source;
	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::copyPointCloud(source.originalPc, source.sampledPcIndices, pc);
	_sacia.setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(pc)));
	_sacia.setSourceFeatures(pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>(_sourceModel.localFeatures)));
}

void SacInitialAligner::align(){
	_sacia.align(_output);
	updateResults();
}

void SacInitialAligner::updateResults(){
	std::lock_guard<std::mutex> lock(key);
	_results.output = _output;
	_results.fitnessScore = _sacia.getFitnessScore(_maxCorrespondenceDistance);
	_results.finalTransformation = _sacia.getFinalTransformation();
}

AlignResults SacInitialAligner::getResults(){
	std::lock_guard<std::mutex> lock(key);
	AlignResults __results = _results;
	return __results;
}
#endif

SacPrerejectiveAligner::SacPrerejectiveAligner() :
_maxIterations(50000),
_numOfSamples(3),
_correspondenceRandomness(5),
_similarityThreshold(0.9f),
_maxCorrespondenceDistance(0.01f),
_inlierFraction(0.7f),
_waitCount(0),

_newResults(false),
_pauseThread(false),
_stopThread(false),

_targetProcessedFrameStream(new FeatureCalculator)
{
	std::cout << "CREATING INITIAL ALIGNER.." << std::endl;
	_sacpr.setMaximumIterations(_maxIterations);
	_sacpr.setNumberOfSamples(_numOfSamples);
	_sacpr.setCorrespondenceRandomness(_correspondenceRandomness);
	_sacpr.setSimilarityThreshold(_similarityThreshold);
	_sacpr.setMaxCorrespondenceDistance(_maxCorrespondenceDistance);
	_sacpr.setInlierFraction(_inlierFraction);
}

void SacPrerejectiveAligner::startAligning(){
	FeatureCalculator::FeatureCloud currentFeatureCloud;
	// Start counting
	pcl::ScopeTime timer("Initial Aligner");
	while (!_stopThread){
		if (_pauseThread){
			_waitCount = 0;
			_newResults = false;
			_condPauseRequest.wait(std::unique_lock<std::mutex>(std::mutex()), [=](){return !_pauseThread; });
		}
		timer.reset();
		currentFeatureCloud = _targetProcessedFrameStream->getCurrentFeatureCloud();
		if (currentFeatureCloud.originalPc.size() < MIN_NUM_OF_INTEREST_POINTS || 
			currentFeatureCloud.sampledPcIndices.size() < MIN_NUM_OF_INTEREST_POINTS ||
			currentFeatureCloud.localFeatures.size() < MIN_NUM_OF_INTEREST_POINTS){
			#ifdef VERBOSE
			std::cerr << "Init Aligner: No enough interest points" << std::endl;
			#endif
			continue;
		}
		std::lock_guard<std::mutex> lock(_setDataKey);
		setTarget(currentFeatureCloud);
		align();
		#ifdef CHECK_ALIGNMENT_TIME
			std::cout << yellow << "SacPrerejectiveAligner Time: " << timer.getTime() << "ms" << white << std::endl;
			//std::cout << "|||||||||||||||| INIT ALIGNER |||||||||||||||| running time: " << timer.getTime() << " ms" << std::endl;
		#endif
	}
}

void SacPrerejectiveAligner::setTarget(FeatureCalculator::FeatureCloud target){
	_targetScene = target;
	//pcl::copyPointCloud(target.originalPc, target.sampledPcIndices, _targetCloud);
	_sacpr.setInputTarget(_targetScene.keyPoints.makeShared());
	_sacpr.setTargetFeatures(_targetScene.localFeatures.makeShared());
}

void SacPrerejectiveAligner::setTargetStream(FeatureCalculator* frameStream){
	_targetProcessedFrameStream = frameStream;
}

void SacPrerejectiveAligner::setSources(FeatureCalculator::FeatureCloud source){
	std::lock_guard<std::mutex> lock(_setDataKey);
	_sourceModel = source;
	//pcl::copyPointCloud(source.originalPc, source.sampledPcIndices, _sourceCloud);
	_sacpr.setInputSource(_sourceModel.keyPoints.makeShared());
	_sacpr.setSourceFeatures(_sourceModel.localFeatures.makeShared());
}

void SacPrerejectiveAligner::align(){
	_sacpr.align(_output);
	if (_sacpr.getFitnessScore() <= MIN_GOOD_FIT_ALIGNER){
		updateResults();
		_waitCount = 0;
	}else if(_waitCount < WAIT_TO_INVALIDATE){
		_waitCount++;
	}else{
		invalidateResults();
		_waitCount = 0;
	}
}

void SacPrerejectiveAligner::updateResults(){
	std::lock_guard<std::mutex> lock1(_getResultsKey);
	_results.output = _output;
	_results.fitnessScore = _sacpr.getFitnessScore(_maxCorrespondenceDistance);
	_results.finalTransformation = _sacpr.getFinalTransformation();
	std::lock_guard<std::mutex> lock2(_checkCenterKey);
	_newCenter = true;
	std::lock_guard<std::mutex> lock3(_checkResultsKey);
	_newResults = true;
	condNewResults.notify_all();
}

AlignResults SacPrerejectiveAligner::getResults(){
	std::lock_guard<std::mutex> lock(_getResultsKey);
	AlignResults __results = _results;
	return __results;
}

bool SacPrerejectiveAligner::hasNewCenter(){
	std::lock_guard<std::mutex> lock(_checkCenterKey);
	return _newCenter;
}
void SacPrerejectiveAligner::invalidateCenter(){
	std::lock_guard<std::mutex> lock(_checkCenterKey);
	_newCenter = false;
}

bool SacPrerejectiveAligner::hasNewResults(){
	std::lock_guard<std::mutex> lock(_checkResultsKey);
	return _newResults;
}
void SacPrerejectiveAligner::invalidateResults(){
	std::lock_guard<std::mutex> lock(_checkResultsKey);
	_newResults = false;
}

void SacPrerejectiveAligner::pause(){
	_pauseThread = true;
}

void SacPrerejectiveAligner::resume(){
	_pauseThread = false;
	_condPauseRequest.notify_one();
}

void SacPrerejectiveAligner::stop(){
	_stopThread = true;
}
