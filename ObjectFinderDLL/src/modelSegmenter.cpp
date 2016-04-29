#include "stdafx.h"
#include "modelSegmenter.h"

ModelSegmentor::ModelSegmentor() : 
_offsetScale(1.2f),
_sorNumOfNeibours(50),
_sorStdDevMultThreshold(0.5f),
_radRadius(0.01),
_radMinNeibours(2),
_bilateralSigmaR(0.03),
_bilateralSigmaS(5.0),
_voxelGridSize(0.005f),
_usRadius(0.005f),
_lostTracking(true),
_pauseThread(false),
_stopThread(false),

_myKinect(new MySensor),
_aligner(new SacPrerejectiveAligner),
_searchMethod(new pcl::search::KdTree<pcl::PointXYZ>),
_organizedSearchMethod(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>),

_originalPc(new pcl::PointCloud<pcl::PointXYZ>),
_segmentedPcIndicesX(new std::vector<int>),
_segmentedPcIndicesY(new std::vector<int>),
_segmentedPcIndices(new std::vector<int>),
_filteredPcIndices(new std::vector<int>),
_segmentedSampledPcIndices(new std::vector<int>)
{
	std::cout << "CREATING MODEL SEGMENTER.." << std::endl;
	passThroughX.setFilterFieldName("x");
	passThroughY.setFilterFieldName("y");
	passThroughZ.setFilterFieldName("z");
	
	statOutlierRemovel.setMeanK(_sorNumOfNeibours);
	statOutlierRemovel.setStddevMulThresh(_sorStdDevMultThreshold);

	radOutlierRemovel.setRadiusSearch(_radRadius);
	radOutlierRemovel.setMinNeighborsInRadius(_radMinNeibours);

	fastBilateral.setSigmaR(_bilateralSigmaR);
	fastBilateral.setSigmaS(_bilateralSigmaS);

	voxelGrid.setLeafSize(_voxelGridSize, _voxelGridSize, _voxelGridSize);

	uniformSampling.setRadiusSearch(_usRadius);
	uniformSampling.setSearchMethod(_organizedSearchMethod);
}

void ModelSegmentor::startSegmenting(){
	pcl::PointCloud<pcl::PointXYZ> __pc;
	pcl::PointCloud<pcl::PointXYZRGB> __pcRgb;
	// Start counting
	pcl::ScopeTime timer("Segmentation");
	while (!_stopThread){
		if (_pauseThread){
			_lostTracking = true;
			_condPauseRequest.wait(std::unique_lock<std::mutex>(std::mutex()), [=](){return !_pauseThread; });
		}
		timer.reset();
		__pcRgb = _myKinect->getCurrentPC();
		if (!__pcRgb.size()){
			#ifdef VERBOSE
			std::cerr << "Segmenter: Empty cloud" << std::endl;
			#endif
			continue;
		}
		pcl::copyPointCloud(__pcRgb, __pc); // Optimize
		std::lock_guard<std::mutex> lock(_setDataKey);
		setInputCloud(__pc);
		filterAndSegment();
		#ifdef CHECK_SEGMENT_TIME
		std::cout << green << "Segmentation Time: " << timer.getTime() << "ms" << white << std::endl;
		#endif
	}
}

inline void ModelSegmentor::setInputCloud(pcl::PointCloud<pcl::PointXYZ> pc){
	_originalPc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(pc));
}

void ModelSegmentor::setInputCloudStream(MySensor* myKinect){
	_myKinect = myKinect;
}

void ModelSegmentor::setInitAligner(SacPrerejectiveAligner* aligner){
	_aligner = aligner;
}

void ModelSegmentor::setModelDimensions(std::vector<float> dimensions){
	std::lock_guard<std::mutex> lock(_setDataKey);
	_distX = dimensions[0]/2.0f * _offsetScale;
	_distY = dimensions[1]/2.0f * _offsetScale;
	_distZ = dimensions[2]/2.0f * _offsetScale;
	_radius = dimensions[3] * _offsetScale;
}

void ModelSegmentor::setModelOrigin(pcl::PointCloud<pcl::PointXYZ> origin){
	std::lock_guard<std::mutex> lock(_setDataKey);
	_origin = origin;
}

inline void ModelSegmentor::segment(){
#if 1
	if (_lostTracking || _centerTransformFromTracker.isZero()){
		pcl::transformPointCloud(_origin, _originTranformed, _aligner->getResults().finalTransformation);
		_center = Eigen::Vector3f(_originTranformed[0].x, _originTranformed[0].y, _originTranformed[0].z);
	}else{
		_updateCenterKey.lock();
		pcl::transformPointCloud(_origin, _originTranformed, _centerTransformFromTracker);
		_center = Eigen::Vector3f(_originTranformed[0].x, _originTranformed[0].y, _originTranformed[0].z);
		_updateCenterKey.unlock();
	}
	passThroughX.setInputCloud(_originalPc);
	passThroughX.setFilterLimits(_center(0) - _radius, _center(0) + _radius);
	passThroughX.filter(*_segmentedPcIndicesX);

	passThroughY.setInputCloud(_originalPc);
	passThroughY.setIndices(_segmentedPcIndicesX);
	passThroughY.setFilterLimits(_center(1) - _radius, _center(1) + _radius);
	passThroughY.filter(*_segmentedPcIndicesY);

	passThroughZ.setInputCloud(_originalPc);
	passThroughZ.setIndices(_segmentedPcIndicesY);
	passThroughZ.setFilterLimits(_center(2) - _radius, _center(2) + _radius);
	passThroughZ.filter(*_segmentedPcIndices);

#else
	passThroughZ.setInputCloud(_originalPc);
	passThroughZ.setFilterLimits(0.0f, 1.0f);
	passThroughZ.filter(*_segmentedPcIndicesY);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(_originalPc);
	seg.setIndices(_segmentedPcIndicesY);
	seg.segment(*inliers, *coefficients);
	_segmentedPcIndicesX = boost::make_shared<std::vector<int>>(inliers->indices);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(_originalPc);
	chull.setIndices(_segmentedPcIndicesX);
	chull.setDimension(2);

	if (_segmentedPcIndicesX->size() <= 0){
		_segmentedPcIndices = _segmentedPcIndicesX;
		return;
	}
	chull.reconstruct(*cloud_hull);
	if (cloud_hull != NULL && cloud_hull->size() > 0)
		cloud_hull->push_back(cloud_hull->at(0));

	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
	prism.setInputCloud(_originalPc);
	prism.setInputPlanarHull(cloud_hull);
	prism.setHeightLimits(0.04, 10);
	pcl::PointIndices::Ptr inl(new pcl::PointIndices);
	prism.segment(*inl);
	_segmentedPcIndices = boost::make_shared<std::vector<int>>(inl->indices);
	//_segmentedPcIndices = _segmentedPcIndicesX;
#endif
	//(0.0789492, -0.0141959, 0.681041)
}

inline void ModelSegmentor::filter(){
#if 0
	statOutlierRemovel.setInputCloud(_originalPc);
	statOutlierRemovel.setIndices(_segmentedPcIndices);
	statOutlierRemovel.filter(*_filteredPcIndices);
#elif 1
	radOutlierRemovel.setInputCloud(_originalPc);
	radOutlierRemovel.setIndices(_segmentedPcIndices);
	radOutlierRemovel.filter(*_filteredPcIndices);
#elif 0
	fastBilateral.setInputCloud(_originalPc);
	fastBilateral.setIndices(_segmentedPcIndices);
#else
	_filteredPcIndices = _segmentedPcIndices;
#endif
}

inline void ModelSegmentor::downSample(){
#if 0
	voxelGrid.setInputCloud(_originalPc);
	voxelGrid.setIndices(_filteredPcIndices);
	voxelGrid.filter(*);
#elif 1
	uniformSampling.setInputCloud(_originalPc);
	uniformSampling.setIndices(_filteredPcIndices);
	pcl::PointCloud<int> keypointIndices;
	uniformSampling.compute(keypointIndices);
	_segmentedSampledPcIndices = boost::make_shared<std::vector<int>>(
		std::vector<int>(keypointIndices.points.begin(), keypointIndices.points.end()));
#else
	_segmentedSampledPcIndices = _filteredPcIndices;
#endif
}

inline void ModelSegmentor::updateCurrentSegmentedCloud(){
	std::lock_guard<std::mutex> lock(_getResultsKey);
	_currentSegmentedCloud.originalPc = *_originalPc;
	_currentSegmentedCloud.segmentedPcIndices = *_segmentedSampledPcIndices;
}

inline void ModelSegmentor::filterAndSegment(){
	segment();
	filter();
	downSample();
	updateCurrentSegmentedCloud();
}

ModelSegmentor::SegmentedCloud ModelSegmentor::getCurrentSegmentedCloud(){
	std::lock_guard<std::mutex> lock(_getResultsKey);
	return std::ref(_currentSegmentedCloud);
}

void ModelSegmentor::updateCenterTransform(Eigen::Matrix4f center){
	std::lock_guard<std::mutex> lock(_updateCenterKey);
	_centerTransformFromTracker = center;
}

void ModelSegmentor::updateTrackingStatus(bool lost){
	std::lock_guard<std::mutex> lock(_updateCenterKey);
	_lostTracking = lost;
}

void ModelSegmentor::pause(){
	_pauseThread = true;
}

void ModelSegmentor::resume(){
	_pauseThread = false;
	_condPauseRequest.notify_one();
}

void ModelSegmentor::stop(){
	_stopThread = true;
}
