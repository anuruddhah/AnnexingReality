#include "stdafx.h"
#include "featureCalculator.h"

FeatureCalculator::FeatureCalculator() :
_depthLlimit(2.0f),
_sorNumOfNeibours(50), //
_sorStdDevMultThreshold(0.2f), //
_voxelGridSize(0.005f),
_usRadius(0.01f),
_normalRadius(0.0075f),
_featureRadius(0.025f),

_myKinect(new MySensor),
_searchMethod(new pcl::search::KdTree<pcl::PointXYZ>),
_organizedSearchMethod(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>),

_pc(new pcl::PointCloud<pcl::PointXYZ>),
_segmentedPcIndicesX(new std::vector<int>), //
_segmentedPcIndicesY(new std::vector<int>), //
_segmentedPcIndicesZ(new std::vector<int>), //
_segmentedPcIndices(new std::vector<int>),
_filteredPcIndices(new std::vector<int>),
_sampledPcIndices(new std::vector<int>),

_keyPoints(new pcl::PointCloud<pcl::PointXYZ>),
_sampledPc(new pcl::PointCloud<pcl::PointXYZ>),

_normals(new pcl::PointCloud<pcl::Normal>),
_localFeatures(new pcl::PointCloud<LocalFeature>),

_stopThread(false)
{
	std::cout << "CREATING FEATURE CACULATOR.." << std::endl;
	passThrough.setFilterFieldName("z");
	passThrough.setFilterLimits(0, _depthLlimit);

	statOutlierRemovel.setMeanK(_sorNumOfNeibours); //
	statOutlierRemovel.setStddevMulThresh(_sorStdDevMultThreshold); //

	voxelGrid.setLeafSize(_voxelGridSize, _voxelGridSize, _voxelGridSize);

	uniformSampling.setRadiusSearch(_usRadius);

	//kpExt.setSearchMethod(_searchMethod);
	//double resolution = 0.0026;
	//kpExt.setSalientRadius(6 * resolution);
	//kpExt.setNonMaxRadius(4 * resolution);
	//kpExt.setMinNeighbors(5);
	//kpExt.setThreshold21(0.975);
	//kpExt.setThreshold32(0.975);
	//kpExt.setNumberOfThreads(2);

	normEst.setSearchMethod(_searchMethod);
	normEst.setRadiusSearch(_normalRadius);
	//normEst.setNumberOfThreads(2); //For OMP

	integralNormEst.setNormalEstimationMethod(integralNormEst.AVERAGE_3D_GRADIENT);
	integralNormEst.setMaxDepthChangeFactor(0.02f);
	integralNormEst.setNormalSmoothingSize(10.0f);

	fpfhEst.setRadiusSearch(_featureRadius);
	//fpfhEst.setMinimalRadius(_featureRadius/10.0f);
	//fpfhEst.setPointDensityRadius(_featureRadius/5.0f);
	//fpfhEst.setNumberOfThreads(2); //For OMP
}

void FeatureCalculator::startProcessing(){
	pcl::PointCloud<pcl::PointXYZ> __pc; // Optimize
	pcl::PointCloud<pcl::PointXYZRGB> __pcRgb; // Optimize
	// Start counting
	pcl::ScopeTime timer("Feature calculation");
	while (!_stopThread){
		timer.reset();
		__pcRgb = _myKinect->getCurrentPC();
		if (!__pcRgb.size()){
			#ifdef VERBOSE
			std::cerr << "Featuer calc: Empty cloud" << std::endl;
			#endif
			continue;
		}
		pcl::copyPointCloud(__pcRgb, __pc); // Optimize
		setInputCloud(__pc); // Optimize
		filterAndFeatureCalc();
		#ifdef CHECK_FEATURECALC_TIME
			std::cout << blue << "FeatureCalculation Time: " << timer.getTime() << "ms" << white << std::endl;
		#endif
	}
}

void FeatureCalculator::setInputCloud(pcl::PointCloud<pcl::PointXYZ> pc){
	_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(pc)); // Optimize
}

void FeatureCalculator::setInputCloudStream(MySensor* myKinect){
	_myKinect = myKinect;
}

void FeatureCalculator::segment(){
#if 1
	passThrough.setInputCloud(_pc);
	passThrough.filter(*_segmentedPcIndices);
#else
	if (_pc->isOrganized()){
		passThrough.setInputCloud(_pc);
		passThrough.setFilterLimits(0.0f, 1.0f);
		passThrough.filter(*_segmentedPcIndicesY);

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);
		seg.setInputCloud(_pc);
		seg.setIndices(_segmentedPcIndicesY);
		seg.segment(*inliers, *coefficients);
		_segmentedPcIndicesX = boost::make_shared<std::vector<int>>(inliers->indices);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud(_pc);
		chull.setIndices(_segmentedPcIndicesX);
		chull.setDimension(2);

		if (_segmentedPcIndicesX->size() <= 0){
			_segmentedPcIndices = _segmentedPcIndicesX;
			return;
		}

		chull.reconstruct(*cloud_hull);
		if (cloud_hull != NULL && cloud_hull->size() > 0)
			cloud_hull->push_back(cloud_hull->at(0));

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(_pc);
		extract.setIndices(_segmentedPcIndicesX);
		extract.setNegative(false);
		extract.setKeepOrganized(true);
		pcl::PointCloud<pcl::PointXYZ> outliers;
		extract.filter(outliers);
		_segmentedPcIndicesZ = extract.getRemovedIndices();

		pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
		prism.setInputCloud(_pc);
		prism.setIndices(_segmentedPcIndicesZ);
		prism.setInputPlanarHull(cloud_hull);
		prism.setHeightLimits(0.005, 10);
		pcl::PointIndices::Ptr inl(new pcl::PointIndices);
		prism.segment(*inl);
		_segmentedPcIndices = boost::make_shared<std::vector<int>>(inl->indices);
	}
	else{
		passThrough.setInputCloud(_pc);
		passThrough.filter(*_segmentedPcIndices);
	}
#endif
}

void FeatureCalculator::filter(){
#if 1
	if (!(_pc->isOrganized())){
		_filteredPcIndices = _segmentedPcIndices;
		return;
	}
	statOutlierRemovel.setInputCloud(_pc);
	statOutlierRemovel.setIndices(_segmentedPcIndices);
	statOutlierRemovel.filter(*_filteredPcIndices);
#else
	_filteredPcIndices = _segmentedPcIndices;
#endif
}

void FeatureCalculator::downSample(){
	//pcl::ScopeTime timer("DOWNSAMPLE");
#if 0
	voxelGrid.setInputCloud(_pc);
	voxelGrid.setIndices(_filteredPcIndices);
	voxelGrid.filter(*);
#elif 1
	if (_pc->isOrganized()){
		uniformSampling.setSearchMethod(_organizedSearchMethod);
	} else{
		uniformSampling.setSearchMethod(_searchMethod);
	}
	uniformSampling.setInputCloud(_pc);
	uniformSampling.setIndices(_filteredPcIndices);
	pcl::PointCloud<int> keypointIndices;
	uniformSampling.compute(keypointIndices);
	_sampledPcIndices = boost::make_shared<std::vector<int>>(
		std::vector<int>(keypointIndices.points.begin(), keypointIndices.points.end()));
	pcl::copyPointCloud(*_pc, *_sampledPcIndices, *_sampledPc);
#else
	_sampledPcIndices = _filteredPcIndices;
	pcl::copyPointCloud(*_pc, *_sampledPcIndices, *_sampledPc);
#endif	
}

void FeatureCalculator::extractKeyPoints(){
	//pcl::ScopeTime timer("EXTRACT KEY POINTS");
#if 0
	kpExt.setInputCloud(_sampledPc);
	//kpExt.setIndices(_sampledPcIndices);
	kpExt.compute(*_keyPoints);
#else
	_keyPoints = _sampledPc;
#endif
}

void FeatureCalculator::calcNormals(){
	//pcl::ScopeTime timer("NORMAL CALC");
	if (false){
		integralNormEst.setInputCloud(_pc);
		integralNormEst.compute(*_normals);
	}else{
		normEst.setInputCloud(_pc);
		normEst.setIndices(_sampledPcIndices);
		normEst.setSearchSurface(_pc);
		normEst.compute(*_normals);
	}
}

void FeatureCalculator::calcFeatures(){
	//pcl::ScopeTime timer("FPFH CALC");
	if (false){
		fpfhEst.setSearchMethod(_organizedSearchMethod);
		fpfhEst.setInputCloud(_pc);
		fpfhEst.setInputNormals(_normals);
		fpfhEst.setIndices(_sampledPcIndices);
	}else{
		fpfhEst.setSearchMethod(_searchMethod);
		fpfhEst.setInputCloud(_keyPoints);
		fpfhEst.setInputNormals(_normals);
		fpfhEst.setSearchSurface(_sampledPc);
	}
	fpfhEst.compute(*_localFeatures);
}

void FeatureCalculator::updateCurrentProcessedCloud(){
	std::lock_guard<std::mutex> lock(_getResultsKey);
	_currentFeatureCloud.originalPc = *_pc;
	_currentFeatureCloud.sampledPcIndices = *_sampledPcIndices;
	_currentFeatureCloud.keyPoints = *_keyPoints;
	_currentFeatureCloud.localFeatures = *_localFeatures;
	//std::cout << "FEATURE SIZE: " << _localFeatures->size() << std::endl;
}

void FeatureCalculator::filterAndFeatureCalc(){
	segment();
	filter();
	downSample();
	extractKeyPoints();
	calcNormals();
	calcFeatures();
	updateCurrentProcessedCloud();
}

void FeatureCalculator::featureCalcOnly(){
	calcNormals();
	calcFeatures();
	updateCurrentProcessedCloud();
}

void FeatureCalculator::filterOnly(){
	segment();
	filter();
	downSample();
	updateCurrentProcessedCloud();
}

FeatureCalculator::FeatureCloud FeatureCalculator::getCurrentFeatureCloud(){
	std::lock_guard<std::mutex> lock(_getResultsKey);
	return std::ref(_currentFeatureCloud);
}

void FeatureCalculator::stop(){
	_stopThread = true;
}
