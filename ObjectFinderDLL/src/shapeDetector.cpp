#include "stdafx.h"
#include "shapeDetector.h"

#if 0
int main(){
	PointCloud pc;

	pcl::PointCloud<pcl::PointNormal> modelFormFile;
	if (pcl::io::loadPCDFile("normalModel.pcd", modelFormFile) != 0){
		return 1;
	}

	for (int i = 0; i < modelFormFile.size(); i++){
		Point Pt;
		Pt.pos[0] = modelFormFile.at(i).x;
		Pt.pos[1] = modelFormFile.at(i).y;
		Pt.pos[2] = modelFormFile.at(i).z;
		Pt.normal[0] = modelFormFile.at(i).normal_x;
		Pt.normal[1] = modelFormFile.at(i).normal_y;
		Pt.normal[2] = modelFormFile.at(i).normal_z;
		pc.push_back(Pt);
	}

	Eigen::Vector4f min, max;
	pcl::getMinMax3D(modelFormFile, min, max);

	Vec3f cbbMin, cbbMax;
	cbbMin[0] = static_cast<float>(min[0]);
	cbbMin[1] = static_cast<float>(min[1]);
	cbbMin[2] = static_cast<float>(min[2]);
	cbbMax[0] = static_cast<float>(max[0]);
	cbbMax[1] = static_cast<float>(max[1]);
	cbbMax[2] = static_cast<float>(max[2]);
	pc.setBBox(cbbMin, cbbMax);

	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_epsilon = .01f * pc.getScale(); // set distance threshold to .01f of bounding box width
	// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
	ransacOptions.m_bitmapEpsilon = .02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
	// NOTE: This threshold is NOT multiplied internally!
	ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = 500; // this is the minimal numer of points required for a primitive
	ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions);
	detector.Add(new PlanePrimitiveShapeConstructor());
	detector.Add(new SpherePrimitiveShapeConstructor());
	detector.Add(new CylinderPrimitiveShapeConstructor());
	detector.Add(new ConePrimitiveShapeConstructor());
	detector.Add(new TorusPrimitiveShapeConstructor());

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes;

	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes);
	std::string name;
	shapes.at(0).first->Description(&name);
	std::cout << "Remain " << remaining << " shapes " << shapes.size() << name << std::endl;
	return 0;
}
#else

#if 0
int main()
{
	int size = 3;

	dlib::matrix<int> cost = dlib::matrix<int>(size, size);

	for (int i = 0; i < size; i++)
		for (int j = 0; j < size; j++)
			cost(i, j) = 0;


	//cost = 10, 0, 10,
	//       0, 0, 5,
	//       9, 0, 1;

	cost(0, 0)= 10;
	cost(0, 2) = 10;
	cost(1, 2) = 5;
	cost(2, 0) = 9;
	cost(2, 2) = 1;

	// To find out the best assignment of people to jobs we just need to call this function.
	std::vector<long> assignment = dlib::max_cost_assignment(cost);

	// This prints optimal assignments:  [2, 0, 1] which indicates that we should assign
	// the person from the first row of the cost matrix to job 2, the middle row person to
	// job 0, and the bottom row person to job 1.
	for (unsigned int i = 0; i < assignment.size(); i++)
		std::cout << assignment[i] << std::endl;

	// This prints optimal cost:  16.0
	// which is correct since our optimal assignment is 6+5+5.
	std::cout << "optimal cost: " << dlib::assignment_cost(cost, assignment) << std::endl;
}

#endif

void BaseTypeParams::dummy(){}

ShapeDetector::ShapeDetector():
_resultsFetched(false),
_resultsFetchedTemp(false),
_noOfDetections(0),
_myKinect(new MySensor),
_allShapesDetected(false),
_pauseThread(false),
_stopThread(false)
{
	std::cout << "CREATING SHAPE DETECTOR.." << std::endl;
}

void ShapeDetector::startMatching(){
	//_viewer = new pcl::visualization::PCLVisualizer("Cluster Viewer");
	//_viewer->setBackgroundColor(0, 0, 0);
	//_viewer->initCameraParameters();
	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::PointCloud<pcl::PointXYZRGB> pcRgb;
	//// Start counting
	//pcl::ScopeTime timer("Shape detection");
	while (!_stopThread){
		if (_pauseThread){
			_condPauseRequest.wait(std::unique_lock<std::mutex>(std::mutex()), [=](){return !_pauseThread; });
		}
		//timer.reset();
		pcRgb = _myKinect->getCurrentPC();
		if (!pcRgb.size()){
		#ifdef VERBOSE
			std::cerr << "Shape Detector: Empty cloud" << std::endl;
		#endif
			continue;
		}
		pcl::copyPointCloud(pcRgb, pc); // Optimize
		setInputCloud(pc);
		std::lock_guard<std::mutex> lock(_setShapesKey);
		cluster();
		detectShapes();
		match();
		updateresults();
		if (false && _allShapesDetected){
			_condShapesFound.wait(std::unique_lock<std::mutex>(std::mutex()), [=](){return !_allShapesDetected; });
		}
		#ifdef CHECK_MATCHING_TIME
			std::cout << blue << "Shape detection Time: " << timer.getTime() << "ms" << white << std::endl;
		#endif
	}
}

void ShapeDetector::setInputCloud(pcl::PointCloud<pcl::PointXYZ> pc){
	_pc = pc;
}

void ShapeDetector::setInputCloudStream(MySensor* myKinect){
	_myKinect = myKinect;
}

void ShapeDetector::setShapes(std::vector<Shape> shapes){
	std::lock_guard<std::mutex> lock(_setShapesKey);
	_desiredShapes = shapes;
}

double ShapeDetector::quickAlign(pcl::PointCloud<pcl::PointXYZ> model, pcl::PointCloud<pcl::PointXYZ> cluster){
	//FeatureCalculator _featureCalc;
	//_featureCalc.setInputCloud(model);
	//_featureCalc.filterAndFeatureCalc();
	//FeatureCalculator::FeatureCloud _processedModel = _featureCalc.getCurrentFeatureCloud();
	//_featureCalc.setInputCloud(cluster);
	//_featureCalc.filterAndFeatureCalc();
	//FeatureCalculator::FeatureCloud _processedCluster = _featureCalc.getCurrentFeatureCloud();
	//pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, LocalFeature> _sacpr;
	//_sacpr.setMaximumIterations(50000);
	//_sacpr.setNumberOfSamples(3);
	//_sacpr.setCorrespondenceRandomness(5);
	//_sacpr.setSimilarityThreshold(0.9);
	//_sacpr.setMaxCorrespondenceDistance(0.005);
	//_sacpr.setInlierFraction(0.9);
	//_sacpr.setInputSource(_processedModel.keyPoints.makeShared());
	//_sacpr.setSourceFeatures(_processedModel.localFeatures.makeShared());
	//_sacpr.setInputTarget(_processedCluster.keyPoints.makeShared());
	//_sacpr.setTargetFeatures(_processedCluster.localFeatures.makeShared());
	//pcl::PointCloud<pcl::PointXYZ> _output;
	//_sacpr.align(_output);
	//return _sacpr.getFitnessScore();

	pcl::PointCloud<pcl::VFHSignature308> modelDescriptor;
	pcl::PointCloud<pcl::VFHSignature308> clusterDescriptor;

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.setRadiusSearch(0.04f);

	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> estimator;
	estimator.setFillSizeComponent(true);
	estimator.setNormalizeBins(true);
	estimator.setNormalizeDistance(false);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointXYZ>);
	estimator.setSearchMethod(kdtree2);

	normalEstimation.setInputCloud(model.makeShared());
	normalEstimation.compute(*normals);

	estimator.setInputCloud(model.makeShared());
	estimator.setInputNormals(normals);
	estimator.compute(modelDescriptor);

	normalEstimation.setInputCloud(cluster.makeShared());
	normalEstimation.compute(*normals);

	estimator.setInputCloud(cluster.makeShared());
	estimator.setInputNormals(normals);
	estimator.compute(clusterDescriptor);

	float dist = 0.0f;
	int pointer = 0;
	float gap;
	for (int i = 0; i < 308; i++){
		gap = std::fabs(*(modelDescriptor.at(0).histogram + pointer) - *(clusterDescriptor.at(0).histogram + pointer));
		if (gap > 0)
			dist = dist + gap;
		pointer++;
	}

	

	//std::cout << green << "MANHATTAN DISTANCE: " << dist << white << std::endl;

#if 0	// debug: plotting vfh histograms for model and cluster
	pcl::visualization::PCLPlotter plotter("Feature Histogram");
	plotter.setWindowSize(1920, 1200);
	plotter.addFeatureHistogram(modelDescriptor, 308, "model");
	plotter.addFeatureHistogram(clusterDescriptor, 308, "cluster");
	plotter.spinOnce(2000);
#endif

	return dist;
}

void ShapeDetector::cluster(){
	// debug: clear visualizing clusters
	//_viewer->removeAllPointClouds();

	_clusters.clear();
	_clusters.resize(0);

	pcl::IndicesPtr segmentedPcIndices1(new std::vector<int>);
	pcl::IndicesPtr segmentedPcIndices2(new std::vector<int>);
	pcl::IndicesPtr segmentedPcIndices3(new std::vector<int>);
	pcl::IndicesConstPtr segmentedPcIndices4(new std::vector<int>);
	pcl::IndicesPtr filteredPcIndices(new std::vector<int>);

	pcl::PassThrough<pcl::PointXYZ> passThrough;
	passThrough.setFilterFieldName("z");
	passThrough.setFilterLimits(0, 1.5f);
	passThrough.setInputCloud(_pc.makeShared());
	passThrough.setFilterLimits(0.0f, 1.5f);
	passThrough.filter(*segmentedPcIndices1);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(_pc.makeShared());
	seg.setIndices(segmentedPcIndices1);
	seg.segment(*inliers, *coefficients);
	segmentedPcIndices2 = boost::make_shared<std::vector<int>>(inliers->indices);

	if (segmentedPcIndices2->size() <= 0){
		std::cerr << "pc size " << _pc.size() << std::endl;
		std::cerr << "Shape Detector: No plane detected" << std::endl;
		initVoteMatrix();
		return;
	}

	
	pcl::PointCloud<pcl::PointXYZ> plane;
	pcl::copyPointCloud(_pc, *segmentedPcIndices2, plane);
	//_viewer->addPointCloud<pcl::PointXYZ>(plane.makeShared(), "plane");
	//_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane");
	//_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "plane");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(_pc.makeShared());
	chull.setIndices(segmentedPcIndices2);
	chull.setDimension(2);
	chull.reconstruct(*cloud_hull);
	if (cloud_hull != NULL && cloud_hull->size() > 0)
		cloud_hull->push_back(cloud_hull->at(0));

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(_pc.makeShared());
	extract.setIndices(segmentedPcIndices2);
	extract.setNegative(false);
	extract.setKeepOrganized(true);
	pcl::PointCloud<pcl::PointXYZ> outliers;
	extract.filter(outliers);
	segmentedPcIndices4 = extract.getRemovedIndices();

	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
	prism.setInputCloud(_pc.makeShared());
	prism.setIndices(segmentedPcIndices4);
	prism.setInputPlanarHull(cloud_hull);
	prism.setHeightLimits(0.005, 10);
	pcl::PointIndices::Ptr inl(new pcl::PointIndices);
	prism.segment(*inl);
	segmentedPcIndices3 = boost::make_shared<std::vector<int>>(inl->indices);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statOutlierRemovel;
	statOutlierRemovel.setMeanK(50);
	statOutlierRemovel.setStddevMulThresh(0.2f);
	statOutlierRemovel.setInputCloud(_pc.makeShared());
	statOutlierRemovel.setIndices(segmentedPcIndices3);
	statOutlierRemovel.filter(*filteredPcIndices);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(500);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(_pc.makeShared());
	ec.setIndices(filteredPcIndices);
	ec.extract(clusterIndices);

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	mls.setComputeNormals(false);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.02);
	
	int count = 3;

	for each (pcl::PointIndices indices in clusterIndices){
		Eigen::Vector4f min, max;
		pcl::getMinMax3D(_pc, indices, min, max);
		bool collision = false;
		for each (Shape shape in _desiredShapes){
			if (shape.jumpAllowed)
				continue;
			TrackingResults trackResult = shape.tracker->getResults();
			if (trackResult.trackingSucceeded){
				if (min.x() < trackResult.translation[0] && trackResult.translation[0] < max.x() &&
					min.y() < trackResult.translation[1] && trackResult.translation[1] < max.y() &&
					min.z() < trackResult.translation[2] && trackResult.translation[2] < max.z()){
					collision = true;
					#ifdef SHAPEVERBOSE
						std::cout << yellow << "COLLISION (tracking)..." << white << std::endl;
					#endif
					break;
				}
			}else if (shape.model.size() > 0){
				pcl::PointCloud<pcl::PointXYZ> cluster;
				pcl::copyPointCloud(_pc, indices, cluster);
				double dist = quickAlign(shape.model, cluster);
				if (dist < SIMILARITY_THRESHOLD){
					collision = true;
					#ifdef SHAPEVERBOSE
						std::cout << yellow << "COLLISION (no tracking)..." << white << std::endl;
					#endif
					break;
				}
			}
		}
		if (!collision){
			pcl::PointCloud<pcl::PointXYZ> cluster;
			pcl::copyPointCloud(_pc, indices, cluster);
			mls.setInputCloud(cluster.makeShared());
			//mls.setIndices(pcl::PointIndicesConstPtr(&indices));
			mls.process(cluster);
			_clusters.push_back(cluster);
			//_viewer->addPointCloud<pcl::PointXYZ>(cluster.makeShared(), std::to_string(count));
			//_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::to_string(count));
			//_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 3 % count, 4 % count, 5 % count, std::to_string(count));
			count++;
		}
	}

	#if 0
		std::cout << green << "Number of clusters: " << _clusters.size() << white << std::endl;
	#endif
	initVoteMatrix();

	//_viewer->spinOnce(1, false);
}

inline void detectprimitive(PointCloud &pc, std::list<int> &shapes, RansacShapeDetector::Options &options,
	MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t>> &primitives, size_t &remain){
	RansacShapeDetector detector(options);
	for each (int id in shapes){
		switch (id){
		case PLANE: detector.Add(new PlanePrimitiveShapeConstructor()); break;
		case SPHERE: detector.Add(new SpherePrimitiveShapeConstructor()); break;
		case CYLINDER: detector.Add(new CylinderPrimitiveShapeConstructor()); break;
		case CONE: detector.Add(new ConePrimitiveShapeConstructor()); break;
		case TORUS: detector.Add(new TorusPrimitiveShapeConstructor()); break;
		}
	}
	remain = detector.Detect(pc, 0, pc.size(), &primitives);
}

inline bool featureVarification(PrimitiveShape *detectedPrimitive){
	switch (detectedPrimitive->Identifier()){
	case PLANE:{
		const PlanePrimitiveShape* plane = static_cast<const PlanePrimitiveShape*>(detectedPrimitive);
		}break;
	case SPHERE:{
		const SpherePrimitiveShape* sphere = static_cast<const SpherePrimitiveShape*>(detectedPrimitive);
		float radius = sphere->Internal().Radius();
		if (radius > 0.25) // Spheres can't be larger than 0.5m radius
			return false;
		}break;
	case CYLINDER:{
		const CylinderPrimitiveShape* cylinder = static_cast<const CylinderPrimitiveShape*>(detectedPrimitive);
		float radius = cylinder->Internal().Radius();
		if (radius > 0.1) // Cylinders can't be larger than 0.2m radius
			return false;
		}break;
	case CONE:{
		const ConePrimitiveShape* cone = static_cast<const ConePrimitiveShape*>(detectedPrimitive);
		float angle = cone->Internal().Angle();
		if (angle > 2.0) // Cone can't be larger than 2.0 rad in angle
			return false;
		}break;
	case TORUS:{
		const TorusPrimitiveShape* torus = static_cast<const TorusPrimitiveShape*>(detectedPrimitive);
		if (torus->Internal().IsAppleShaped()) // Torus can't be apple shaped
			return false;
		float majorRadius = torus->Internal().MajorRadius();
		if (majorRadius > 0.25) // Torus can't be larger than 0.5m radius
			return false;
		}break;
	}
	return true;
}

void ShapeDetector::detectShapes(){
	if (_clusters.size() <= 0){
		std::cerr << "No clusters detected" << std::endl;
		return;
	}

	_detectedShapes.clear();

	//for each cluster, detect, save shape and vote
	for (int clusterNo = 0; clusterNo < _clusters.size(); clusterNo++){
		ShapeBasic extractedShape(std::vector<Primitive>{});

		pcl::PointCloud<pcl::PointXYZ> object = _clusters.at(clusterNo);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		
		//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
		//mls.setComputeNormals(false);
		//mls.setInputCloud(_clusters.at(clusterNo).makeShared());
		//mls.setPolynomialFit(true);
		//mls.setSearchMethod(kdtree);
		//mls.setSearchRadius(0.02);
		//mls.process(object);

		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(object.makeShared());
		normalEstimation.setRadiusSearch(0.02f);
		normalEstimation.setSearchMethod(kdtree);
		normalEstimation.compute(*normals);

		pcl::PointCloud<pcl::PointNormal> objectNormal;
		if (normals->size() <= 0){
			std::cerr << "Shape Detector: Empty normals" << std::endl;
			return;
		}
		pcl::concatenateFields(object, *normals, objectNormal);

		PointCloud pc;
		for (int i = 0; i < object.size(); i++){
			Point pt;
			pt.pos[0] = objectNormal.at(i).x;
			pt.pos[1] = objectNormal.at(i).y;
			pt.pos[2] = objectNormal.at(i).z;
			pt.normal[0] = objectNormal.at(i).normal_x;
			pt.normal[1] = objectNormal.at(i).normal_y;
			pt.normal[2] = objectNormal.at(i).normal_z;
			pc.push_back(pt);
		}

		float pcSize = pc.size();

		Eigen::Vector4f min, max;
		pcl::getMinMax3D(objectNormal, min, max);
		Vec3f cbbMin, cbbMax;
		cbbMin[0] = static_cast<float>(min[0]);
		cbbMin[1] = static_cast<float>(min[1]);
		cbbMin[2] = static_cast<float>(min[2]);
		cbbMax[0] = static_cast<float>(max[0]);
		cbbMax[1] = static_cast<float>(max[1]);
		cbbMax[2] = static_cast<float>(max[2]);
		pc.setBBox(cbbMin, cbbMax);

		std::list<int> shapeIds{0, 1, 2, 3, 4};

		RansacShapeDetector::Options ransacOptions;
		ransacOptions.m_epsilon = .01f * pc.getScale();
		ransacOptions.m_bitmapEpsilon = .02f * pc.getScale();
		ransacOptions.m_normalThresh = .99f;
		ransacOptions.m_minSupport = 300;
		ransacOptions.m_probability = .001f;

	detect:
		MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t>> detectedPrimitives;
		size_t remainingPts;
		
		if (shapeIds.size() <= 0)
			continue;

		detectprimitive(pc, shapeIds, ransacOptions, detectedPrimitives, remainingPts);

		if (detectedPrimitives.size() > 0){
			PrimitiveShape *detectedPrimitive1 = detectedPrimitives.at(0).first;
			int pointCount = detectedPrimitives.at(0).second;
			//int detectedPrimPtCnt = detectedPrimitives.at(0).second;
			if (featureVarification(detectedPrimitive1)){
				#ifdef SHAPEVERBOSE
					std::cout << blue << "VERIFIED" << white << std::endl;
				#endif
				PrimShape type = static_cast<PrimShape>(detectedPrimitive1->Identifier());
				BaseTypeParams *params = new BaseTypeParams();
				switch (type){
				case PLANE:	{
					const PlanePrimitiveShape* plane
						= static_cast<const PlanePrimitiveShape*>(detectedPrimitive1);
					// calculating dimensions
					float minX, maxX, minY, maxY;
					for (int i = 0; i < pointCount; i++){
						std::pair<float, float> param;
						plane->Parameters(pc.at(pcSize - 1 - i).pos, &param);
						if (i != 0){
							if (minX > param.first)
								minX = param.first;
							else if (maxX < param.first)
								maxX = param.first;
							if (minY > param.second)
								minY = param.second;
							else if (maxY < param.second)
								maxY = param.second;
						}else{
							minX = maxX = param.first;
							minY = maxY = param.second;
						}
					}
					float width = maxX - minX;
					float length = maxY - minY;
					PlaneParams *planeParams = new PlaneParams(width, length);
					plane->Internal().getPosition().getValue(planeParams->center.x(), 
						planeParams->center.y(), planeParams->center.z());
					plane->Internal().getNormal().getValue(planeParams->normal.x(), 
						planeParams->normal.y(), planeParams->normal.z());
					params = planeParams;
					//std::cout << "plane center " << planeParams->center.matrix().transpose() << std::endl;
					//std::cout << "plane normal " << planeParams->center.matrix().transpose() << std::endl;
					break;
				}
				case SPHERE: {
					const SpherePrimitiveShape* sphere
						= static_cast<const SpherePrimitiveShape*>(detectedPrimitive1);
					SphereParams *sphereParams = new SphereParams(sphere->Internal().Radius());
					sphere->Internal().Center().getValue(sphereParams->center.x(), 
						sphereParams->center.y(), sphereParams->center.z());
					params = sphereParams;
					//std::cout << "sphere center " << sphereParams->center.matrix().transpose() << std::endl;
					break;
				}
				case CYLINDER: {
					const CylinderPrimitiveShape* cylinder
						= static_cast<const CylinderPrimitiveShape*>(detectedPrimitive1);
					// Recalculating center
					Vec3f axisPos = cylinder->Internal().AxisPosition();
					Vec3f axisDir = cylinder->Internal().AxisDirection();
					float radius = cylinder->Internal().Radius();
					float minH = cylinder->MinHeight();
					float maxH = cylinder->MaxHeight();
					float shift = minH + (maxH - minH) / 2.0f;
					axisPos = axisPos + axisDir * shift;
					CylinderParams *cylinderParams = new CylinderParams(radius, std::fabs(maxH - minH));
					axisPos.getValue(cylinderParams->axisPosition.x(),
						cylinderParams->axisPosition.y(), cylinderParams->axisPosition.z());
					axisDir.getValue(cylinderParams->axisDirection.x(),
						cylinderParams->axisDirection.y(), cylinderParams->axisDirection.z());
					params = cylinderParams;
					//std::cout << "cylinder center " << cylinderParams->axisPosition.matrix().transpose() << std::endl;
					//std::cout << "cylinder dir " << cylinderParams->axisDirection.matrix().transpose() << std::endl;
					break;
				}
				case CONE: {
					const ConePrimitiveShape* cone
						= static_cast<const ConePrimitiveShape*>(detectedPrimitive1);
					// Recalculating center
					Vec3f center = cone->Internal().Center(); // Init value is apex
					Vec3f axisDir = cone->Internal().AxisDirection();
					float angle = cone->Internal().Angle();
					float height = 0.0f;
					float heightTemp = 0.0f;
					for (int i = 0; i < pointCount; i++){
						heightTemp = cone->Internal().Height(pc.at(pcSize - 1 - i).pos);
						if (height < heightTemp)
							height = heightTemp;
					}
					float shift = height / 2.0f;
					center = center + axisDir * shift;
					ConeParams *coneParams = new ConeParams(angle, height);
					center.getValue(coneParams->center.x(), 
						coneParams->center.y(), coneParams->center.z());
					axisDir.getValue(coneParams->axisDirection.x(), 
						coneParams->axisDirection.y(), coneParams->axisDirection.z());
					params = coneParams;
					//std::cout << "cone center " << coneParams->center.matrix().transpose() << std::endl;
					//std::cout << "cone dir " << coneParams->axisDirection.matrix().transpose() << std::endl;
					break;
				}
				case TORUS: {
					const TorusPrimitiveShape* torus
						= static_cast<const TorusPrimitiveShape*>(detectedPrimitive1);
					TorusParams *torusParams = new TorusParams(torus->Internal().MinorRadius(), 
						torus->Internal().MajorRadius());
					torus->Internal().Center().getValue(torusParams->center.x(),
						torusParams->center.y(), torusParams->center.z());
					torus->Internal().AxisDirection().getValue(torusParams->axisDirection.x(),
						torusParams->axisDirection.y(), torusParams->axisDirection.z());
					params = torusParams;
					//std::cout << "torus center " << torusParams->center.matrix().transpose() << std::endl;
					//std::cout << "torus dir " << torusParams->axisDirection.matrix().transpose() << std::endl;
					break;
				}
				}
				std::vector<Primitive> primCandidates{Primitive(type, params)}; // Currently only interested in the dominent primitive
				ShapeBasic shape(primCandidates);
				extractedShape = shape;
				vote(shape, clusterNo);
			}else{
				#ifdef SHAPEVERBOSE
					std::cout << blue << "VERIFICATION FAILED  " << detectedPrimitive1->Identifier() << white << std::endl;
				#endif
				shapeIds.remove(detectedPrimitive1->Identifier());
				goto detect;
			}
		}

		_detectedShapes.push_back(extractedShape);


		////debug: print mathing
		//std::cout << green << "Total points: " << pc.size() << "      Remaining: " << remainingPts << "   ||   ";
		//if (detectedPrimitives.size() > 0){
		//	std::string name;
		//	for each (std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t> shape in detectedPrimitives)
		//	{
		//		shape.first->Description(&name);
		//		std::cout << yellow << name << ": " << shape.second << "   |   ";
		//	}
		//	std::cout << white << std::endl;
		//}
	}
}

void ShapeDetector::vote(ShapeBasic shapeCandidate, int clusterNo){
	for (int shapeNo = 0; shapeNo < _desiredShapes.size(); shapeNo++){
		Shape shapeDesired = _desiredShapes.at(shapeNo);
		int finalVote = 0;
		// Only vote if jumpAllowed or doesn't have a model yet
		if (shapeDesired.jumpAllowed || shapeDesired.model.size() <= 0){
			// For all primitives in the order of saliency: later
			for (int primNo = 0; primNo < shapeDesired.preferredPrimitives.size(); primNo++){
				if (shapeDesired.preferredPrimitives.at(primNo).type == shapeCandidate.primitives.at(0).type){ // Only interested in dominent primitive type
					int primSpecificVote = shapeDesired.objectPref * shapeDesired.preferredPrimitives.at(primNo).params->primitivePref; // Init the highest vote for the specific primitive of the specific shape
					switch (shapeDesired.preferredPrimitives.at(primNo).type){ // Reduce vote based on params
					case PLANE:	{
						PlaneParams *planeDesired = dynamic_cast<PlaneParams*>
							(shapeDesired.preferredPrimitives.at(primNo).params);
						PlaneParams *planeCandidate = dynamic_cast<PlaneParams*>
							(shapeCandidate.primitives.at(0).params);
						if (planeDesired != NULL && planeCandidate != NULL){
							float desiredLength = planeDesired->length;
							float detectedLength = planeCandidate->length;
							float desiredWidth = planeDesired->width;
							float detectedWidth = planeCandidate->width;
							float desiredRatio = desiredLength / desiredWidth;
							float detectedRatio = detectedLength / detectedWidth;
							float lengthMatch;
							(desiredLength >= detectedLength) ? (lengthMatch = detectedLength / desiredLength)
								: (lengthMatch = desiredLength / detectedLength);
							float widthmatch;
							(desiredWidth >= detectedWidth) ? (widthmatch = detectedWidth / desiredWidth)
								: (widthmatch = desiredWidth / detectedWidth);
							float ratioMatch;
							(desiredRatio >= detectedRatio) ? (ratioMatch = detectedRatio / desiredRatio)
								: (ratioMatch = desiredRatio / detectedRatio);
							primSpecificVote = primSpecificVote*
								(lengthMatch*planeDesired->lengthPref +
								widthmatch*planeDesired->widthPref +
								ratioMatch*planeDesired->ratioPref);
							//std::cout << ".....desiredLength >= detectedLength....." << desiredLength << "  " << detectedLength << std::endl;
							//std::cout << ".....desiredWidth >= detectedWidth....." << desiredWidth << "  " << detectedWidth << std::endl;
							//std::cout << ".....desiredRatio >= detectedRatio....." << desiredRatio << "  " << detectedRatio << std::endl;
							//std::cout << ".....lengthMatch....." << lengthMatch << std::endl;
							//std::cout << ".....widthmatch....." << widthmatch << std::endl;
							//std::cout << ".....ratioMatch...." << ratioMatch << std::endl;
						}
						break;
					}
					case SPHERE: {
						SphereParams *sphereDesired = dynamic_cast<SphereParams*>
							(shapeDesired.preferredPrimitives.at(primNo).params);
						SphereParams *sphereCandidate = dynamic_cast<SphereParams*>
							(shapeCandidate.primitives.at(0).params);
						if (sphereDesired != NULL && sphereCandidate != NULL){
							float desiredRadius = sphereDesired->radius;
							float detectedRadius = sphereCandidate->radius;
							float radiusmatch;
							(desiredRadius >= detectedRadius) ? (radiusmatch = detectedRadius / desiredRadius)
								: (radiusmatch = desiredRadius / detectedRadius);
							primSpecificVote = primSpecificVote*
								(radiusmatch*sphereDesired->radiusPref);
							//std::cout << ".....desiredRadius >= detectedRadius....." << desiredRadius << "  " << detectedRadius << std::endl;
							//std::cout << ".....radiusmatch....." << radiusmatch << std::endl;
						}
						break;
					}
					case CYLINDER: {
						CylinderParams *cylinderDesired = dynamic_cast<CylinderParams*>
							(shapeDesired.preferredPrimitives.at(primNo).params);
						CylinderParams *cylinderCandidate = dynamic_cast<CylinderParams*>
							(shapeCandidate.primitives.at(0).params);
						if (cylinderDesired != NULL && cylinderCandidate != NULL){
							float desiredHeight = cylinderDesired->height;
							float detectedHeight = cylinderCandidate->height;
							float desiredRadius = cylinderDesired->radius;
							float detectedRadius = cylinderCandidate->radius;
							float desiredRatio = desiredHeight / desiredRadius;
							float detectedRatio = detectedHeight / detectedRadius;
							float heightMatch;
							(desiredHeight >= detectedHeight) ? (heightMatch = detectedHeight / desiredHeight)
								: (heightMatch = desiredHeight / detectedHeight);
							float radiusmatch;
							(desiredRadius >= detectedRadius) ? (radiusmatch = detectedRadius / desiredRadius)
								: (radiusmatch = desiredRadius / detectedRadius);
							float ratioMatch;
							(desiredRatio >= detectedRatio) ? (ratioMatch = detectedRatio / desiredRatio)
								: (ratioMatch = desiredRatio / detectedRatio);
							primSpecificVote = primSpecificVote*
								(heightMatch*cylinderDesired->heightPref +
								radiusmatch*cylinderDesired->radiusPref +
								ratioMatch*cylinderDesired->ratioPref);
							//std::cout << ".....desiredHeight >= detectedHeight....." << desiredHeight << "  " << detectedHeight << std::endl;
							//std::cout << ".....desiredRadius >= detectedRadius....." << desiredRadius << "  " << detectedRadius << std::endl;
							//std::cout << ".....desiredRatio >= detectedRatio....." << desiredRatio << "  " << detectedRatio << std::endl;
							//std::cout << ".....heightMatch....." << heightMatch << std::endl;
							//std::cout << ".....radiusmatch....." << radiusmatch << std::endl;
							//std::cout << ".....ratioMatch...." << ratioMatch << std::endl;
						}
						break;
					}
					case CONE: {
						ConeParams *coneDesired = dynamic_cast<ConeParams*>
							(shapeDesired.preferredPrimitives.at(primNo).params);
						ConeParams *coneCandidate = dynamic_cast<ConeParams*>
							(shapeCandidate.primitives.at(0).params);
						if (coneDesired != NULL && coneCandidate != NULL){
							float desiredHeight = coneDesired->height;
							float detectedHeight = coneCandidate->height;
							float desiredAngle = coneDesired->angle;
							float detectedAngle = coneCandidate->angle;
							float desiredRatio = desiredHeight / desiredAngle;
							float detectedRatio = detectedHeight / detectedAngle;
							float heightMatch;
							(desiredHeight >= detectedHeight) ? (heightMatch = detectedHeight / desiredHeight)
								: (heightMatch = desiredHeight / detectedHeight);
							float anglematch;
							(desiredAngle >= detectedAngle) ? (anglematch = detectedAngle / desiredAngle)
								: (anglematch = desiredAngle / detectedAngle);
							float ratioMatch;
							(desiredRatio >= detectedRatio) ? (ratioMatch = detectedRatio / desiredRatio)
								: (ratioMatch = desiredRatio / detectedRatio);
							primSpecificVote = primSpecificVote*
								(heightMatch*coneDesired->heightPref +
								anglematch*coneDesired->anglePref +
								ratioMatch*coneDesired->ratioPref);
							//std::cout << ".....desiredHeight >= detectedHeight....." << desiredHeight << "  " << detectedHeight << std::endl;
							//std::cout << ".....desiredAngle >= detectedAngle....." << desiredAngle << "  " << detectedAngle << std::endl;
							//std::cout << ".....desiredRatio >= detectedRatio....." << desiredRatio << "  " << detectedRatio << std::endl;
							//std::cout << ".....heightMatch....." << heightMatch << std::endl;
							//std::cout << ".....anglematch....." << anglematch << std::endl;
							//std::cout << ".....ratioMatch...." << ratioMatch << std::endl;
						}
						break;
					}
					case TORUS: {
						TorusParams *torusDesired = dynamic_cast<TorusParams*>
							(shapeDesired.preferredPrimitives.at(primNo).params);
						TorusParams *torusCandidate = dynamic_cast<TorusParams*>
							(shapeCandidate.primitives.at(0).params);
						if (torusDesired != NULL && torusCandidate != NULL){
							float desiredMinRadius = torusDesired->minRadius;
							float detectedMinRadius = torusCandidate->minRadius;
							float desiredMaxRadius = torusDesired->maxRadius;
							float detectedMaxRadius = torusCandidate->maxRadius;
							float desiredRatio = desiredMaxRadius / desiredMinRadius;
							float detectedRatio = detectedMaxRadius / detectedMinRadius;
							float minRadiusMatch;
							(desiredMinRadius >= detectedMinRadius) ? (minRadiusMatch = detectedMinRadius / desiredMinRadius)
								: (minRadiusMatch = desiredMinRadius / detectedMinRadius);
							float maxRadiusMatch;
							(desiredMaxRadius >= detectedMaxRadius) ? (maxRadiusMatch = detectedMaxRadius / desiredMaxRadius)
								: (maxRadiusMatch = desiredMaxRadius / detectedMaxRadius);
							float ratioMatch;
							(desiredRatio >= detectedRatio) ? (ratioMatch = detectedRatio / desiredRatio)
								: (ratioMatch = desiredRatio / detectedRatio);
							primSpecificVote = primSpecificVote*
								(minRadiusMatch*torusDesired->minRadiusPref +
								maxRadiusMatch*torusDesired->maxRadiusPref +
								ratioMatch*torusDesired->ratioPref);
							//std::cout << ".....desiredMaxRadius >= detectedMaxRadius....." << desiredMaxRadius << "  " << detectedMaxRadius << std::endl;
							//std::cout << ".....desiredMinRadius >= detectedMinRadius....." << desiredMinRadius << "  " << detectedMinRadius << std::endl;
							//std::cout << ".....desiredRatio >= detectedRatio....." << desiredRatio << "  " << detectedRatio << std::endl;
							//std::cout << ".....heightMatch....." << minRadiusMatch << std::endl;
							//std::cout << ".....anglematch....." << maxRadiusMatch << std::endl;
							//std::cout << ".....ratioMatch...." << ratioMatch << std::endl;
						}
						break;
					}
					default:
						primSpecificVote = 0;
					}
					finalVote = finalVote + primSpecificVote;
					break; //stop comparing primitives
				}
			}
		}
		_voteMatrix(shapeNo, clusterNo) = finalVote;
	}

#if 0// debug: print detected shape
	switch (shapeCandidate.primitives.at(0).type){
	case PLANE: std::cout << green << "                    PLANE" << white << std::endl; break;
	case SPHERE: std::cout << green << "                    SPHERE" << white << std::endl; break;
	case CYLINDER: std::cout << green << "                    CYLINDER" << white << std::endl; break;
	case CONE: std::cout << green << "                    CONE" << white << std::endl; break;
	case TORUS: std::cout << green << "                    TORUS" << white << std::endl; break;
	default: std::cout << green << "                    UNKNOWN" << white << std::endl;
	}
#endif
}

void ShapeDetector::match(){
	std::vector<long> assignment = dlib::max_cost_assignment(_voteMatrix);
	int detections = 0;
	_shapeToClusterMapping.clear();
	_shapeToClusterMapping.resize(0);
	for (int shapeNo = 0; shapeNo < _desiredShapes.size(); shapeNo++){
		if ((_desiredShapes.at(shapeNo).jumpAllowed || _desiredShapes.at(shapeNo).model.size() <= 0) &&
			assignment.at(shapeNo) < _clusters.size() && _voteMatrix(shapeNo, assignment.at(shapeNo)) > 0){
			_shapeToClusterMapping.push_back(assignment.at(shapeNo));
			detections++;
		}
		else{
			_shapeToClusterMapping.push_back(-1);
		}
	}

#if SHAPEVERBOSE	//debug: print vote matrix
	for (int r = 0; r < _voteMatrixSize; r++){
		for (int c = 0; c < _voteMatrixSize; c++){
			std::cout << yellow << _voteMatrix(r, c) << " ";
		}
		std::cout << white << std::endl;
	}
#endif

#if SHAPEVERBOSE	// debug: print shapeToCluster indices
	for (int i = 0; i < _shapeToClusterMapping.size(); i++)
		std::cout << red << _shapeToClusterMapping.at(i) << " ";
	std::cout << white << std::endl;
#endif

	std::lock_guard<std::mutex> lock(_getNoOfDetectionsKey);
	_noOfDetections = detections;
}

void ShapeDetector::updateresults(){
	_allShapesDetected =
		std::find(_shapeToClusterMapping.begin(), _shapeToClusterMapping.end(), -1) == _shapeToClusterMapping.end();

	//_results.desiredShapes = _desiredShapes;
	//_results.clusters = _clusters;
	//_results.detectedShapes = _detectedShapes;
	//_results.shapeToClusterMapping = _shapeToClusterMapping;

	int shapeClusterIndex = -1;
	_getResultsKey.lock();
		_resultsFetchedTemp = _resultsFetched;
	_getResultsKey.unlock();
	_desiredShapesTemp = _desiredShapes;
	for (int shapeNo = 0; shapeNo < _desiredShapes.size(); shapeNo++){
		_desiredShapesTemp.at(shapeNo).hasNewModel = false;
		if (_resultsFetchedTemp)
			_desiredShapes.at(shapeNo).hasNewModel = false;
		shapeClusterIndex = _shapeToClusterMapping.at(shapeNo);
		if (shapeClusterIndex != -1
			&& (_desiredShapes.at(shapeNo).model.size() <= 0
			|| quickAlign(_desiredShapes.at(shapeNo).model, _clusters.at(shapeClusterIndex)) > SIMILARITY_THRESHOLD)){
				_desiredShapes.at(shapeNo).hasNewModel = true;
				_desiredShapesTemp.at(shapeNo).hasNewModel = true;
				_desiredShapes.at(shapeNo).model = _clusters.at(shapeClusterIndex);
				_desiredShapesTemp.at(shapeNo).model = _clusters.at(shapeClusterIndex);
				_desiredShapes.at(shapeNo).matchedPrimitives = _detectedShapes.at(shapeClusterIndex).primitives;
				_desiredShapesTemp.at(shapeNo).matchedPrimitives = _detectedShapes.at(shapeClusterIndex).primitives;
				#if 0
					std::cout << "Updating model of shape (array number) " << shapeNo
						<< " to " << _desiredShapes.at(shapeNo).matchedPrimitives.at(0).type << std::endl;
				#endif
		}
	}

	std::lock_guard<std::mutex> lock(_getResultsKey);
	if (_resultsFetchedTemp != _resultsFetched)
		_desiredShapes = _desiredShapesTemp;
	_resultShapes = _desiredShapes;

	_resultsFetched = false;

	if (_noOfDetections > 0){
		//std::cout << yellow << "SHAPEDETECTOR: NOTIFY ALL: no of detections " << _noOfDetections << white << std::endl;
		condNewResults.notify_all();
	}
}

void ShapeDetector::initVoteMatrix(){
	_voteMatrixSize = std::max(_desiredShapes.size(), _clusters.size());
	_voteMatrix = dlib::matrix<int>(_voteMatrixSize, _voteMatrixSize);
	for (int r = 0; r < _voteMatrixSize; r++)
		for (int c = 0; c < _voteMatrixSize; c++)
			_voteMatrix(r, c) = 0;
	#ifdef SHAPEVERBOSE
		std::cout << "Vote martix size: " << _voteMatrix.size() << std::endl;
	#endif
}

std::vector<Shape> ShapeDetector::getResults(){
	std::lock_guard<std::mutex> lock(_getResultsKey);
	_resultsFetched = true;
	std::vector<Shape> resultShapesTemp = _resultShapes;
	for (int shapeNo = 0; shapeNo < _desiredShapes.size(); shapeNo++){
		_resultShapes.at(shapeNo).hasNewModel = false;
	}
	return resultShapesTemp;
}

int ShapeDetector::getNoOfDetections(){
	std::lock_guard<std::mutex> lock(_getNoOfDetectionsKey);
	return _noOfDetections;
}

void ShapeDetector::wake(){ // Call only when thread is waiting?? Make thread safe?
	_allShapesDetected = false;
	_condShapesFound.notify_one();
}

void ShapeDetector::pause(){
	_pauseThread = true;
}

void ShapeDetector::resume(){
	_pauseThread = false;
	_condPauseRequest.notify_one();
}

void ShapeDetector::stop(){
	_stopThread = true;
}

#endif