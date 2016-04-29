#include "stdafx.h"
#include "sensorReader.h"
#include "shapeDetector.h"
#include "featureCalculator.h"
#include "initialAligner.h"
#include "modelSegmenter.h"
#include "poseTracker.h"
#include <conio.h>
#include <iomanip> 

#define WAIT_TO_RESET_MODEL 20

typedef pcl::PointCloud<pcl::VFHSignature308> GlobalFeature;
const int histSize = 308;


pcl::ScopeTime globaltimer("global timer");
double globaltime;

std::vector<Shape> shapes;
SacPrerejectiveAligner** aligners;
ModelSegmentor** targetSegmenters;
//IcpTracker** trackers;

MySensor* myKinect;
ShapeDetector* shapeDetector;
FeatureCalculator* targetProcessor;

// Model feature calculator
FeatureCalculator sourceProcessor;
FeatureCalculator::FeatureCloud modelFormFileProcessed;

// Model dimensions
float modelHeight, modelWidth, modelDepth, modelRadius;
// Model origin
Eigen::Vector4f origin;
// Model bounding points: to calculate dimensions
Eigen::Vector4f min, max;





inline void savePcd(pcl::PointCloud<pcl::PointXYZ> pc){
	std::thread pcdWritingThread([=](){
		try{
			pcl::io::savePCDFileASCII("objects\\model.pcd", pc);
			std::cout << "Frame saved. ("
				<< " Size: " << pc.size()
				<< ", Width: " << pc.width
				<< ", Height: " << pc.height
				<< " )" << std::endl;
		}
		catch (...){
			std::cerr << "Saving error.." << std::endl;
		}
	});
	pcdWritingThread.detach();
}

inline pcl::PointCloud<pcl::PointXYZRGBNormal> loadPcd(std::string file){
	pcl::PointCloud<pcl::PointXYZRGBNormal> pc;
	if (pcl::io::loadPCDFile(file, pc) == 0){
		return pc;
	}
	else{
		std::cerr << "Error loading file. Empty cloud returned." << std::endl;
		return pcl::PointCloud<pcl::PointXYZRGBNormal>();
	}
}

inline void simpleVis(pcl::PointCloud<pcl::PointXYZ> pc){
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZ>(pc));
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	//viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	viewer.spin();
	viewer.close();
}

inline void rgbVis(pcl::PointCloud<pcl::PointXYZRGB> pc){
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(pc));
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	viewer.spin();
	viewer.close();
}

inline void normalsVis(pcl::PointCloud<pcl::PointXYZ> pcXyz, pcl::PointCloud<pcl::Normal> pcNormal){
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZ>(pcXyz));
	pcl::PointCloud<pcl::Normal>::ConstPtr normals(new pcl::PointCloud<pcl::Normal>(pcNormal));
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer.initCameraParameters();
	viewer.spin();
	viewer.close();
}

inline void rgbNormalsVis(pcl::PointCloud<pcl::PointXYZRGB> pcXyzRgb, pcl::PointCloud<pcl::Normal> pcNormal){
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(pcXyzRgb));
	pcl::PointCloud<pcl::Normal>::ConstPtr normals(new pcl::PointCloud<pcl::Normal>(pcNormal));
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();
	viewer.spin();
	viewer.close();
}

inline std::vector<pcl::IndicesPtr> objectSegmenter(pcl::PointCloud<pcl::PointXYZ> pc){
	pcl::IndicesPtr _segmentedPcIndices1(new std::vector<int>);
	pcl::IndicesPtr _segmentedPcIndices2(new std::vector<int>);
	pcl::IndicesPtr _segmentedPcIndices3(new std::vector<int>);
	pcl::IndicesConstPtr _segmentedPcIndices4(new std::vector<int>);
	pcl::IndicesPtr _filteredPcIndices(new std::vector<int>);

	pcl::PassThrough<pcl::PointXYZ> passThrough;
	passThrough.setFilterFieldName("z");
	passThrough.setFilterLimits(0, 1.0f);
	passThrough.setInputCloud(pc.makeShared());
	passThrough.setFilterLimits(0.0f, 1.0f);
	passThrough.filter(*_segmentedPcIndices1);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(pc.makeShared());
	seg.setIndices(_segmentedPcIndices1);
	seg.segment(*inliers, *coefficients);
	_segmentedPcIndices2 = boost::make_shared<std::vector<int>>(inliers->indices);

	if (_segmentedPcIndices2->size() <= 0){
		return std::vector<pcl::IndicesPtr>{};
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(pc.makeShared());
	chull.setIndices(_segmentedPcIndices2);
	chull.setDimension(2);
	chull.reconstruct(*cloud_hull);
	if (cloud_hull != NULL && cloud_hull->size() > 0)
		cloud_hull->push_back(cloud_hull->at(0));

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(pc.makeShared());
	extract.setIndices(_segmentedPcIndices2);
	extract.setNegative(false);
	extract.setKeepOrganized(true);
	pcl::PointCloud<pcl::PointXYZ> outliers;
	extract.filter(outliers);
	_segmentedPcIndices4 = extract.getRemovedIndices();

	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
	prism.setInputCloud(pc.makeShared());
	prism.setIndices(_segmentedPcIndices4);
	prism.setInputPlanarHull(cloud_hull);
	prism.setHeightLimits(0.005, 10);
	pcl::PointIndices::Ptr inl(new pcl::PointIndices);
	prism.segment(*inl);
	_segmentedPcIndices3 = boost::make_shared<std::vector<int>>(inl->indices);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statOutlierRemovel;
	statOutlierRemovel.setMeanK(50);
	statOutlierRemovel.setStddevMulThresh(0.2f);
	statOutlierRemovel.setInputCloud(pc.makeShared());
	statOutlierRemovel.setIndices(_segmentedPcIndices3);
	statOutlierRemovel.filter(*_filteredPcIndices);

	std::vector<pcl::IndicesPtr> ptrs;
#if 1
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.02);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(pc.makeShared());
	ec.setIndices(_filteredPcIndices);
	ec.extract(cluster_indices);

	for each (pcl::PointIndices indices in cluster_indices)
	{
		ptrs.push_back(boost::make_shared<std::vector<int>>(indices.indices));
	}
#else
	ptrs.push_back(_filteredPcIndices);
#endif

	return ptrs;
}

inline pcl::IndicesPtr sacModelDetector(pcl::PointCloud<pcl::PointXYZ> pc){
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setInputCloud(pc.makeShared());
	normalEstimation.setRadiusSearch(0.02f);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	pcl::IndicesPtr indices(new std::vector<int>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CONE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setProbability(0.99);
	seg.setNormalDistanceWeight(0.1);
	seg.setDistanceThreshold(0.02);
	seg.setMinMaxOpeningAngle(0, 0.5);
	seg.setMaxIterations(10000);
	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(pc.makeShared());
	seg.setInputNormals(normals);
	seg.segment(*inliers, *coefficients);

	if (coefficients->values.size() == 4){
		std::cout << green << "Coefficients: " << "("
			<< coefficients->values.at(0) << ", "
			<< coefficients->values.at(1) << ", "
			<< coefficients->values.at(2) << ") "
			<< coefficients->values.at(3)
			<< white << std::endl;
	}
	
	indices = boost::make_shared<std::vector<int>>(inliers->indices);
	return indices;
}

inline GlobalFeature globalFeatureCalc(pcl::PointCloud<pcl::PointXYZ> object){
	GlobalFeature descriptor;
	if (object.size() > 0){
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setInputCloud(object.makeShared());
		normalEstimation.setRadiusSearch(0.04f);
		normalEstimation.setSearchMethod(kdtree);
		normalEstimation.compute(*normals);

#if 1	//VFH
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> estimator;
		estimator.setFillSizeComponent(true);
		estimator.setNormalizeBins(true);
		estimator.setNormalizeDistance(true);
#elif 0	//OUR-CVFH
		pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> estimator;
		estimator.setEPSAngleThreshold(180.0 / 180.0 * M_PI); // 5 degrees.
		estimator.setCurvatureThreshold(10000.0);
		estimator.setAxisRatio(0.8);
#elif 0 //ESF
		pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> estimator;

#elif 0	//FPFH
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> estimator;
		estimator.setRadiusSearch(0.1f);
#elif 0	//PFH
		pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> estimator;
		estimator.setRadiusSearch(0.1f);
#endif

#if 1 // For local decriptors, push centroid and set indices
		//Eigen::Vector4f centroidCoords;
		//pcl::compute3DCentroid(object, centroidCoords);
		//float normalLength = std::sqrtf(std::powf(centroidCoords.x(), 2.0) + std::powf(centroidCoords.y(), 2.0) + std::powf(centroidCoords.z(), 2.0));
		//pcl::PointXYZ centroid(centroidCoords.x(), centroidCoords.y(), centroidCoords.z());
		//pcl::Normal normal(0 - centroidCoords.x() / normalLength, 0 - centroidCoords.y() / normalLength, 0 - centroidCoords.z() / normalLength);
		//object.insert(object.begin(), centroid);
		//normals->insert(normals->begin(), normal);
		//std::cout << "Point: " << object.at(0).x << ", " << object.at(0).y << ", " << object.at(0).z << std::endl;
		//std::cout << "Normal: " << std::powf(normals->at(0).normal_x, 2.0) + std::powf(normals->at(0).normal_y, 2.0) + std::powf(normals->at(0).normal_z, 2.0) << std::endl;
	
		pcl::IndicesPtr pt(boost::shared_ptr<std::vector<int>>(new std::vector < int > { 500 }));
		//pcl::PointCloud<pcl::PointXYZ> pc;
		//pcl::copyPointCloud(object, *pt, pc);

		//std::cout << "Copied Point: " << pc.at(0).x << ", " << pc.at(0).y << ", " << pc.at(0).z << std::endl;
		//estimator.setIndices(pt);
		//estimator.setInputCloud(pc.makeShared());
		//estimator.setSearchSurface(object.makeShared());
#endif
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointXYZ>);
		estimator.setInputCloud(object.makeShared());
		estimator.setInputNormals(normals);
		estimator.setSearchMethod(kdtree2);
		estimator.compute(descriptor);
	}
	return descriptor;
}

inline void debugfuncs(pcl::PointCloud<pcl::PointXYZ> pc, int &F1, int &F5){
	if (GetKeyState(VK_F1) < 0 && F1){
		savePcd(pc);
		F1 = 0;
	} else if (GetKeyState(VK_F1) >= 0){
		F1 = 1;
	}
	if (GetKeyState(VK_F5) < 0 && F5){
		pcl::PointCloud<pcl::PointXYZRGBNormal> pcXyzRgbNormal = loadPcd("objects\\model.pcd");
		pcl::PointCloud<pcl::PointXYZRGB> pcXyzRgb;
		pcl::PointCloud<pcl::PointXYZ> pcXyz;
		pcl::copyPointCloud(pcXyzRgbNormal, pcXyzRgb);
		pcl::copyPointCloud(pcXyzRgbNormal, pcXyz);
		std::thread(simpleVis, pcXyz).detach();
		F5 = 0;
	} else if (GetKeyState(VK_F5) >= 0){
		F5 = 1;
	}
}

inline void copyShapeModels(std::vector<Shape> &resultShapes){
	for (int shapeIndex = 0; shapeIndex < shapes.size(); shapeIndex++){
		shapes.at(shapeIndex).hasNewModel = false;
		if (resultShapes.at(shapeIndex).hasNewModel){
			Eigen::Vector4f origin;
			pcl::compute3DCentroid(resultShapes.at(shapeIndex).model, origin);
			Eigen::Vector4f primOrigin(origin);
			// Calculate initial rotation of shape (Rodrigues' rotation formula)
			Eigen::Vector3f y0(0.0, 1.0, 0.0), y1(0.0, 1.0, 0.0);
			std::vector<Primitive> prims = resultShapes.at(shapeIndex).matchedPrimitives;
			if (prims.size() > 0){ // Must always be true; Cluster must have primitives if a shape has voted it
				Primitive salientPrim = prims.at(0); // Only consider orientation of the most salient primitive shape
				switch (salientPrim.type){
				case PLANE:{
					primOrigin.block<3, 1>(0, 0) = static_cast<PlaneParams*>(salientPrim.params)->center;
					y0 = Eigen::Vector3f(0.0, 0.0, -1.0);
					y1 = static_cast<PlaneParams*>(salientPrim.params)->normal;
					break;
				}
				case SPHERE:{
					primOrigin.block<3, 1>(0, 0) = static_cast<SphereParams*>(salientPrim.params)->center;
						break;
				}
				case CYLINDER:{
					primOrigin.block<3, 1>(0, 0) = static_cast<CylinderParams*>(salientPrim.params)->axisPosition;
					y1 = static_cast<CylinderParams*>(salientPrim.params)->axisDirection;
					y1[1] < 0 ? y0 = -y0 : y0;
					break;
				}
				case CONE:{
					primOrigin.block<3, 1>(0, 0) = static_cast<ConeParams*>(salientPrim.params)->center;
					y1 = -static_cast<ConeParams*>(salientPrim.params)->axisDirection;
					break;
				}
				case TORUS:{
					primOrigin.block<3, 1>(0, 0) = static_cast<TorusParams*>(salientPrim.params)->center;
					y1 = static_cast<TorusParams*>(salientPrim.params)->axisDirection;
					y1[1] < 0 ? y0 = -y0 : y0;
					break;
				}
				default:
					break;
				}

				//std::cout << salientPrim.type << " origin " << origin.transpose() << std::endl;
				//std::cout << salientPrim.type << " prim origin " << primOrigin.transpose() << std::endl;
			}

			resultShapes.at(shapeIndex).objToPrimOrigin = primOrigin - origin;
			
			y0.normalize();
			y1.normalize();

			Eigen::Vector3f axis = y1.cross(y0);
			float rcos = y1.dot(y0);
			float rsin = axis.norm();
			if (rsin != 0)
				axis.normalize();

			Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();
			Eigen::Matrix4f backTranslation = Eigen::Matrix4f::Identity();
			Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

			float u(axis(0)), v(axis(1)), w(axis(2));
			rot(0, 0) = rcos + u*u*(1 - rcos);
			rot(1, 0) = w * rsin + v*u*(1 - rcos);
			rot(2, 0) = -v * rsin + w*u*(1 - rcos);
			rot(0, 1) = -w * rsin + u*v*(1 - rcos);
			rot(1, 1) = rcos + v*v*(1 - rcos);
			rot(2, 1) = u * rsin + w*v*(1 - rcos);
			rot(0, 2) = v * rsin + u*w*(1 - rcos);
			rot(1, 2) = -u * rsin + v*w*(1 - rcos);
			rot(2, 2) = rcos + w*w*(1 - rcos);

			backTranslation(0, 3) = -origin(0);
			backTranslation(1, 3) = -origin(1);
			backTranslation(2, 3) = -origin(2);

			translation(0, 3) = origin(0);
			translation(1, 3) = origin(1);
			translation(2, 3) = origin(2);

			//std::cout << green << axis.matrix().transpose() << white << std::endl;

			transform = translation*rot*backTranslation;

			shapes.at(shapeIndex).rot = rot;
			shapes.at(shapeIndex).initTrans = transform;

			pcl::transformPointCloud(resultShapes.at(shapeIndex).model, shapes.at(shapeIndex).model, transform);
			shapes.at(shapeIndex).objToPrimOrigin = rot*resultShapes.at(shapeIndex).objToPrimOrigin;
			shapes.at(shapeIndex).matchedPrimitives = resultShapes.at(shapeIndex).matchedPrimitives;
			shapes.at(shapeIndex).hasNewModel = true;

			//std::cout << "Origin vec before " << resultShapes.at(shapeIndex).objToPrimOrigin.transpose() << std::endl;
			//std::cout << "Origin vec after  " << shapes.at(shapeIndex).objToPrimOrigin.transpose() << std::endl;
		}
	}
}

void updateTracking(){
	// Copy newly detected shapes (clusters)
	copyShapeModels(shapeDetector->getResults());

	//For each shape, set new shape models (clusters), and resume tracking pipeline
	for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
		if (!shapes.at(shapeId).hasNewModel){
			if (shapes.at(shapeId).model.size() > 0){
				continue;
			}
			else{
				aligners[shapeId]->pause();
				targetSegmenters[shapeId]->pause();
				shapes.at(shapeId).tracker->pause();
				continue;
			}
		}

		aligners[shapeId]->pause();
		targetSegmenters[shapeId]->pause();
		shapes.at(shapeId).tracker->pause();

		// Get origin
		pcl::compute3DCentroid(shapes.at(shapeId).model, origin);
		pcl::PointCloud<pcl::PointXYZ> originCloud(1, 1, pcl::PointXYZ(origin.x(), origin.y(), origin.z()));

		// Get dimensions
		pcl::getMinMax3D(shapes.at(shapeId).model, min, max);
		modelWidth = std::fabs(max.x() - min.x());
		modelHeight = std::fabs(max.y() - min.y());
		modelDepth = std::fabs(max.z() - min.z());
		modelRadius = std::max(modelHeight, modelWidth) / 2.0;

		// Process model 
		sourceProcessor.setInputCloud(shapes.at(shapeId).model);
		sourceProcessor.filterAndFeatureCalc();
		modelFormFileProcessed = sourceProcessor.getCurrentFeatureCloud();

		if (modelFormFileProcessed.localFeatures.size() <= 0)
			continue;

		// Set models in aligners
		aligners[shapeId]->setSources(modelFormFileProcessed);
		aligners[shapeId]->resume();

		// Set params in segmenters
		targetSegmenters[shapeId]->setModelDimensions(std::vector<float>{modelWidth, modelHeight, modelDepth, modelRadius});
		targetSegmenters[shapeId]->setModelOrigin(originCloud);
		targetSegmenters[shapeId]->resume();

		// Restart trackers ???
		//pcl::PointCloud<pcl::PointXYZ> model;
		//Eigen::Matrix4f inv =  shapes.at(shapeId).initTrans.inverse();
		//pcl::transformPointCloud(shapes.at(shapeId).model, model, inv);
		//shapes.at(shapeId).tracker->setSources(model, shapes.at(shapeId).rot);
		shapes.at(shapeId).tracker->resume();

		std::cout << "Shape " << shapeId << " tracking model updated" << std::endl;
	}
}

/*****************  Core functions  ********************/
int setShapes(const std::vector<Shape> &_shapes){
	shapes = std::vector<Shape>(_shapes);
	aligners = new SacPrerejectiveAligner*[shapes.size()];
	targetSegmenters = new ModelSegmentor*[shapes.size()];
	myKinect = new MySensor();
	shapeDetector = new ShapeDetector();
	targetProcessor = new FeatureCalculator();

	if (myKinect->initSesor() == -1){
		return -1;
	}

	shapeDetector->setInputCloudStream(myKinect);
	shapeDetector->setShapes(shapes);

	targetProcessor->setInputCloudStream(myKinect);

	// Initialize and temporarilty start all tracking threads
	for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
		// Thread: allocate Initial Aligners
		aligners[shapeId] = new SacPrerejectiveAligner();
		aligners[shapeId]->setTargetStream(targetProcessor);
		std::thread(&SacPrerejectiveAligner::startAligning, aligners[shapeId]).detach();
		aligners[shapeId]->pause();

		// Thread: allocate Model segmenters
		targetSegmenters[shapeId] = new ModelSegmentor();
		targetSegmenters[shapeId]->setInputCloudStream(myKinect);
		targetSegmenters[shapeId]->setInitAligner(aligners[shapeId]);
		std::thread(&ModelSegmentor::startSegmenting, targetSegmenters[shapeId]).detach();
		targetSegmenters[shapeId]->pause();

		// Thread: allocate Trackers
		shapes.at(shapeId).tracker->setTargetStream(targetSegmenters[shapeId]);
		shapes.at(shapeId).tracker->setInitAligner(aligners[shapeId]);
		std::thread(&IcpTracker::startTracking, shapes.at(shapeId).tracker).detach();
		shapes.at(shapeId).tracker->pause();
	}
	return 0;
}

void startDetection(){
	// Start reading
	std::thread(&MySensor::startReading, std::ref(*myKinect)).detach();

	std::thread(&ShapeDetector::startMatching, std::ref(*shapeDetector)).detach();

	std::thread(&FeatureCalculator::startProcessing, std::ref(*targetProcessor)).detach();

	// Wait for detected shapes
	shapeDetector->condNewResults.wait(std::unique_lock<std::mutex>(std::mutex()));
	//std::this_thread::sleep_for(std::chrono::seconds(5));
	
	updateTracking();
}

void stopDetection(){
	for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
		shapes.at(shapeId).tracker->stop();
		targetSegmenters[shapeId]->stop();
		aligners[shapeId]->stop();
	}
	targetProcessor->stop();
	shapeDetector->stop();
	myKinect->stop();
}

void restartDetection(){
	for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
		shapes.at(shapeId).tracker->pause();
		targetSegmenters[shapeId]->pause();
		aligners[shapeId]->pause();
	}
	shapeDetector->pause();
	for (int shapeID = 0; shapeID < shapes.size(); shapeID++){
		shapes.at(shapeID).resetModel();
	}
	shapeDetector->setShapes(shapes);
	shapeDetector->resume();

	shapeDetector->condNewResults.wait(std::unique_lock<std::mutex>(std::mutex()));
}

std::vector<Pose> getPoses(){
	std::vector<Pose> poses(shapes.size());
	TrackingResults trackResults;
	Pose pose;

	//Eigen::Vector4f translation;

	for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
		trackResults = shapes.at(shapeId).tracker->getResults();
		//std::cout << yellow << "Shape " << shapeId << "  succeeded.. ?" << trackResults.trackingSucceeded << white << std::endl;
		if (trackResults.trackingSucceeded){
			shapes.at(shapeId).lostTrackCount = 0;
			//// Translation
			//pcl::compute3DCentroid(trackResults.output, translation);
			//std::cout << "Timer " << timer.getTime() << std::endl;
			//// Rotation
			//Eigen::Quaternionf qRotation(trackResults.finalTransformation.topLeftCorner<3, 3>());

			//pose.x = translation[0];
			//pose.y = translation[1];
			//pose.z = translation[2];
			//pose.qX = qRotation.x();
			//pose.qY = qRotation.y();
			//pose.qZ = qRotation.z();
			//pose.qW = qRotation.w();

			pose.qX = trackResults.quaternion[0];
			pose.qY = trackResults.quaternion[1];
			pose.qZ = trackResults.quaternion[2];
			pose.qW = trackResults.quaternion[3];

			Eigen::Vector3f primOriginShift = Eigen::Quaternionf(pose.qW, pose.qX, pose.qY, pose.qZ)._transformVector
				(shapes.at(shapeId).objToPrimOrigin.head<3>());

			pose.x = trackResults.translation[0] + primOriginShift.x();
			pose.y = trackResults.translation[1] + primOriginShift.y();
			pose.z = trackResults.translation[2] + primOriginShift.z();

		}else{
			pose.x = pose.y = pose.z = pose.qX = pose.qY = pose.qZ = pose.qW = 0.0;
		}

		if (!trackResults.trackingSucceeded && (shapes.at(shapeId).jumpAllowed || shapes.at(shapeId).model.size() <= 0)){
			if (shapes.at(shapeId).lostTrackCount > WAIT_TO_RESET_MODEL){
				//std::cout << yellow << "Shape " << shapeId << "Lost tracking.. " << shapes.at(shapeId).lostTrackCount << white << std::endl;
				//pcl::ScopeTime time;
				updateTracking();
				//std::cout << yellow << "Update Time " << time.getTime() << "ms" << white << std::endl;
			}else{
				shapes.at(shapeId).lostTrackCount++;
				//std::cout << green << "Shape " << shapeId << " lost tracking count increasing.. " << shapes.at(shapeId).lostTrackCount << white << std::endl;
			}
		}

		poses.at(shapeId) = pose;

		//std::cout << yellow << "Trans " << "(" << pose.x << ", " << pose.y << ", " << pose.z << ")" << white << std::endl;
		//std::cout << yellow << "Rot   " << "(" << pose.qX << ", " << pose.qY << ", " << pose.qZ << ", " << pose.qW << ")" << white << std::endl;
		//Eigen::AngleAxisf angleAxis(qRotation);
		//std::cout << yellow << "Angle " << angleAxis.angle()*(180/M_PI) << white << std::endl;
		//std::cout << std::endl;
	}
	
	return poses;
}




#if 0
inline std::list<pcl::recognition::ObjRecRANSAC::Output> ransacObjRec(pcl::PointCloud<pcl::PointXYZ> object, pcl::PointCloud<pcl::PointXYZ> scene){
	pcl::PointCloud<pcl::Normal>::Ptr objectnormals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr scenenormals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setRadiusSearch(0.01f);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.setInputCloud(object.makeShared());
	normalEstimation.compute(*objectnormals);
	normalEstimation.setInputCloud(scene.makeShared());
	normalEstimation.compute(*scenenormals);

	pcl::recognition::ObjRecRANSAC ransac(0.05, 0.005);
	ransac.setMaxCoplanarityAngleDegrees(15.0f);
	ransac.clear();
	ransac.addModel(object, *objectnormals, "obj");

	std::list<pcl::recognition::ObjRecRANSAC::Output> output;

	ransac.recognize(scene, *scenenormals, output);

	return output;
}
#endif

/*****************  Exposed functions  ********************/

extern "C"{
	__declspec(dllexport) int setShapes(char* shapes_){
		std::vector<Shape> shapes;

		Json::Reader jReader;
		Json::Value jObj;
		Json::Value jShapes(Json::arrayValue);

		int objPref;
		bool jump;
		Json::Value jPrimitives(Json::arrayValue);

		jReader.parse(shapes_, jObj);
		jShapes = jObj["Shapes"];

		for each (Json::Value jShape in jShapes){
			objPref = jShape["objectPref"].asInt();
			jump = jShape["jumpAllowed"].asBool();
			//std::cout << "objPref: " << objPref << std::endl;
			//std::cout << "jump: " << jump << std::endl;
			jPrimitives = jShape["Primitives"];
			std::vector<Primitive> primitives;
			for each (Json::Value jPrimitive in jPrimitives){
				std::string primShape = jPrimitive["PrimShape"].asString();
				Json::Value jBaseTypeParams = jPrimitive["BaseTypeParams"];
				float primPref = jBaseTypeParams["primitivePref"].asFloat();
				if (primShape.compare("PLANE") == 0){
					float width = jBaseTypeParams["width"].asFloat();
					float length = jBaseTypeParams["length"].asFloat();
					float widthPref = jBaseTypeParams["widthPref"].asFloat();
					float lengthPref = jBaseTypeParams["lengthPref"].asFloat();
					float ratioPref = jBaseTypeParams["ratioPref"].asFloat();
					//std::cout << "width: " << width << " length: " << length << " widthPref: " << widthPref << " lengthPref: " << lengthPref << " ratioPref" << ratioPref << std::endl;
					primitives.push_back(Primitive(PLANE,
						new PlaneParams(width, length, primPref, widthPref, lengthPref, ratioPref)));
				}
				else if (primShape.compare("SPHERE") == 0){
					float radius = jBaseTypeParams["radius"].asFloat();
					float radiusPref = jBaseTypeParams["radiusPref"].asFloat();
					//std::cout << "radius: " << radius << " radiusPref: " << radiusPref << std::endl;
					primitives.push_back(Primitive(SPHERE,
						new SphereParams(radius, primPref, radiusPref)));
				}
				else if (primShape.compare("CYLINDER") == 0){
					float radius = jBaseTypeParams["radius"].asFloat();
					float height = jBaseTypeParams["height"].asFloat();
					float radiusPref = jBaseTypeParams["radiusPref"].asFloat();
					float heightPref = jBaseTypeParams["heightPref"].asFloat();
					float ratioPref = jBaseTypeParams["ratioPref"].asFloat();
					//std::cout << "radius: " << radius << " height: " << height << " radiusPref: " << radiusPref << " heightPref: " << heightPref << " ratioPref" << ratioPref << std::endl;
					primitives.push_back(Primitive(CYLINDER,
						new CylinderParams(radius, height, primPref, radiusPref, heightPref, ratioPref)));
				}
				else if (primShape.compare("CONE") == 0){
					float angle = jBaseTypeParams["angle"].asFloat();
					float height = jBaseTypeParams["height"].asFloat();
					float anglePref = jBaseTypeParams["anglePref"].asFloat();
					float heightPref = jBaseTypeParams["heightPref"].asFloat();
					float ratioPref = jBaseTypeParams["ratioPref"].asFloat();
					//std::cout << "angle: " << angle << " height: " << height << " anglePref: " << anglePref << " heightPref: " << heightPref << " ratioPref" << ratioPref << std::endl;
					primitives.push_back(Primitive(CONE,
						new ConeParams(angle, height, primPref, anglePref, heightPref, ratioPref)));
				}
				else if (primShape.compare("TORUS") == 0){
					float minRadius = jBaseTypeParams["minRadius"].asFloat();
					float maxRadius = jBaseTypeParams["maxRadius"].asFloat();
					float minRadiusPref = jBaseTypeParams["minRadiusPref"].asFloat();
					float maxRadiusPref = jBaseTypeParams["maxRadiusPref"].asFloat();
					float ratioPref = jBaseTypeParams["ratioPref"].asFloat();
					//std::cout << "minRadius: " << minRadius << " maxRadius: " << maxRadius << " minRadiusPref: " << minRadiusPref << " lengthPref: " << maxRadiusPref << " maxRadiusPref" << ratioPref << std::endl;
					primitives.push_back(Primitive(TORUS,
						new TorusParams(minRadius, maxRadius, primPref, minRadiusPref, maxRadiusPref, ratioPref)));
				}
				else{
				}
			}
			shapes.push_back(Shape(primitives, objPref, jump));
		}

		int setupCode = setShapes(shapes);
		return setupCode;
	}
}

extern "C"{
	__declspec(dllexport) void start(){
		startDetection();
	}
}

extern "C"{
	__declspec(dllexport) int getPoses(int shapeCount, int* shapeTypes, float* dims, float* trans, float* rot){
		std::vector<Pose> poses;
		poses = getPoses();
		if (shapeCount != poses.size()){
			std::cerr << "ERROR: Shape count mismatch.." << std::endl;
			return -1;
		}
		for (int shapeId = 0; shapeId < shapeCount; shapeId++){
			shapeTypes[shapeId] = -1;
			dims[2 * shapeId + 0] = 0.0; dims[2 * shapeId + 1] = 0.0;
			if (shapes.at(shapeId).matchedPrimitives.size() > 0){
				Primitive salientPrim = shapes.at(shapeId).matchedPrimitives.at(0);
				shapeTypes[shapeId] = salientPrim.type;
				//std::cout << std::endl << "Type getPoses " << salientPrim.type << std::endl;
				switch (shapeTypes[shapeId]){
				case PLANE:{
					PlaneParams* planeParams =  static_cast<PlaneParams*>(salientPrim.params);
					dims[2 * shapeId + 0] = planeParams->width; dims[2 * shapeId + 1] = planeParams->length;
					break;
				}
				case SPHERE:{
					SphereParams* sphereParams = static_cast<SphereParams*>(salientPrim.params);
					dims[2 * shapeId + 0] = sphereParams->radius; dims[2 * shapeId + 1] = sphereParams->radius;
					break;
				}
				case CYLINDER:{
					CylinderParams* cylinderParams = static_cast<CylinderParams*>(salientPrim.params);
					dims[2 * shapeId + 0] = cylinderParams->radius; dims[2 * shapeId + 1] = cylinderParams->height;
					break;
				}
				case CONE:{
					ConeParams* coneParams = static_cast<ConeParams*>(salientPrim.params);
					dims[2 * shapeId + 0] = (coneParams->angle) * 180.0 / M_PI; dims[2 * shapeId + 1] = coneParams->height;
					break;
				}
				case TORUS:{
					TorusParams* torusParams = static_cast<TorusParams*>(salientPrim.params);
					dims[2 * shapeId + 0] = torusParams->minRadius; dims[2 * shapeId + 1] = torusParams->maxRadius;
					break;
				}
				default:
					break;
				}
			}
			trans[3 * shapeId + 0] = poses.at(shapeId).x; trans[3 * shapeId + 1] = poses.at(shapeId).y; trans[3 * shapeId + 2] = poses.at(shapeId).z;
			rot[4 * shapeId + 0] = poses.at(shapeId).qX; rot[4 * shapeId + 1] = poses.at(shapeId).qY; rot[4 * shapeId + 2] = poses.at(shapeId).qZ; rot[4 * shapeId + 3] = poses.at(shapeId).qW;
		}
		return 0;
	}
}

extern "C"{
	__declspec(dllexport) void restart(){
		restartDetection();
	}
}

extern "C"{
	__declspec(dllexport) void stop(){
		stopDetection();
	}
}

/*****************  Main  ********************/

#if 1

int main(){
	// Viewer
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.initCameraParameters();
	//viewer.addCoordinateSystem();

	//Hardcoded shapes for Debugging, Get them from IDE later
	std::vector<Primitive> primitives1{ Primitive(CYLINDER, new CylinderParams(0.038, 0.17)) }; // Only one primitive
	Shape shape1(primitives1);
	std::vector<Primitive> primitives2{ Primitive(CONE, new ConeParams(0.6, 0.1)) }; // Only one primitive
	Shape shape2(primitives2);
	std::vector<Primitive> primitives3{ Primitive(TORUS, new TorusParams(0.03, 0.13)) }; // Only one primitive
	Shape shape3(primitives3);
	std::vector<Primitive> primitives4{ Primitive(SPHERE, new SphereParams(0.07)) }; // Only one primitive
	Shape shape4(primitives4);
	std::vector<Primitive> primitives5{ Primitive(PLANE, new PlaneParams(0.13, 0.15)) }; // Only one primitive
	Shape shape5(primitives5);


	//std::string str("{\"Shapes\":[{\"Primitives\":[{\"PrimShape\":\"CYLINDER\",\"BaseTypeParams\":{\"radius\":0.038,\"height\":0.17}},{\"PrimShape\":\"SPHERE\",\"BaseTypeParams\":{\"radius\":0.038}}]},{\"Primitives\":[{\"PrimShape\":\"SPHERE\",\"BaseTypeParams\":{\"radius\":0.07}},{\"PrimShape\":\"CONE\",\"BaseTypeParams\":{\"angle\":0.6,\"height\":0.1}}]}]}");
	//std::string str("{\"Shapes\":[{\"objectPref\":144,\"jumpAllowed\":true,\"Primitives\":[{\"PrimShape\":\"CONE\",\"BaseTypeParams\":{\"primitivePref\":0.063,\"angle\":35.00001,\"height\":0.09999999,\"anglePref\":0.28,\"heightPref\":0.524,\"ratioPref\":0.06}},{\"PrimShape\":\"CYLINDER\",\"BaseTypeParams\":{\"primitivePref\":0.891,\"radius\":0.04000003,\"height\":0.17,\"radiusPref\":0.345,\"heightPref\":0.53,\"ratioPref\":0.173}}]}]}");
	std::string str("{\"Shapes\":[{\"objectPref\":87,\"jumpAllowed\":false,\"Primitives\":[{\"PrimShape\":\"CYLINDER\",\"BaseTypeParams\":{\"primitivePref\":1,\"radius\":0.01896293,\"height\":0.1505642,\"radiusPref\":0.304,\"heightPref\":0.529,\"ratioPref\":0.167}},{\"PrimShape\":\"SPHERE\",\"BaseTypeParams\":{\"primitivePref\":0,\"radius\":0.07527389,\"radiusPref\":1}},{\"PrimShape\":\"CONE\",\"BaseTypeParams\":{\"primitivePref\":0,\"angle\":43.89524,\"height\":0.07484054,\"anglePref\":0.284,\"heightPref\":0.41,\"ratioPref\":0.129}},{\"PrimShape\":\"TORUS\",\"BaseTypeParams\":{\"primitivePref\":0,\"minRadius\":0.05081224,\"maxRadius\":0.08900551,\"minRadiusPref\":0.108,\"maxRadiusPref\":0.892,\"ratioPref\":0.049}}]}]}");
	char ar[1024];
	strcpy(ar, str.c_str());

	int setupCode = setShapes(ar);
	//int setupCode = setShapes(std::vector<Shape>{shape1, shape4});
	if (setupCode != 0)
		return -1;
	start();

	int shapeCount = 1;
	int *types = new int[shapeCount]();
	float *dims = new float[2 * shapeCount]();
	float *trans = new float[3 * shapeCount]();
	float *rot = new float[4 * shapeCount]();
	//std::vector<Pose> poses;

	vrpn_OneEuroFilterQuat qFilter;
	qFilter.setBeta(1.0);
	qFilter.setMinCutoff(0.1);
	pcl::ScopeTime timer;

	int SP = 1;

	while (true){
		if (GetKeyState(VK_SPACE) < 0 && SP){
			std::cout << yellow << "Restarting detection.." << white << std::endl;
			restartDetection();
			SP = 0;
		}
		else if (GetKeyState(VK_F1) >= 0){
			SP = 1;
		}
		int poseCode = getPoses(shapeCount, types, dims, trans, rot);
		//poses = getPoses();

		pcl::PointCloud<pcl::PointXYZRGB> sceneFormKinect = myKinect->getCurrentPC();
		if (!sceneFormKinect.size()){
			std::cerr << "Empty cloud" << std::endl;
			continue;
		}
		pcl::PointCloud<pcl::PointXYZ> sceneFormKinectXyz; pcl::copyPointCloud(sceneFormKinect, sceneFormKinectXyz);


		viewer.removeAllPointClouds();
		for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
			// Get segmentation cloud
			pcl::PointCloud<pcl::PointXYZ> segmentedScene;
			pcl::copyPointCloud(targetSegmenters[shapeId]->getCurrentSegmentedCloud().originalPc,
				targetSegmenters[shapeId]->getCurrentSegmentedCloud().segmentedPcIndices, segmentedScene);

			// Get aliging results
			AlignResults alignResults = aligners[shapeId]->getResults();
			pcl::PointCloud<pcl::PointXYZ> alignOutput = alignResults.output;

			// Get tracker results
			TrackingResults trackResults = shapes.at(shapeId).tracker->getResults();
			pcl::PointCloud<pcl::PointXYZ> trackOutput = trackResults.output;

#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc(new pcl::PointCloud<pcl::PointXYZ>(sceneFormKinectXyz));
			viewer.addPointCloud<pcl::PointXYZ>(pc, "PointCloud" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "PointCloud" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "PointCloud" + std::to_string(shapeId));
#endif

#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr Aligner(new pcl::PointCloud<pcl::PointXYZ>(alignOutput));
			viewer.addPointCloud<pcl::PointXYZ>(Aligner, "Aligner" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Aligner" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "Aligner" + std::to_string(shapeId));
#endif
#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr Tracker(new pcl::PointCloud<pcl::PointXYZ>(trackOutput));
			viewer.addPointCloud<pcl::PointXYZ>(Tracker, "Tracker" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "Tracker" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 2.0, 1.0, 0.0, "Tracker" + std::to_string(shapeId));
#endif
#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr SegmentedScene(new pcl::PointCloud<pcl::PointXYZ>(segmentedScene));
			viewer.addPointCloud<pcl::PointXYZ>(SegmentedScene, "Segmented Scene" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Segmented Scene" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "Segmented Scene" + std::to_string(shapeId));
#endif
#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr Model(new pcl::PointCloud<pcl::PointXYZ>(shapes.at(shapeId).model));
			viewer.addPointCloud<pcl::PointXYZ>(Model, "Model" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "Model" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 2.0, 1.0, "Model" + std::to_string(shapeId));
#endif
#if 1
			//Eigen::Vector4f centroidVec;
			//pcl::compute3DCentroid(shapeModels.at(shapeId), centroidVec);

			//pcl::PointXYZ o(centroidVec.x(), centroidVec.y(), centroidVec.z());
			//pcl::PointXYZ ux(centroidVec.x() + 1.0, centroidVec.y(), centroidVec.z());
			//pcl::PointXYZ uy(centroidVec.x(), centroidVec.y() + 1.0, centroidVec.z());
			//pcl::PointCloud<pcl::PointXYZ> before(3, 1);
			//before[0] = o; before[1] = ux; before[2] = uy;

			//pcl::PointCloud<pcl::PointXYZ> after(3, 1);

			//pcl::transformPointCloud(before, after, trackResults.finalTransformation);

			//pcl::PointXYZ point1(poses.at(shapeId).x, poses.at(shapeId).y, poses.at(shapeId).z);
			pcl::PointXYZ point1(trans[3 * shapeId + 0], trans[3 * shapeId + 1], trans[3 * shapeId + 2]);
			pcl::PointCloud<pcl::PointXYZ> before(1, 1, point1);

			//double rot[4] = { poses.at(shapeId).qX, poses.at(shapeId).qY, poses.at(shapeId).qZ, poses.at(shapeId).qW };
			//const double *rotnew = qFilter.filter(timer.getTimeSeconds(), rot);

			//std::cout << "Timer " << timer.getTimeSeconds() << std::endl;
			//timer.reset();

			pcl::PointXYZ point2(trackResults.translation[0], trackResults.translation[1], trackResults.translation[2]);
			pcl::PointCloud<pcl::PointXYZ> after(1, 1, point2);

			pcl::PointCloud<pcl::PointXYZ>::ConstPtr TransPoint(before.makeShared());
			viewer.addPointCloud<pcl::PointXYZ>(TransPoint, "CoordBefore" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "CoordBefore" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "CoordBefore" + std::to_string(shapeId));

			pcl::PointCloud<pcl::PointXYZ>::ConstPtr TransPoint1(after.makeShared());
			viewer.addPointCloud<pcl::PointXYZ>(TransPoint1, "CoordAfter" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "CoordAfter" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "CoordAfter" + std::to_string(shapeId));

			//pcl::PointCloud<pcl::PointXYZ>::ConstPtr TransPoint2(trans.makeShared());
			//viewer.addPointCloud<pcl::PointXYZ>(TransPoint2, "trans" + std::to_string(shapeId));
			//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "trans" + std::to_string(shapeId));
			//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "trans" + std::to_string(shapeId));

#endif
		}
		viewer.spinOnce(1, false);

		if (poseCode == -1)
			continue;
		for (int shapeId = 0; shapeId < shapeCount; shapeId++){
			//std::cout << std::endl << "Type " << types[shapeId] << std::endl;
			//std::cout << "Dims " << dims[2 * shapeId + 0] << ", " << dims[2 * shapeId + 1] << std::endl;
			//std::cout << "Translation " << trans[3 * shapeId + 0] << ", " << trans[3 * shapeId + 1] << ", " << trans[3 * shapeId + 2] << std::endl;
			//std::cout << "Rotation   " << rot[4 * shapeId + 0] << ", " << rot[4 * shapeId + 1] << ", " << rot[4 * shapeId + 2] << ", " << rot[4 * shapeId + 3] << std::endl;
		}
		//std::cout << std::endl;

		//for each (Pose pose in poses){
			//std::cout << "Translation " << "(" << pose.x << ", " << pose.y << ", " << pose.z << ")" << std::endl;
			//std::cout << "Rotation  prev " << "(" << pose.qX << ", " << pose.qY << ", " << pose.qZ << ", " << pose.qW << ")" << std::endl;
		
			//double rot[4] = { pose.qX, pose.qY, pose.qZ, pose.qW };
			//const double *rotnew = qfilter.filter(0.03, rot);
		
			//std::cout << "Rotation  new  " << "(" << rotnew[0] << ", " << rotnew[1] << ", " << rotnew[2] << ", " << rotnew[3] << ")" << std::endl;
		//}
		//std::cout << std::endl;

	}

	stopDetection();
	for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
		delete aligners[shapeId];
		delete targetSegmenters[shapeId];
	}
	delete[] aligners;
	delete[] targetSegmenters;
	delete 	myKinect;
	delete shapeDetector;
	delete targetProcessor;
	return 0;
}


#elif 1
int main(){
	//Debug key states
	int F1 = 1, F5 = 1;

	//Hardcoded shapes for Debugging, Get them from IDE later
	std::vector<Primitive> primitives1{ Primitive(CYLINDER, new CylinderParams(0.038, 0.17)) }; // Only one primitive
	Shape shape1(primitives1);
	std::vector<Primitive> primitives2{ Primitive(CONE, new ConeParams(0.6, 0.1)) }; // Only one primitive
	Shape shape2(primitives2);
	std::vector<Primitive> primitives3{ Primitive(TORUS, new TorusParams(0.03, 0.13)) }; // Only one primitive
	Shape shape3(primitives3);
	std::vector<Primitive> primitives4{ Primitive(SPHERE, new SphereParams(0.07)) }; // Only one primitive
	Shape shape4(primitives4);
	std::vector<Primitive> primitives5{ Primitive(PLANE, new PlaneParams(0.13, 0.15)) }; // Only one primitive
	Shape shape5(primitives5);

	/* Create vectors/arrays to hold Shapes and corresponding
									Shape models, Initial aligners, Model segmenters, and Trackers*/ 
	std::vector<Shape> shapes{ shape1};
	std::vector<pcl::PointCloud<pcl::PointXYZ>> shapeModels(shapes.size(), pcl::PointCloud<pcl::PointXYZ>());
	SacPrerejectiveAligner** aligners = new SacPrerejectiveAligner*[shapes.size()];
	ModelSegmentor** targetSegmenters = new ModelSegmentor*[shapes.size()];
	IcpTracker** trackers = new IcpTracker*[shapes.size()];

	// Sensor object for data reading ** READ ONLY **
	MySensor myKinect;
	if (myKinect.initSesor() == -1){
		return -1;
	}
	// Start reading
	std::thread(&MySensor::startReading, std::ref(myKinect)).detach();

	// Shape detector
	ShapeDetector shapeDetector;
	shapeDetector.setInputCloudStream(&myKinect);
	shapeDetector.setShapes(shapes);
	std::thread (&ShapeDetector::startMatching, std::ref(shapeDetector)).detach();

	// Thread: Scene feature calculator
	FeatureCalculator targetProcessor;
	targetProcessor.setInputCloudStream(&myKinect);
	std::thread (&FeatureCalculator::startProcessing, std::ref(targetProcessor)).detach();

	// Initialize and temporarilty start all tracking threads
	for (int shapeId = 0; shapeId < shapes.size(); shapeId++){	
		// Thread: allocate Initial Aligners
		aligners[shapeId] = new SacPrerejectiveAligner();
		aligners[shapeId]->setTargetStream(&targetProcessor);
		std::thread(&SacPrerejectiveAligner::startAligning, aligners[shapeId]).detach();
		aligners[shapeId]->pause();

		// Thread: allocate Model segmenters
		targetSegmenters[shapeId] = new ModelSegmentor();
		targetSegmenters[shapeId]->setInputCloudStream(&myKinect);
		targetSegmenters[shapeId]->setInitAligner(aligners[shapeId]);
		std::thread(&ModelSegmentor::startSegmenting, targetSegmenters[shapeId]).detach();
		targetSegmenters[shapeId]->pause();

		// Thread: allocate Trackers
		trackers[shapeId] = new IcpTracker();
		trackers[shapeId]->setTargetStream(targetSegmenters[shapeId]);
		trackers[shapeId]->setInitAligner(aligners[shapeId]);
		std::thread(&IcpTracker::startTracking, trackers[shapeId]).detach();
		trackers[shapeId]->pause();
	}

	// Model feature calculator
	FeatureCalculator sourceProcessor;
	FeatureCalculator::FeatureCloud modelFormFileProcessed;

	// Model dimensions
	float modelHeight, modelWidth, modelDepth, modelRadius;
	// Model origin
	Eigen::Vector4f origin;
	// Model bounding points: to calculate dimensions
	Eigen::Vector4f min, max;

	// Wait for detected shapes
	shapeDetector.condNewResults.wait(std::unique_lock<std::mutex>(std::mutex()));

	// Fill shape models (clusters)
	fillshapeModels(shapeModels, shapeDetector.getResults());

	//For each shape, set new shape models (clusters), and resume tracking pipeline
	for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
		aligners[shapeId]->pause();
		targetSegmenters[shapeId]->pause();
		trackers[shapeId]->pause();

		if (shapeModels.at(shapeId).size() <= 0)
			continue;

		// Get origin
		pcl::compute3DCentroid(shapeModels.at(shapeId), origin);
		pcl::PointCloud<pcl::PointXYZ> originCloud(1, 1, pcl::PointXYZ(origin.x(), origin.y(), origin.z()));

		// Get dimensions
		pcl::getMinMax3D(shapeModels.at(shapeId), min, max);
		modelWidth = std::fabs(max.x() - min.x());
		modelHeight = std::fabs(max.y() - min.y());
		modelDepth = std::fabs(max.z() - min.z());
		modelRadius = std::max(modelHeight, modelWidth);

		// Process model 
		sourceProcessor.setInputCloud(shapeModels.at(shapeId));
		sourceProcessor.filterAndFeatureCalc();
		modelFormFileProcessed = sourceProcessor.getCurrentFeatureCloud();

		if (modelFormFileProcessed.localFeatures.size() <= 0)
			continue;

		// Set models in aligners
		aligners[shapeId]->setSources(modelFormFileProcessed);
		aligners[shapeId]->resume();
	
		// Set params in segmenters
		targetSegmenters[shapeId]->setModelDimensions(std::vector<float>{modelWidth, modelHeight, modelDepth, modelRadius});
		targetSegmenters[shapeId]->setModelOrigin(originCloud);
		targetSegmenters[shapeId]->resume();

		// Restart trackers ???
		trackers[shapeId]->resume();
	}


/*	//pcl::tracking implementation
	boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>> kld_
		(new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY> (8));

	std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
	default_step_covariance[3] *= 40.0;
	default_step_covariance[4] *= 40.0;
	default_step_covariance[5] *= 40.0;

	std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
	std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

	pcl::tracking::ParticleXYZRPY bin_size;
	bin_size.x = 0.1f;
	bin_size.y = 0.1f;
	bin_size.z = 0.1f;
	bin_size.roll = 0.1f;
	bin_size.pitch = 0.1f;
	bin_size.yaw = 0.1f;

	kld_->setMaximumParticleNum(1000);
	kld_->setDelta(0.99);
	kld_->setEpsilon(0.2);
	kld_->setBinSize(bin_size);

	boost::shared_ptr<pcl::tracking::ParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleXYZRPY>> tracker_ = kld_;

	tracker_->setTrans(Eigen::Affine3f::Identity());
	tracker_->setStepNoiseCovariance(default_step_covariance);
	tracker_->setInitialNoiseCovariance(initial_noise_covariance);
	tracker_->setInitialNoiseMean(default_initial_mean);
	tracker_->setIterationNum(1);
	tracker_->setParticleNum(600);
	tracker_->setResampleLikelihoodThr(0.00);
	tracker_->setUseNormal(false);

	pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr coherence
		(new pcl::tracking::ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>());

	boost::shared_ptr<pcl::tracking::DistanceCoherence<pcl::PointXYZ> > distance_coherence
		(new pcl::tracking::DistanceCoherence<pcl::PointXYZ>());
	coherence->addPointCoherence(distance_coherence);

	boost::shared_ptr<pcl::search::Octree<pcl::PointXYZ>> search(new pcl::search::Octree<pcl::PointXYZ>(0.01));
	coherence->setSearchMethod(search);
	coherence->setMaximumDistance(0.01);

	tracker_->setCloudCoherence(coherence);
	tracker_->setReferenceCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
		(new pcl::PointCloud<pcl::PointXYZ>(modelFormFileProcessed.sampledPc)));
*/

	// Start counting
	pcl::ScopeTime timer("Alignment");

	// Viewer
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor(0, 0, 0);
	viewer.initCameraParameters();
	//viewer.addCoordinateSystem();

	//Plotter
	pcl::visualization::PCLPlotter plotter("Feature Histogram");
	plotter.setWindowSize(1920, 1200);
	plotter.setShowLegend(true);

	//// Adding model descriptor to pltter
	//GlobalFeature modelDescriptor = globalFeatureCalc(modelFormFileXyz);
	//if (modelDescriptor.size() > 0)
	//	plotter.addFeatureHistogram(modelDescriptor, histSize, "model");

	/*****************  Adding saved object descriptors to plotter ********************/
#if 0
	std::ifstream is("objects\\savedObjs.txt");
	std::string pcdFileName;
	while (is.good()){
		std::getline(is, pcdFileName);
		if (pcdFileName.empty() || pcdFileName.at(0) == '#')
			continue;
		pcl::PointCloud<pcl::PointXYZ> savedObject;
		pcl::copyPointCloud(loadPcd(pcdFileName), savedObject);
		plotter.addFeatureHistogram(globalFeatureCalc(savedObject), histSize, pcdFileName.substr(8));
		plotter.spinOnce();
	}
	is.close();
#endif
	/*****************  END Adding saved objects  ********************/

	while (true){
		timer.reset();
		viewer.removeAllPointClouds();

		//Full scene for visualization only
		pcl::PointCloud<pcl::PointXYZRGB> sceneFormKinect = myKinect.getCurrentPC();
		if (!sceneFormKinect.size()){
			//std::cerr << "Empty cloud" << std::endl;
			continue;
		}
		pcl::PointCloud<pcl::PointXYZ> sceneFormKinectXyz; pcl::copyPointCloud(sceneFormKinect, sceneFormKinectXyz);

		// Get featureCalc cloud
		pcl::PointCloud<pcl::PointXYZ> featureCalcScene;
		pcl::PointCloud<pcl::PointXYZ> keypoints;
		FeatureCalculator::FeatureCloud currentFc = targetProcessor.getCurrentFeatureCloud();
		keypoints = currentFc.keyPoints;
		pcl::copyPointCloud(currentFc.originalPc, currentFc.sampledPcIndices, featureCalcScene);
		
		//// Segmented objects from kinect
		//pcl::PointCloud<pcl::PointXYZ> object;
		//std::vector<pcl::IndicesPtr> objectPointers  = objectSegmenter(sceneFormKinectXyz);
		//if (objectPointers.size() > 0)
		//	pcl::copyPointCloud(sceneFormKinectXyz, *objectPointers.at(0), object);

		//std::cout << "Objects count " << objectPointers.size() << std::endl;

		//int count = 3;
		//viewer.removeAllPointClouds();
		//for each (pcl::IndicesPtr ptr in objectPointers)
		//{
		//	pcl::PointCloud<pcl::PointXYZ> object;
		//	pcl::copyPointCloud(sceneFormKinectXyz, *ptr , object);
		//	pcl::PointCloud<pcl::PointXYZ>::ConstPtr SegmentedObject(new pcl::PointCloud<pcl::PointXYZ>(object));
		//	viewer.addPointCloud<pcl::PointXYZ>(SegmentedObject, std::to_string(count));
		//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, std::to_string(count));
		//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 3%count, 4%count, 5%count, std::to_string(count));
		//	count++;
		//}

		debugfuncs(trackers[0]->getResults().output, F1, F5);
		
		//pcl::PointCloud<pcl::PointXYZ> model;
		//pcl::copyPointCloud(object, *sacModelDetector(object), model);

		/***************** Global Features for realtime kinect input ********************/
#if 0
		GlobalFeature sceneDescriptor = globalFeatureCalc(object);
		//std::vector<double> hist(descriptor.at(0).histogram + 45, descriptor.at(0).histogram + 90);
		if (sceneDescriptor.size() > 0){
			plotter.addFeatureHistogram(sceneDescriptor, histSize);

			float dist = 0.0f;
			int pointer = 0;
			float gap;
			for (int i = 0; i < 45; i++){
				gap = std::fabs(*(sceneDescriptor.at(0).histogram + 45 + pointer) - *(modelDescriptor.at(0).histogram + 45 + pointer));
				if (gap > 0)
					dist = dist + gap;
				pointer++;
			}

			std::cout << green << "MANHATTAN DISTANCE: " << dist << white << std::endl;
		}
		plotter.setWindowSize(1920, 1200);
		plotter.setShowLegend(false);
		plotter.spinOnce();
#endif
		/***************** Global Features END ********************/


		/***************** Visualizing Normals ********************/
#if 0
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setInputCloud(object.makeShared());
		normalEstimation.setRadiusSearch(0.03f);
		normalEstimation.setSearchMethod(kdtree);
		normalEstimation.compute(*normals);
		if (object.size() > 0){
			pcl::PointCloud<pcl::PointNormal> xyzNormal;
			pcl::concatenateFields(object, *normals, xyzNormal);
			pcl::io::savePCDFileASCII("objects\\Model.pcd", object);
			pcl::io::savePCDFileASCII("objects\\normalModel.pcd", xyzNormal);
			normalsVis(object, *normals);
		}

#endif
		/***************** Normals END ********************/


/*		Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
		trans(0, 3) = 0.057192f;
		trans(1, 3) = 0.009532f;
		trans(2, 3) = -0.321338f;
		pcl::PointCloud<pcl::PointXYZ> modelFormFileXyztans;
		pcl::transformPointCloud(modelFormFileXyz, modelFormFileXyztans, trans);
		savePcd(modelFormFileXyztans);
*/
/*		//pcl::tracking implementation
		tracker_->setInputCloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
			(new pcl::PointCloud<pcl::PointXYZ>(targetProcessor.getCurrentProcessedCloud().sampledPc)));
		tracker_->compute();

		//pcl::tracking::ParticleXYZRPY result = tracker_->getResult();
		//Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);

		////move close to camera a little for better visualization
		//transformation.translation() += Eigen::Vector3f(0.0f, 0.0f, -0.005f);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		//pcl::transformPointCloud(*(tracker_->getReferenceCloud()), *result_cloud, transformation);
*/

		// Print time, fps, fitness
		//std::cout << "Main Exe Time: " << timer.getTime() << "ms" << std::endl;
		//std::cout << "Main Exe FPS: " << (1 / timer.getTimeSeconds()) << std::endl;
		//std::cout << blue << "Init Fitness: " << alignResults.fitnessScore << std::endl;
		//std::cout << yellow << "Tracker Fitness: " << trackResults.fitnessScore << white << std::endl << std::endl;

		// Translation point
		//pcl::PointCloud<pcl::PointXYZ> originCloudTemp;
		//pcl::transformPointCloud(originCloud, originCloudTemp, alignResults.finalTransformation);
		//pcl::PointXYZ center = originCloudTemp[0];

		// Visualization
#if 1
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr Scene(new pcl::PointCloud<pcl::PointXYZ>(sceneFormKinectXyz));
		viewer.addPointCloud<pcl::PointXYZ>(Scene, "Scene");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Scene");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "Scene");
#endif

		for (int shapeId = 0; shapeId < shapes.size(); shapeId++){
			// Get segmentation cloud
			pcl::PointCloud<pcl::PointXYZ> segmentedScene;
			pcl::copyPointCloud(targetSegmenters[shapeId]->getCurrentSegmentedCloud().originalPc, 
				targetSegmenters[shapeId]->getCurrentSegmentedCloud().segmentedPcIndices, segmentedScene);

			// Get aliging results
			AlignResults alignResults = aligners[shapeId]->getResults();
			pcl::PointCloud<pcl::PointXYZ> alignOutput = alignResults.output;

			// Get tracker results
			TrackingResults trackResults = trackers[shapeId]->getResults();
			pcl::PointCloud<pcl::PointXYZ> trackOutput = trackResults.output;

#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr Aligner(new pcl::PointCloud<pcl::PointXYZ>(alignOutput));
			viewer.addPointCloud<pcl::PointXYZ>(Aligner, "Aligner" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Aligner" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "Aligner" + std::to_string(shapeId));
#endif
#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr Tracker(new pcl::PointCloud<pcl::PointXYZ>(trackOutput));
			viewer.addPointCloud<pcl::PointXYZ>(Tracker, "Tracker" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "Tracker" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 2.0, 1.0, 0.0, "Tracker" + std::to_string(shapeId));
#endif
#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr SegmentedScene(new pcl::PointCloud<pcl::PointXYZ>(segmentedScene));
			viewer.addPointCloud<pcl::PointXYZ>(SegmentedScene, "Segmented Scene" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Segmented Scene" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "Segmented Scene" + std::to_string(shapeId));
#endif
#if 1
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr Model(new pcl::PointCloud<pcl::PointXYZ>(shapeModels.at(shapeId)));
			viewer.addPointCloud<pcl::PointXYZ>(Model, "Model" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "Model" + std::to_string(shapeId));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 2.0, 1.0, "Model" + std::to_string(shapeId));
#endif
		}

#if 0
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr Keypoints(new pcl::PointCloud<pcl::PointXYZ>(keypoints));
		viewer.addPointCloud<pcl::PointXYZ>(Keypoints, "Key Points");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Key Points");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "Key Points");
#endif
#if 0
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr SegmentedObject(new pcl::PointCloud<pcl::PointXYZ>(object));
		viewer.addPointCloud<pcl::PointXYZ>(SegmentedObject, "Segmented Object");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Segmented Object");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "Segmented Object");
#endif
#if 0
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr SegmentedModel(new pcl::PointCloud<pcl::PointXYZ>(model));
		viewer.addPointCloud<pcl::PointXYZ>(SegmentedModel, "Segmented Model");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Segmented Model");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "Segmented Model");
#endif

#if 0
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr TransPoint(new pcl::PointCloud<pcl::PointXYZ>(1, 1, center));
		viewer.addPointCloud<pcl::PointXYZ>(TransPoint, "Trans Point");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "Trans Point");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "Trans Point");
#endif
		viewer.spinOnce(1, false);
	}

	return 0;
}
#endif

#if 0
class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped())
         viewer.showCloud (cloud);
     }

     void run ()
     {
		 std::string device_id("#1");
		 pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
		 pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;


		//pcl::Grabber* grabber = new pcl::io::OpenNI2Grabber(device_id, depth_mode, image_mode);

      // boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
      //   boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

	   //grabber->registerCallback(f);

	   //grabber->start();

       //while (!viewer.wasStopped())
       //{
       //  boost::this_thread::sleep (boost::posix_time::seconds (1));
       //}

	   //grabber->stop();
     }

     pcl::visualization::CloudViewer viewer;
 };

int main()
{
	//SimpleOpenNIViewer v;
	//v.run();

	boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance ();
    if (deviceManager->getNumOfConnectedDevices () > 0)
    {
      boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getAnyDevice ();
      cout << "Device ID not set, using default device: " << device->getStringID () << endl;
    }

	return 0;
}

#endif