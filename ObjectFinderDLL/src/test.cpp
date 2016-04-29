#include "stdafx.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

//inline void normalsVis(pcl::PointCloud<pcl::PointXYZ> pcXyz, pcl::PointCloud<pcl::Normal> pcNormal){
//	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZ>(pcXyz));
//	pcl::PointCloud<pcl::Normal>::ConstPtr normals(new pcl::PointCloud<pcl::Normal>(pcNormal));
//	pcl::visualization::PCLVisualizer viewer("3D Viewer");
//	viewer.setBackgroundColor(0, 0, 0);
//	viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
//	viewer.initCameraParameters();
//	viewer.spin();
//	viewer.close();
//}
//
//int
//main(int argc, char** argv)
//{
//	// Load input file into a PointCloud<T> with an appropriate type
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	// Load bun0.pcd -- should be available with the PCL archive in test 
//	pcl::io::loadPCDFile("objects\\beer_bottle.pcd", *cloud);
//
//	// Create a KD-Tree
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//
//	// Output has the PointNormal type in order to store the normals calculated by MLS
//	pcl::PointCloud<pcl::PointXYZ> mls_points;
//
//	// Init object (second point type is for the normals, even if unused)
//	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
//
//	mls.setComputeNormals(false);
//
//	pcl::ScopeTime timer("Timer");
//
//	// Set parameters
//	mls.setInputCloud(cloud);
//	mls.setPolynomialFit(true);
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(0.04);
//
//	// Reconstruct
//	mls.process(mls_points);
//
//
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
//	normalEstimation.setInputCloud(mls_points.makeShared());
//	normalEstimation.setRadiusSearch(0.02f);
//	normalEstimation.setSearchMethod(kdtree);
//	normalEstimation.compute(*normals);
//
//	normalsVis(mls_points, *normals);
//
//	// Save output
//	pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
//}