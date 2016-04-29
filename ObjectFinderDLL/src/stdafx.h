#pragma once

#define NOMINMAX

//#define VERBOSE

//#define CHECK_READING_TIME
//#define CHECK_MATCHING_TIME
//#define CHECK_SEGMENT_TIME
//#define CHECK_FEATURECALC_TIME
//#define CHECK_ALIGNMENT_TIME
//#define CHECK_TRACKING_TIME

#include <Windows.h>
#include <kinect.h>
#include <DepthSense.hxx>

#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>

// Boost headers
#include <boost\make_shared.hpp>

// PCL headers
#include <pcl\common\time.h>

#include <pcl\surface\on_nurbs\fitting_curve_2d_asdm.h>
#include <pcl\surface\on_nurbs\fitting_surface_tdm.h>
#include <pcl\surface\on_nurbs\triangulation.h>
#include <pcl\surface\mls.h>

#include <pcl\io\pcd_io.h>
#include <pcl\point_types.h>
#include <pcl\search\kdtree.h>
#include <pcl\search\organized.h>


#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\sample_consensus\method_types.h>
#include <pcl\sample_consensus\model_types.h>
#include <pcl\ModelCoefficients.h>
#include <pcl\surface\convex_hull.h>
#include <pcl\segmentation\extract_polygonal_prism_data.h>
#include <pcl\segmentation\extract_clusters.h>


#include <pcl\filters\passthrough.h>
#include <pcl\filters\statistical_outlier_removal.h>
#include <pcl\filters\radius_outlier_removal.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\filters\extract_indices.h>
#include <pcl\filters\fast_bilateral.h>

#include <pcl\keypoints\uniform_sampling.h>
//#include <pcl\keypoints\iss_3d.h>

//#include <pcl\features\normal_3d_omp.h>
#include <pcl\features\normal_3d.h>
#include <pcl\features\integral_image_normal.h>
#include <pcl\features\pfh.h>
//#include <pcl\features\fpfh_omp.h>
#include <pcl\features\fpfh.h>
#include <pcl\features\ppf.h>
#include <pcl\features\3dsc.h>
#include <pcl\features\usc.h>
#include <pcl\features\shot_omp.h>
#include <pcl\features\spin_image.h>
#include <pcl\features\vfh.h>
#include <pcl\features\our_cvfh.h>
#include <pcl\features\esf.h>

#include <pcl\registration\ia_ransac.h>
#include <pcl\registration\sample_consensus_prerejective.h>
#include <pcl\registration\icp.h>

#include <pcl\visualization\cloud_viewer.h>
#include <pcl\visualization\pcl_plotter.h>

// RANSAC headers
#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include <SpherePrimitiveShape.h>
#include <CylinderPrimitiveShape.h>
#include <ConePrimitiveShape.h>
#include <TorusPrimitiveShape.h>

// dlib Hungarian optimization headers  
#include <dlib/optimization/max_cost_assignment.h>

// JsonCpp headers
#include <json\json.h>

// VRPN 1 Euro filter
#include "vrpn_OneEuroFilter.h" 


inline std::ostream& red(std::ostream &s)
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hStdout,
		FOREGROUND_RED | FOREGROUND_INTENSITY);
	return s;
}

inline std::ostream& green(std::ostream &s)
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hStdout,
		FOREGROUND_GREEN | FOREGROUND_INTENSITY);
	return s;
}

inline std::ostream& blue(std::ostream &s)
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hStdout, FOREGROUND_BLUE
		| FOREGROUND_GREEN | FOREGROUND_INTENSITY);
	return s;
}

inline std::ostream& yellow(std::ostream &s)
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hStdout,
		FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_INTENSITY);
	return s;
}

inline std::ostream& white(std::ostream &s)
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hStdout,
		FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
	return s;
}

struct color {
	color(WORD attribute) :m_color(attribute){};
	WORD m_color;
};

template <class _Elem, class _Traits>
std::basic_ostream<_Elem, _Traits>&
operator<<(std::basic_ostream<_Elem, _Traits>& i, color& c)
{
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	SetConsoleTextAttribute(hStdout, c.m_color);
	return i;
}