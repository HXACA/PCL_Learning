#include <iostream>
#include <string>
#include "tinyxml2.h"

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
int times = 0;
double cal[6];
bool next_time = false;
Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
pcl::console::TicToc gettime;

void readDat(cv::Mat &depth, std::string name) {
	std::ifstream file(name+".dat", std::ios::in | std::ios::binary);
	file.seekg(0, file.end);
	int length = file.tellg();
	int range_length = length / 5 * 4;
	int intensity_length = length / 5;
	int width = cal[5];
	int height = intensity_length / width;
	depth.create(height, width, CV_32F);
	file.seekg(0, file.beg);
	file.read((char*)depth.data, range_length);
}

void readXml(std::string name) {
	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(const_cast<char *>((name+".xml").c_str())) != 0) {
		std::cout << "load failed!" << std::endl;
		return;
	}
	tinyxml2::XMLElement* root = doc.RootElement();
	tinyxml2::XMLElement* componemtNode = root->FirstChildElement("component");
	tinyxml2::XMLElement* worldrangetraitsNode = componemtNode->FirstChildElement("worldrangetraits");
	tinyxml2::XMLElement* parameterNode = worldrangetraitsNode->FirstChildElement("parameter");
	for (int i = 0; i < 4; i++) {
		cal[i] = std::stof(parameterNode->GetText());
		parameterNode = parameterNode->NextSiblingElement();
	}
	tinyxml2::XMLElement* subcomponentNode = componemtNode->FirstChildElement("subcomponent");
	parameterNode = subcomponentNode->FirstChildElement("parameter");
	for (int i = 4; i < 6; i++) {
		cal[i] = std::stof(parameterNode->GetText());
		parameterNode = parameterNode->NextSiblingElement();
	}
}

PointCloud::Ptr savePointCloud(const cv::Mat& depth, double yRes) {
	double xRes = (cal[2] - cal[0]) / cal[5];
	PointCloud::Ptr cloud(new PointCloud);
	for (int m = 0; m < 1000; m++) {
		for (int n = 0; n < depth.cols; n++){
				float d = depth.ptr<float>(m)[n];
				if (isnan(d) || d == -1e+06)
					continue;
				PointT p;
				p.z = double(d);
				p.x = n*xRes/2;//+cal[0];
				p.y = m*yRes/2;

				p.b = 255;
				p.g = 255;
				p.r = 255;
				if(n%2==0 && m%2==0)
					cloud->points.push_back(p);
		}
	}
		

	cloud->height = 1;
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
	cloud->width = cloud->points.size();
	std::cout << "point cloud size = " << cloud->points.size() << std::endl;
	cloud->is_dense = true;
	pcl::io::savePCDFile("1.pcd", *cloud);
	//cloud->points.clear();
	std::cout << "Point cloud saved." << std::endl;
	return cloud;
}

int PCDtoPLYconvertor(std::string  input_filename, std::string output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(input_filename, cloud) < 0)
	{
		std::cout << "Error: cannot load the PCD file!!!" << std::endl;
		return -1;
	}
	pcl::PLYWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
	return 0;

}

void printMatrix(const Eigen::Matrix4d &matrix) {
	printf("旋转矩阵R:\n");
	printf("      |%6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf(" R =  |%6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("      |%6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("平移矩阵T: \n");
	printf("T = < %6.3f %6.3f %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void init(double theta) {
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);
	transformation_matrix(0, 3) = 0;
	transformation_matrix(1, 3) = 0;
	transformation_matrix(2, 3) = 1;
	printMatrix(transformation_matrix);
	times= 1;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_time = true;
}

int visualICP(PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_tr, PointCloud::Ptr cloud_icp, pcl::IterativeClosestPoint<PointT, PointT> icp) {
	pcl::visualization::PCLVisualizer viewer("ICP Demo");
	int v1(0);
	int v2(1);
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);
	double bac_gray_level = 0.0;
	double txt_gray_level = 1.0 - bac_gray_level;

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, 255, 255, 255);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 0, 255, 0);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 255, 0, 0);
	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	std::stringstream ss;
	ss << times;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, 1, 1, 1, "iterations_cnt");
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.setBackgroundColor(0, 0, 0, v2);

	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
		if (next_time) {
			gettime.tic();
			icp.align(*cloud_icp);
			std::cout << "Applied" << 1 << "ICP iteration in " << gettime.toc() << "ms" << std::endl;
			if (icp.hasConverged()) {
				std::cout << "\n ICP has converged, score is " << icp.getFitnessScore() << std::endl;
				std::cout << "\n ICP transformation " << times++ << ": cloud_icp -> cloud_in" << std::endl;
				transformation_matrix = icp.getFinalTransformation().cast<double>();;
				printMatrix(transformation_matrix);
				ss.str("");
				ss << times;
				iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, 1, 1, 1, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
			}
			else {
				PCL_ERROR(" \n ICP has not converged.\n");
				return -1;
			}
		}
		next_time = true;
		if (times<2 || icp.getFitnessScore()<0.01 || times>100) {
			next_time = false;
		}
	}
	return 0;
}


int main(int argc, char *argv[])
{
	//读入点云
	PointCloud::Ptr cloud_in(new PointCloud);
	if (pcl::io::loadPCDFile("data//1.pcd", *cloud_in) < 0) {
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return -1;
	}
	PointCloud::Ptr cloud_tr(new PointCloud);//中间结果
	PointCloud::Ptr cloud_icp(new PointCloud);//icp结果

	//初始化数据
	
	double theta = M_PI / 8; //角度
	init(theta);
	pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
	*cloud_tr = *cloud_icp;
	//ICP
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.setMaximumIterations(1);
 	visualICP(cloud_in, cloud_tr, cloud_icp, icp);
	system("Pause");
	return 0;
}