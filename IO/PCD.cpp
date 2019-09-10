#include<iostream>
#include<pcl\io\pcd_io.h>
#include<pcl\point_types.h>
#include<string>

int read(std::string name) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(name, *cloud) < 0) {
		PCL_ERROR("qusiba");
		return -1;
	}
	for (size_t i = 0; i < cloud->points.size(); i++) 
		std::cout << "   " << cloud->points[i].x << " " << cloud->points[i].y <<" "<< cloud->points[i].z << std::endl;
	return 0;
}

int write(std::string name,int width) {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = width;
	cloud.height = 1;
	cloud.is_dense = false; //包含nan
	cloud.points.resize(cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size(); i++) {
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	std::cout<<pcl::io::savePCDFile(name, cloud,false);
	for (size_t i = 0; i < cloud.points.size(); i++)
		std::cout << "   " << cloud.points[i].x << " " << cloud.points[i].y <<" "<< cloud.points[i].z << std::endl;
	std::cout << std::endl;
	return 0;
}

int write2(std::string name, int width) {
	pcl::PointCloud<pcl::PointNormal> cloud;
	cloud.width = width;
	cloud.height = 1;
	cloud.is_dense = false; //包含nan
	cloud.points.resize(cloud.width * cloud.height);
	for (size_t i = 0; i < cloud.points.size(); i++) {
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	std::cout << pcl::io::savePCDFile(name, cloud, false);
	for (size_t i = 0; i < cloud.points.size(); i++)
		std::cout << "   " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
	std::cout << std::endl;
	return 0;
}


int concatenate_clouds(std::string name_a, std::string name_b,std::string name_c) {
	pcl::PointCloud<pcl::PointXYZ>cloud_a, cloud_b, cloud_c;
	//pcl::PointCloud<pcl::Normal>n_cloud_b;//normal点云
	pcl::PointCloud<pcl::PointNormal>n_cloud_b,p_n_cloud_c;//连接xyz与normal后的点云
	if (pcl::io::loadPCDFile(name_a, cloud_a) < 0) {
		PCL_ERROR("test1错误");
		return -1;
	}
	if (pcl::io::loadPCDFile(name_b, n_cloud_b) < 0) {
		PCL_ERROR("test2错误");
		return -1;
	}
	pcl::concatenateFields(cloud_a, n_cloud_b, p_n_cloud_c);
	std::cout<<pcl::io::savePCDFile(name_c, p_n_cloud_c, false);

}

int main() {
	write("data//test1.pcd",5);
	write2("data//test2.pcd",5);
	concatenate_clouds("data//test1.pcd", "data//test2.pcd", "data//test3.pcd");
	read("data//test3.pcd");
	system("PAUSE");
	return 0;
}