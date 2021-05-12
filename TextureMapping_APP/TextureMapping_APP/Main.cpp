//#include "Manager.h"
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

int main()
{
	//TextureMaterialManager manager;

	//int err = manager.initialize("abc");

	pcl::PolygonMesh mesh;
	pcl::PLYReader reader;

	//reader.read("E:/data/blue_glove0415/dense/meshed-poisson.ply", mesh);

	reader.read("E:/data/blue_glove0415/dense/meshed-poisson.ply", mesh);
	
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	
	pcl::PointCloud<pcl::PointXYZRGBNormal> vert;
	pcl::fromPCLPointCloud2(mesh.cloud, vert);
	
	//for (int i = 0; i < vert->size(); i++)
	//{
	//	std::cout << vert->points[i].x << " " << vert->points[i].y << " " << vert->points[i].z << '\n';
	//}

	//pcl::PointCloud<pcl::PointXYZRGBNormal> vertex;

	//reader.read("E:/data/blue_glove0415/dense/meshed-poisson2.ply", vertex);


	return 0;
}