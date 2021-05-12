#include "pch.h"
#include "FileManager.h"
//#include "TextureMapper.h"
//s#include "MaterialClassifier.h"
#include<fstream>
#include<iostream>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

int FileManager::readParameter(std::string parameterPath, Parameter& parameter)
{
	std::ifstream in(parameterPath);

	if (!in.is_open())
	{
		std::cout << "Parameter Path Wrong ! \n";
		return 0;
	}

	// �Ķ���͸� �о parameter �� �־��ִ� �ڵ� �ۼ�

	std::cout << "Read Parameter Successfully ! \n";

	in.close();

	return 1;
}
int FileManager::loadImages(std::string imageFolder, std::vector<ImgInfo>& parameter)
{
	std::ifstream in(imageFolder);

	if (!in.is_open())
	{
		std::cout << "Image Folder Path Wrong ! \n";
		return 0;
	}

	// �̹��� ���� ���� �� �о vecter�� �־��ִ� �ڵ� �ۼ�
	
	std::cout << "Loaded Images Successfully ! \n";

	in.close();

	return 1;
}
int FileManager::loadMesh(std::string meshPath, TexturedMesh& texturedMesh)
{
	std::ifstream in(meshPath);

	if (!in.is_open())
	{
		std::cout << "Mesh Path Wrong ! \n";
		return 0;
	}
	in.close();

	// Mesh Path�� Mesh�� �о� TexturedMesh ����ü�� �־��ִ� �ڵ� �ۼ�
	pcl::PolygonMesh polygonMesh;
	pcl::PLYReader reader;

	reader.read(meshPath, polygonMesh);
	pcl::PointCloud<pcl::PointXYZRGBNormal> points;

	pcl::fromPCLPointCloud2(polygonMesh.cloud, points);
	
	Point pTmp;
	for (int i = 0; i < points.size(); i++)
	{
		pTmp.point = Point3D<float>(points[i].x, points[i].y, points[i].z);
		pTmp.normal = Point3D<float>(points[i].normal_x, points[i].normal_y, points[i].normal_z);
		pTmp.index = i;
		texturedMesh.points.push_back(pTmp);
	}

	Face fTmp;
	for (int i = 0; i < polygonMesh.polygons.size(); i++)
	{
		fTmp.vertexIndex[0] = polygonMesh.polygons[i].vertices[0];
		fTmp.vertexIndex[1] = polygonMesh.polygons[i].vertices[1];
		fTmp.vertexIndex[2] = polygonMesh.polygons[i].vertices[2];
		texturedMesh.faces.push_back(fTmp);
	}

	std::cout << "Loaded Mesh Successfully ! \n";

	return 1;
}
int FileManager::saveTexturedMesh(std::string outPath, const TexturedMesh& texturedMesh)
{
	std::ofstream out(outPath);

	if (!out.is_open())
	{
		std::cout << "TexturedMesh Output Path Wrong ! \n";
		return 0;
	}

	// TexturedMesh ����ü�� �ִ� �����͸� output Path�� �������ִ� �ڵ� �ۼ�

	std::cout << "Saved TexturedMesh Successfully ! \n";

	out.close();

	return 1;
}
int FileManager::loadTexturedMesh(std::string inputPath, TexturedMesh& texturedMesh)
{
	std::ifstream in(inputPath);

	if (!in.is_open())
	{
		std::cout << "TexturedMesh Input Path Wrong ! \n";
		return 0;
	}

	// TexturedMesh Path�� TexturedMesh�� �о� TexturedMesh ����ü�� �־��ִ� �ڵ� �ۼ�

	std::cout << "Loaded TexturedMesh Successfully ! \n";

	in.close();

	return 1;
}
int FileManager::saveTexturedMeshwithMaterial(std::string outPath, const TexturedMesh& texturedMesh, const Material& material)
{
	std::ofstream out(outPath);

	if (!out.is_open())
	{
		std::cout << "TexturedMesh with Material Output Path Wrong ! \n";
		return 0;
	}

	// TexturedMesh�� Material ����ü�� �ִ� �����͸� output Path�� �������ִ� �ڵ� �ۼ�

	std::cout << "Saved TexturedMesh with Material Successfully ! \n";

	out.close();

	return 1;
}
int FileManager::loadTexturedMeshwithMaterial(std::string inputPath, TexturedMesh& texturedMesh, Material& material)
{
	std::ifstream in(inputPath);

	if (!in.is_open())
	{
		std::cout << "TexturedMesh with Material Input Path Wrong ! \n";
		return 0;
	}

	// TexturedMeshwithMaterial Path�� TexturedMeshwithMaterial�� �о� TexturedMesh�� Material ����ü�� �־��ִ� �ڵ� �ۼ�

	std::cout << "Loaded TexturedMeshwithMaterial Successfully ! \n";

	in.close();

	return 1;
}