#include "pch.h"
#include "FileManager.h"
#include "utils.cpp"
#include<fstream>
#include<iostream>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

Eigen::Matrix3d createIntrinsic(double fx, double fy, double cx, double cy)
{
	Eigen::Matrix3d K = Eigen::Matrix3d::Identity();

	K(0, 0) = fx;
	K(1, 1) = fy;
	K(0, 2) = cx;
	K(1, 2) = cy;

	return K;
}
TMErrCode FileManager::readParamater(std::string parameterPath, Parameter& parameter)
{
	cv::FileStorage fileStorage;

	bool ret = fileStorage.open(parameterPath, cv::FileStorage::READ);
	if(!ret)
	{
		return TM_ERR_FILE_OPEN;
	}

	int tempType;
	fileStorage["inputType"] >> tempType;
	parameter.inputType = static_cast<InputType>(tempType);

	fileStorage["meshPath"] >> parameter.meshPath;
	fileStorage["imagesFolder"] >> parameter.imagesFolder;
	fileStorage["psvSensorCameraParamPath"] >> parameter.psvSensorCameraParamPath;
	fileStorage["psvSensorImagesParamPath"] >> parameter.psvSensorImagesParamPath;	
	fileStorage["psvSensorPoints3DParamPath"] >> parameter.psvSensorPoints3DParamPath;
	fileStorage["psvSensorVisParamPath"] >> parameter.psvSensorVisParamPath;
	fileStorage["atvSensorParamPath"] >> parameter.atvSensorParamPath;

	return TM_OK;
}
TMErrCode FileManager::loadImages(std::string imageFolder, std::vector<ImgInfo>& imgInfos)
{

	if (imgInfos.size() == 0)  //  �̹��� Index�� ���������� ����
	{
		std::vector<cv::String> fn;
		cv::glob(imageFolder + "/*", fn, false);
		imgInfos.resize(fn.size());
		for (int i = 0; i < fn.size(); i++)
		{
			imgInfos[i].image = cv::imread(fn[i]);
		}
	}
	else  //  �̹��� Index�� ����������, Active �����غ�����
	{
		for (int i = 0; i < imgInfos.size(); i++)
		{
			std::string imageName = imgInfos[i].imageName;
			imgInfos[i].image = cv::imread(imageFolder + "/" + imageName);
		}
	}

	std::cout << "Loaded Images Successfully ! \n";

	return TM_OK;
}
TMErrCode FileManager::loadMesh(std::string meshPath, TexturedMesh& texturedMesh)
{
	std::ifstream in(meshPath);

	if (!in.is_open())
	{
		std::cout << "Mesh Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
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

	return TM_OK;
}
TMErrCode FileManager::readPassiveSensorInfo(const std::vector<std::string>& passiveInfoPath, std::vector<ImgInfo>& imgInfos)
{
	std::string cameraPath = passiveInfoPath[0];
	std::string imagePath = passiveInfoPath[1];

	// Read cameras.bin
	std::ifstream in(cameraPath, std::ios::binary);
	if (!in.is_open())
	{
		std::cout << "cameras.bin Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
	}

	const size_t num_cameras = readBinaryLittleEndian<uint64_t>(&in);

	std::vector<Eigen::Matrix3d> kVec;
	kVec.resize(num_cameras);

	for (size_t i = 0; i < num_cameras; ++i) {
		uint32_t cameraID = readBinaryLittleEndian<uint32_t>(&in); // Camera ID
		int cameraModel = readBinaryLittleEndian<int>(&in);
		uint64_t width = readBinaryLittleEndian<uint64_t>(&in);
		uint64_t height = readBinaryLittleEndian<uint64_t>(&in);

		std::vector<double> params;
		Eigen::Matrix3d K;
		switch (cameraModel)
		{
		case 1: // PINHOLE(fx, fy, cx, cy)
			params.resize(4);
			readBinaryLittleEndian<double>(&in, &params);
			K = createIntrinsic(params[0], params[1], params[2], params[3]);
			break;
		case 2: // SIMPLE_RAIDAL(f, cx, cy, k)
			params.resize(4);
			readBinaryLittleEndian<double>(&in, &params);
			K = createIntrinsic(params[0], params[0], params[1], params[2]);
			break;
		default:
			std::cout << "No Camera Model Exist ! \n";
			break;
		}
		kVec.at(cameraID) = K;
	}

	in.close();

	// Read images.bin
	std::ifstream inImage(imagePath, std::ios::binary);
	if (!inImage.is_open())
	{
		std::cout << "images.bin Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
	}

	const size_t num_reg_images = readBinaryLittleEndian<uint64_t>(&inImage);

	if (imgInfos.size() == 0)
		imgInfos.resize(num_reg_images);

	if (imgInfos.size() != num_reg_images)
	{
		std::cout << "Nums of Image and Reg_Image are Not Same ! \n";
		return TM_ERR;
	}

	for (size_t i = 0; i < num_reg_images; ++i) {

		uint32_t imageID = readBinaryLittleEndian<uint32_t>(&inImage);
		Eigen::Vector4d qVec;
		qVec(0) = readBinaryLittleEndian<double>(&inImage);
		qVec(1) = readBinaryLittleEndian<double>(&inImage);
		qVec(2) = readBinaryLittleEndian<double>(&inImage);
		qVec(3) = readBinaryLittleEndian<double>(&inImage);
		qVec = normalizeQuaternion(qVec);

		Eigen::Vector3d tVec;
		tVec(0) = readBinaryLittleEndian<double>(&inImage);
		tVec(1) = readBinaryLittleEndian<double>(&inImage);
		tVec(2) = readBinaryLittleEndian<double>(&inImage);

		uint32_t cameraID = readBinaryLittleEndian<uint32_t>(&inImage);

		char name_char;
		std::string imageName;
		do {
			inImage.read(&name_char, 1);
			if (name_char != '\0') {
				imageName += name_char;
			}
		} while (name_char != '\0');

		imgInfos.at(imageID).imageID = imageID;
		imgInfos.at(imageID).imageName = imageName;
		imgInfos.at(imageID).T = composeProjectionMatrix(qVec, tVec);
		imgInfos.at(imageID).cameraID = cameraID;
		imgInfos.at(imageID).K = kVec.at(cameraID);

		const size_t num_points2D = readBinaryLittleEndian<uint64_t>(&inImage);

		std::vector<Eigen::Vector2d> points2D;
		points2D.reserve(num_points2D);
		std::vector<uint64_t> point3D_ids;
		point3D_ids.reserve(num_points2D);
		for (size_t j = 0; j < num_points2D; ++j) {
			const double x = readBinaryLittleEndian<double>(&inImage);
			const double y = readBinaryLittleEndian<double>(&inImage);
			points2D.emplace_back(x, y);
			point3D_ids.push_back(readBinaryLittleEndian<uint64_t>(&inImage));
		}

		//image.SetUp(Camera(image.CameraId()));
		//image.SetPoints2D(points2D);
		//for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
		//	++point2D_idx) {
		//	if (point3D_ids[point2D_idx] != kInvalidPoint3DId) {
		//		image.SetPoint3DForPoint2D(point2D_idx, point3D_ids[point2D_idx]);
		//	}
		//}
		//image.SetRegistered(true);
		//reg_image_ids_.push_back(image.ImageId());

		//images_.emplace(image.ImageId(), image);
	}

	return TM_OK;
}
TMErrCode FileManager::saveTexturedMesh(std::string outPath, const TexturedMesh& texturedMesh)
{
	std::ofstream out(outPath);

	if (!out.is_open())
	{
		std::cout << "TexturedMesh Output Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
	}

	// TexturedMesh ����ü�� �ִ� �����͸� output Path�� �������ִ� �ڵ� �ۼ�

	std::cout << "Saved TexturedMesh Successfully ! \n";

	out.close();

	return TM_OK;
}
TMErrCode FileManager::loadTexturedMesh(std::string inputPath, TexturedMesh& texturedMesh)
{
	std::ifstream in(inputPath);

	if (!in.is_open())
	{
		std::cout << "TexturedMesh Input Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
	}

	// TexturedMesh Path�� TexturedMesh�� �о� TexturedMesh ����ü�� �־��ִ� �ڵ� �ۼ�

	std::cout << "Loaded TexturedMesh Successfully ! \n";

	in.close();

	return TM_OK;
}
TMErrCode FileManager::saveTexturedMeshwithMaterial(std::string outPath, const TexturedMesh& texturedMesh, const Material& material)
{
	std::ofstream out(outPath);

	if (!out.is_open())
	{
		std::cout << "TexturedMesh with Material Output Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
	}

	// TexturedMesh�� Material ����ü�� �ִ� �����͸� output Path�� �������ִ� �ڵ� �ۼ�

	std::cout << "Saved TexturedMesh with Material Successfully ! \n";

	out.close();

	return TM_OK;
}
TMErrCode FileManager::loadTexturedMeshwithMaterial(std::string inputPath, TexturedMesh& texturedMesh, Material& material)
{
	std::ifstream in(inputPath);

	if (!in.is_open())
	{
		std::cout << "TexturedMesh with Material Input Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
	}

	// TexturedMeshwithMaterial Path�� TexturedMeshwithMaterial�� �о� TexturedMesh�� Material ����ü�� �־��ִ� �ڵ� �ۼ�

	std::cout << "Loaded TexturedMeshwithMaterial Successfully ! \n";

	in.close();

	return TM_OK;
}