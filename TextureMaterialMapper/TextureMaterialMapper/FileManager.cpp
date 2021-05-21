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

	if (imgInfos.size() == 0)  //  이미지 Index가 정해져있지 않음
	{
		std::vector<cv::String> fn;
		cv::glob(imageFolder + "/*", fn, false);
		imgInfos.resize(fn.size());
		for (int i = 0; i < fn.size(); i++)
		{
			imgInfos[i].image = cv::imread(fn[i]);
		}
	}
	else  //  이미지 Index가 정해져있음, Active 생각해봐야함
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

	// Mesh Path의 Mesh를 읽어 TexturedMesh 구조체에 넣어주는 코드 작성
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

	std::cout << "Loaded Mesh Successfully ! \n\n";

	return TM_OK;
}
TMErrCode FileManager::readPassiveSensorInfoBinary(const std::vector<std::string>& passiveInfoPath, std::vector<ImgInfo>& imgInfos)
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
			return TM_ERR;
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

	}

	std::cout << "Loaded Passive Sensor Info Successfully ! \n\n";
	return TM_OK;
}
TMErrCode readPassiveSensorInfoText(const std::vector<std::string>& passiveInfoPath, std::vector<ImgInfo>& imgInfos)
{
	std::string cameraPath = passiveInfoPath[0];
	std::string imagePath = passiveInfoPath[1];

	std::ifstream file(cameraPath);
	
	if (!file.is_open())
	{
		std::cout << "cameras.txt Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
	}

	std::string line;
	std::string item;

	std::vector<Eigen::Matrix3d> kTmp;
	std::vector<uint32_t> cameraIDVec;
	while (std::getline(file, line)) 
	{
		StringTrim(&line);

		if (line.empty() || line[0] == '#') 
			continue;
		
		std::stringstream line_stream(line);

		// ID
		std::getline(line_stream, item, ' ');
		cameraIDVec.push_back(std::stoul(item));

		// MODEL
		std::getline(line_stream, item, ' ');
		std::string modelID = item;

		// WIDTH
		std::getline(line_stream, item, ' ');
		uint64_t width = std::stoll(item);

		// HEIGHT
		std::getline(line_stream, item, ' ');
		uint64_t height = std::stoll(item);

		// PARAMS
		std::vector<double> params;
		while (!line_stream.eof()) 
		{
			std::getline(line_stream, item, ' ');
			params.push_back(std::stold(item));
		}
		
		if (modelID == "PINHOLE")
			kTmp.push_back(createIntrinsic(params[0], params[1], params[2], params[3]));
		else if (modelID == "SIMPLE_RADIAL")
			kTmp.push_back(createIntrinsic(params[0], params[0], params[2], params[3]));
		else
		{
			std::cout << "No Camera Model Exist ! \n";
			return TM_ERR;
		}
	}

	// K를 CAMERA ID 에 대해 정렬
	std::vector<Eigen::Matrix3d> kVec;
	kVec.resize(cameraIDVec.size());
	for (int i = 0; i < cameraIDVec.size(); i++)
		kVec[cameraIDVec[i]] = kTmp[i];

	file.close();

	// Read images.txt
	file.open(imagePath);
	if (!file.is_open())
	{
		std::cout << "images.txt Path Wrong ! \n";
		return TM_ERR_FILE_OPEN;
	}

	bool noImageInStruct;

	if (imgInfos.size() == 0)
		noImageInStruct = true;
	else
		noImageInStruct = false;

	while (std::getline(file, line)) 
	{
		StringTrim(&line);
		if (line.empty() || line[0] == '#') 
			continue;

		std::stringstream line_stream1(line);

		// ID
		std::getline(line_stream1, item, ' ');
		const uint32_t imageID = std::stoul(item);

		// QVEC (qw, qx, qy, qz)
		Eigen::Vector4d qVec;
		std::getline(line_stream1, item, ' ');
		qVec(0) = std::stold(item);

		std::getline(line_stream1, item, ' ');
		qVec(1) = std::stold(item);

		std::getline(line_stream1, item, ' ');
		qVec(2) = std::stold(item);

		std::getline(line_stream1, item, ' ');
		qVec(3) = std::stold(item);

		qVec = normalizeQuaternion(qVec);

		// TVEC
		Eigen::Vector3d tVec;
		std::getline(line_stream1, item, ' ');
		tVec(0) = std::stold(item);

		std::getline(line_stream1, item, ' ');
		tVec(1) = std::stold(item);

		std::getline(line_stream1, item, ' ');
		tVec(2) = std::stold(item);

		// CAMERA_ID
		std::getline(line_stream1, item, ' ');
		uint32_t cameraID = std::stoul(item);

		// NAME
		std::getline(line_stream1, item, ' ');
		std::string imageName = item;

		ImgInfo infoTmp;
		infoTmp.cameraID = cameraID;
		infoTmp.imageID = imageID;
		infoTmp.imageName = imageName;
		infoTmp.T = composeProjectionMatrix(qVec, tVec);
		infoTmp.K = kVec[cameraID];

		// * * 예외처리하긴 했지만 PassiveSensor 있을때는 Image 말고 PassiveSensorInfo부터 Load 해야합니다. * *
		if (noImageInStruct)
		{
			imgInfos.push_back(infoTmp);
		}
		else
		{
			std::cout << "Image Load 전에 Passive Sensor Info 부터 Load 해야함 ! !\n";
			imgInfos[imageID] = infoTmp; 
		}

		// POINTS2D
		if (!std::getline(file, line)) 
			break;

		StringTrim(&line);
		std::stringstream line_stream2(line);

		if (!line.empty()) 
		{
			while (!line_stream2.eof()) 
			{
				std::getline(line_stream2, item, ' '); // x
				std::getline(line_stream2, item, ' '); // y
				std::getline(line_stream2, item, ' '); // 3D index
			}
		}
	}

	file.close();

	std::cout << "Loaded Passive Sensor Info Successfully ! \n\n";

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

	// TexturedMesh 구조체에 있는 데이터를 output Path에 저장해주는 코드 작성

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

	// TexturedMesh Path의 TexturedMesh를 읽어 TexturedMesh 구조체에 넣어주는 코드 작성

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

	// TexturedMesh과 Material 구조체에 있는 데이터를 output Path에 저장해주는 코드 작성

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

	// TexturedMeshwithMaterial Path의 TexturedMeshwithMaterial를 읽어 TexturedMesh와 Material 구조체에 넣어주는 코드 작성

	std::cout << "Loaded TexturedMeshwithMaterial Successfully ! \n";

	in.close();

	return TM_OK;
}
