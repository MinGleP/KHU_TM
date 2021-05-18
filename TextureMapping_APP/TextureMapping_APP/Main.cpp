//#include "Manager.h"
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>


template <typename T>
T littleEndianToNative(const T x) {
	if (true) {
		return x;
	}
	else {
		return x;
	}
}

template <typename T>
T readBinaryLittleEndian(std::istream* stream) {
	T data_little_endian;
	stream->read(reinterpret_cast<char*>(&data_little_endian), sizeof(T));
	return littleEndianToNative(data_little_endian);
}

template <typename T>
void readBinaryLittleEndian(std::istream* stream, std::vector<T>* data) {
	for (size_t i = 0; i < data->size(); ++i) {
		(*data)[i] = readBinaryLittleEndian<T>(stream);
	}
}

Eigen::Vector4d normalizeQuaternion(const Eigen::Vector4d& qvec) {
	const double norm = qvec.norm();
	if (norm == 0) {
		// We do not just use (1, 0, 0, 0) because that is a constant and when used
		// for automatic differentiation that would lead to a zero derivative.
		return Eigen::Vector4d(1.0, qvec(1), qvec(2), qvec(3));
	}
	else {
		return qvec / norm;
	}
}

Eigen::Matrix<double, 3, 4, Eigen::RowMajor> composeProjectionMatrix(const Eigen::Vector4d& qvec,
	const Eigen::Vector3d& tvec) {
	Eigen::Matrix<double, 3, 4, Eigen::RowMajor> proj_matrix;
	const Eigen::Vector4d normalized_qvec = normalizeQuaternion(qvec);
	Eigen::Quaterniond quat(normalized_qvec(0), normalized_qvec(1), normalized_qvec(2), normalized_qvec(3));
	proj_matrix.leftCols<3>() = quat.toRotationMatrix();
	proj_matrix.rightCols<1>() = tvec;
	return proj_matrix;
}

int main()
{
	//std::string cameraPath = "E:/data/blue_glove0415/dense/sparse/cameras.bin";
	//std::string imagePath = "E:/data/blue_glove0415/dense/sparse/images.bin";
	////std::string imagePath = passiveInfoPath[1];

	//std::ifstream in(cameraPath, std::ios::binary);

	//if (!in.is_open())
	//{
	//	std::cout << "Camera.txt Path Wrong ! \n";
	//	return 0;
	//}

	//const size_t num_cameras = readBinaryLittleEndian<uint64_t>(&in);
	//std::cout << "Camera : " << num_cameras << '\n';
	////ImgInfo tmp;
	//std::vector<double> params(4, 0);
	//for (size_t i = 0; i < num_cameras; ++i) {
	//	std::cout << "Camera ID : " << readBinaryLittleEndian<uint32_t>(&in) << '\n';
	//	std::cout << "Camera Model : " << readBinaryLittleEndian<int>(&in) << '\n';
	//	std::cout << "Image Width : " << readBinaryLittleEndian<uint64_t>(&in) << '\n';
	//	std::cout << "Image Height : " << readBinaryLittleEndian<uint64_t>(&in) << '\n';
	//	readBinaryLittleEndian<double>(&in, &params);
	//	for (int j = 0; j < params.size(); j++)
	//	{
	//		std::cout << params[j] << " ";
	//	}
	//	std::cout << "\n\n";
	//}

	//in.close();

	//// Read images.bin
	//std::ifstream inImage(imagePath, std::ios::binary);
	//if (!inImage.is_open())
	//{
	//	std::cout << "images.bin Path Wrong ! \n";
	//	return 0;
	//}

	//const size_t num_reg_images = readBinaryLittleEndian<uint64_t>(&inImage);
	//for (size_t i = 0; i < num_reg_images; ++i) {

	//	uint32_t imageID = readBinaryLittleEndian<uint32_t>(&inImage);
	//	Eigen::Vector4d qVec;
	//	qVec(0) = readBinaryLittleEndian<double>(&inImage);
	//	qVec(1) = readBinaryLittleEndian<double>(&inImage);
	//	qVec(2) = readBinaryLittleEndian<double>(&inImage);
	//	qVec(3) = readBinaryLittleEndian<double>(&inImage);
	//	qVec = normalizeQuaternion(qVec);

	//	//std::cout << "Quad : " << qVec(0) << " " << qVec(1) << " " << qVec(2) << " " << qVec(3) << '\n';

	//	Eigen::Vector3d tVec;
	//	tVec(0) = readBinaryLittleEndian<double>(&inImage);
	//	tVec(1) = readBinaryLittleEndian<double>(&inImage);
	//	tVec(2) = readBinaryLittleEndian<double>(&inImage);

	//	//std::cout << "T : " << tVec(0) << " " << tVec(1) << " " << tVec(2) << '\n';

	//	uint32_t cameraID = readBinaryLittleEndian<uint32_t>(&inImage);

	//	char name_char;
	//	std::string imageName;
	//	do {
	//		inImage.read(&name_char, 1);
	//		if (name_char != '\0') {
	//			imageName += name_char;
	//		}
	//	} while (name_char != '\0');

	//	std::cout << "Camera ID : " << cameraID << "   Image Name : " << imageName << "    Image ID : " << imageID << "\n";
	//	

	//	Eigen::Matrix<double, 3, 4, Eigen::RowMajor> T = composeProjectionMatrix(qVec, tVec);
	//	std::cout << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " " << T(0, 3) << '\n';
	//	std::cout << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << '\n';
	//	std::cout << T(2, 0) << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 3) << "\n\n";
	//	const size_t num_points2D = readBinaryLittleEndian<uint64_t>(&inImage);

	//	std::vector<Eigen::Vector2d> points2D;
	//	points2D.reserve(num_points2D);
	//	std::vector<uint64_t> point3D_ids;
	//	point3D_ids.reserve(num_points2D);
	//	for (size_t j = 0; j < num_points2D; ++j) {
	//		const double x = readBinaryLittleEndian<double>(&inImage);
	//		const double y = readBinaryLittleEndian<double>(&inImage);
	//		points2D.emplace_back(x, y);
	//		point3D_ids.push_back(readBinaryLittleEndian<uint64_t>(&inImage));
	//	}

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
	//}


	return 0;
}