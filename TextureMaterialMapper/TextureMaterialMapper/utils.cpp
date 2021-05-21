#include "pch.h"
#include<iostream>
#include<vector>
#ifdef max
#undef max
#undef min
#endif  //max
#include<pcl/common/eigen.h>

template <typename T>
T littleEndianToNative(const T x) 
{
	if (true) {
		return x;
	}
	else {
		return x;
	}
}

template <typename T>
T readBinaryLittleEndian(std::istream* stream) 
{
	T data_little_endian;
	stream->read(reinterpret_cast<char*>(&data_little_endian), sizeof(T));
	return littleEndianToNative(data_little_endian);
}

template <typename T>
void readBinaryLittleEndian(std::istream* stream, std::vector<T>* data) 
{
	for (size_t i = 0; i < data->size(); ++i) {
		(*data)[i] = readBinaryLittleEndian<T>(stream);
	}
}

bool IsNotWhiteSpace(const int character) 
{
	return character != ' ' && character != '\n' && character != '\r' &&
		character != '\t';
}

void StringLeftTrim(std::string* str) 
{
	str->erase(str->begin(),
		std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
}

void StringRightTrim(std::string* str) 
{
	str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(),
		str->end());
}

void StringTrim(std::string* str) 
{
	StringLeftTrim(str);
	StringRightTrim(str);
}

Eigen::Vector4d normalizeQuaternion(const Eigen::Vector4d& qvec) 
{
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
	const Eigen::Vector3d& tvec) 
{
	Eigen::Matrix<double, 3, 4, Eigen::RowMajor> proj_matrix;
	const Eigen::Vector4d normalized_qvec = normalizeQuaternion(qvec);
	Eigen::Quaterniond quat(normalized_qvec(0), normalized_qvec(1), normalized_qvec(2), normalized_qvec(3));
	proj_matrix.leftCols<3>() = quat.toRotationMatrix();
	proj_matrix.rightCols<1>() = tvec;
	return proj_matrix;
}

