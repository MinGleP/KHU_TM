#pragma once
#include<string>
#include<vector>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<pcl/common/eigen.h>

template< class Real >
struct Point3D
{
	Real coords[3];
	Point3D(void) { coords[0] = coords[1] = coords[2] = Real(0); }
	Point3D(Real v) { coords[0] = coords[1] = coords[2] = v; }
	template< class _Real > Point3D(_Real v0, _Real v1, _Real v2) { coords[0] = Real(v0), coords[1] = Real(v1), coords[2] = Real(v2); }
	template< class _Real > Point3D(const Point3D< _Real >& p) { coords[0] = Real(p[0]), coords[1] = Real(p[1]), coords[2] = Real(p[2]); }
	inline       Real& operator[] (int i) { return coords[i]; }
	inline const Real& operator[] (int i) const { return coords[i]; }
	inline Point3D  operator - (void) const { Point3D q; q.coords[0] = -coords[0], q.coords[1] = -coords[1], q.coords[2] = -coords[2]; return q; }

	template< class _Real > inline Point3D& operator += (Point3D< _Real > p) { coords[0] += Real(p.coords[0]), coords[1] += Real(p.coords[1]), coords[2] += Real(p.coords[2]); return *this; }
	template< class _Real > inline Point3D  operator +  (Point3D< _Real > p) const { Point3D q; q.coords[0] = coords[0] + Real(p.coords[0]), q.coords[1] = coords[1] + Real(p.coords[1]), q.coords[2] = coords[2] + Real(p.coords[2]); return q; }
	template< class _Real > inline Point3D& operator *= (_Real r) { coords[0] *= Real(r), coords[1] *= Real(r), coords[2] *= Real(r); return *this; }
	template< class _Real > inline Point3D  operator *  (_Real r) const { Point3D q; q.coords[0] = coords[0] * Real(r), q.coords[1] = coords[1] * Real(r), q.coords[2] = coords[2] * Real(r); return q; }

	template< class _Real > inline Point3D& operator -= (Point3D< _Real > p) { return ((*this) += (-p)); }
	template< class _Real > inline Point3D  operator -  (Point3D< _Real > p) const { return (*this) + (-p); }
	template< class _Real > inline Point3D& operator /= (_Real r) { return ((*this) *= Real(1. / r)); }
	template< class _Real > inline Point3D  operator /  (_Real r) const { return (*this) * (Real(1.) / r); }

	static Real Dot(const Point3D< Real >& p1, const Point3D< Real >& p2) { return p1.coords[0] * p2.coords[0] + p1.coords[1] * p2.coords[1] + p1.coords[2] * p2.coords[2]; }
	template< class Real1, class Real2 >
	static Real Dot(const Point3D< Real1 >& p1, const Point3D< Real2 >& p2) { return Real(p1.coords[0] * p2.coords[0] + p1.coords[1] * p2.coords[1] + p1.coords[2] * p2.coords[2]); }
};

template< class Real >
struct Point2D
{
	Real coords[2];
	Point2D(void) { coords[0] = coords[1] = Real(0); }
	Point2D(Real v) { coords[0] = coords[1] = v; }
	template< class _Real > Point2D(_Real v0, _Real v1) { coords[0] = Real(v0), coords[1] = Real(v1); }
	template< class _Real > Point2D(const Point2D< _Real >& p) { coords[0] = Real(p[0]), coords[1] = Real(p[1]); }
	inline       Real& operator[] (int i) { return coords[i]; }
	inline const Real& operator[] (int i) const { return coords[i]; }
};

struct Point
{
	Point3D<float> point;
	Point3D<float> normal;
	std::vector<std::pair<int, Point2D<float>>> imageIndexUV;
	std::vector<float> colorConsistencyScore;
	std::vector<Point2D<int>> edgeFaceIndex;
	int index; // 실제 파일 읽어올때의 obj의 point index
	int cameraType;
};

struct Face
{
	uint32_t vertexIndex[3];
	uint32_t uvIndex[3];
	int imageIndex;
};

struct TexturedMesh
{
	std::vector<Point> points;
	std::vector<Face> faces;
};


typedef enum _InputType
{
	TM_INPUT_UNDEFINED = 0,	///< define되기 전
	TM_INPUT_PASSIVE = 1,	///< passive sensor only
	TM_INPUT_ACTIVE = 2,	///< active sensor only
	TM_INPUT_PASSIVE_ACTIVE = 3,	///< passive + active

}InputType;

typedef struct _Parameter
{
	// ---- system setup ---- //
	InputType inputType;

	// ---- path setup ----//	
	std::string meshPath;	///< mesh folder + file name
	std::string imagesFolder; ///< texture images folder

	std::string psvSensorCameraParamPath;	///< passive sensor camera parameter file의 folder + file name (e.g. cameras.txt)
	std::string psvSensorImagesParamPath;	///< passive sensor images parameter file의 folder + file name (e.g. images.txt)
	std::string psvSensorPoints3DParamPath; ///< passive sensor points3D parameter file의 folder + file name (e.g. points3D.txt)
	std::string psvSensorVisParamPath;		///< passive sensor vis parameter file의 folder + file name (e.g. ~~~.vis)
	std::string atvSensorParamPath;			///< active sensor parameter file의 folder + file name

	// 초기화
	_Parameter()
	{
		inputType = TM_INPUT_UNDEFINED;

		meshPath = "";
		imagesFolder = "";

		psvSensorCameraParamPath = "";
		psvSensorImagesParamPath = "";
		psvSensorPoints3DParamPath = "";
		psvSensorVisParamPath = "";
		atvSensorParamPath = "";

	}
}Parameter;

struct ImgInfo
{
	cv::Mat image;
	std::string imageName;
	uint32_t cameraID;
	uint32_t imageID;
	Eigen::Matrix3d K; // Calibration Matrix(Camera to Image)
	Eigen::Matrix<double, 3, 4, Eigen::RowMajor> T; // Transpose Matrix(World to Camera)
};

struct PassiveSensorInfo
{};

struct ActiveSensorInfo
{};

struct Material
{

};