
//////////////////////////
//luluping06023@qq.com
//////////////////////////

//CoordinateSystemConvert.h

#pragma once

#include <string>
#include <sstream>
#include <unordered_map>

#ifdef COORDINATESYSTEMCONVERT_EXPORTS
#define COORDINATESYSTEMCONVERT_API __declspec(dllexport)
#else
#define COORDINATESYSTEMCONVERT_API __declspec(dllimport)
#endif


class COORDINATESYSTEMCONVERT_API CCoordinateSystemConvert
{
public:
	CCoordinateSystemConvert(const char* proj_share);
	~CCoordinateSystemConvert();

public:
	//通过 epsg 码定义坐标系
	bool SetInputCoordByEPSG(int epsg);
	bool SetOutputCoordByEPSG(int epsg);

	// 读取 srs 字符串，获得对应的 epsg 码
	bool GetSrsInfo(std::string srs, int& epsg);

	//通过 srs 字符串定义坐标系
	bool SetInputCoordBySrs(std::string srs);
	bool SetOutputCoordBySrs(std::string srs);

	//输入wkt文本文件，获得对应的 wkt 字符串
	int File2WKTString(char* pwktFile, std::string& wktString);

	//通过 wkt 字符串定义坐标系
	bool SetInputCoordByWKT(std::string wktString);
	bool SetOutputCoordByWKT(std::string wktString);

	//通过 proj4 字符串定义坐标系
	bool SetInputCoordByProj4(std::string Proj4String);
	bool SetOutputCoordByProj4(std::string Proj4String);

	//输入到输出坐标系的转换
	bool Transform(int count, double* east, double* north, double* height);

	//输出到输入坐标系的转换
	bool InverseTransform(int count, double* east, double* north, double* height);

	std::unordered_map<std::string, double> getParam(int ESPGValue);

private:
	void* m_pInputCS;
	void* m_pOutputCS;
};

//methods about ENU/NED, use WGS84 ellipsoid
class COORDINATESYSTEMCONVERT_API CLocalCSConvert
{
public:
	CLocalCSConvert(const char* proj_share);
	~CLocalCSConvert();

	////////////////////////////
	//About ENU
	int DefineENUByLBH(double Origin_Lon, double Origin_Lat, double Origin_Hgt = 0.0);
	int DefineENUByCXYZ(double Origin_CX, double Origin_CY, double Origin_CZ);

	int ENUToECEFMatrix(double TransformMatrix[12]);
	int ECEFToENUMatrix(double TransformMatrix[12]);

	//Points[0], Points[1], Points[2] is XYZ of one point
	int PointFromENUToECEF(std::vector<double>& Points);
	int PointFromECEFToENU(std::vector<double>& Points);
	int PointFromENUToECEF(int count, double* X, double* Y, double* Z);
	int PointFromECEFToENU(int count, double* X, double* Y, double* Z);

	//Vectors[0], Vectors[1], Vectors[2] is components of one Vector
	int VectorFromENUToECEF(std::vector<double>& Vectors);
	int VectorFromECEFToENU(std::vector<double>& Vectors);
	int VectorFromENUToECEF(int count, double* Vx, double* Vy, double* Vz);
	int VectorFromECEFToENU(int count, double* Vx, double* Vy, double* Vz);

	//Q[0], Q[1], Q[2], Q[3] is qw qx qy qz
	//T[0], T[1], T[2] is tx ty tz
	int POSQTFromENUToECEF(int count, double* Q, double* T);
	int POSQTFromECEFToENU(int count, double* Q, double* T);

	//Q[0], Q[1], Q[2], Q[3] is qw qx qy qz
	//C[0], C[1], C[2] is cx cy cz
	int POSQCFromENUToECEF(int count, double* Q, double* C);
	int POSQCFromECEFToENU(int count, double* Q, double* C);

	////////////////////////////
	//About NED	
	int DefineNEDByLBH(double Origin_Lon, double Origin_Lat, double Origin_Hgt = 0.0);
	int DefineNEDByCXYZ(double Origin_CX, double Origin_CY, double Origin_CZ);

	int NEDToECEFMatrix(double TransformMatrix[12]);
	int ECEFToNEDMatrix(double TransformMatrix[12]);

	//Points[0], Points[1], Points[2] is XYZ of one point
	int PointFromNEDToECEF(std::vector<double>& Points);
	int PointFromECEFToNED(std::vector<double>& Points);
	int PointFromNEDToECEF(int count, double* X, double* Y, double* Z);
	int PointFromECEFToNED(int count, double* X, double* Y, double* Z);

	//Vectors[0], Vectors[1], Vectors[2] is components of one Vector
	int VectorFromNEDToECEF(std::vector<double>& Vectors);
	int VectorFromECEFToNED(std::vector<double>& Vectors);
	int VectorFromNEDToECEF(int count, double* Vx, double* Vy, double* Vz);
	int VectorFromECEFToNED(int count, double* Vx, double* Vy, double* Vz);

	//Q[0], Q[1], Q[2], Q[3] is qw qx qy qz
	//T[0], T[1], T[2] is tx ty tz
	int POSQTFromNEDToECEF(int count, double* Q, double* T);
	int POSQTFromECEFToNED(int count, double* Q, double* T);

	//Q[0], Q[1], Q[2], Q[3] is qw qx qy qz
	//C[0], C[1], C[2] is cx cy cz
	int POSQCFromNEDToECEF(int count, double* Q, double* C);
	int POSQCFromECEFToNED(int count, double* Q, double* C);

	int Test(int argc, char* argv[]);

private:
	double m_OriginLBH[3] = { 0 };
	double m_OriginECEF[3] = { 0 };
	CCoordinateSystemConvert* m_pCoorCvtWGS84ToCXYZ = nullptr;
};

// 功能：wgs84经纬度坐标与地图服务坐标互转
// 火星坐标系 (GCJ-02):国家规定,国内出版的各种地图系统(包括电子形式)，必须至少采用GCJ-02对地理位置进行首次加密。高德地图、腾讯地图等在使用
// 百度地图坐标系(BD09):百度标准，百度 SDK，百度地图，百度GeoCoding 使用；基于GCJ-02基础上的二次加密
class COORDINATESYSTEMCONVERT_API CWgs84MapServiceConvert {
public:
	enum TRANSWAY {
		WGS2GCJ,  // wgs84经纬度转火星坐标系
		CCJ2WGS,  // 火星坐标系转wgs84经纬度
		WGS2BD,   // wgs84经纬度转百度坐标系
		BD2WGS    // 百度坐标系转wgs84经纬度
	};
	bool Transform(double& east, double& north, TRANSWAY way);
};


