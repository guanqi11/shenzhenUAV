
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
	//ͨ�� epsg �붨������ϵ
	bool SetInputCoordByEPSG(int epsg);
	bool SetOutputCoordByEPSG(int epsg);

	// ��ȡ srs �ַ�������ö�Ӧ�� epsg ��
	bool GetSrsInfo(std::string srs, int& epsg);

	//ͨ�� srs �ַ�����������ϵ
	bool SetInputCoordBySrs(std::string srs);
	bool SetOutputCoordBySrs(std::string srs);

	//����wkt�ı��ļ�����ö�Ӧ�� wkt �ַ���
	int File2WKTString(char* pwktFile, std::string& wktString);

	//ͨ�� wkt �ַ�����������ϵ
	bool SetInputCoordByWKT(std::string wktString);
	bool SetOutputCoordByWKT(std::string wktString);

	//ͨ�� proj4 �ַ�����������ϵ
	bool SetInputCoordByProj4(std::string Proj4String);
	bool SetOutputCoordByProj4(std::string Proj4String);

	//���뵽�������ϵ��ת��
	bool Transform(int count, double* east, double* north, double* height);

	//�������������ϵ��ת��
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

// ���ܣ�wgs84��γ���������ͼ�������껥ת
// ��������ϵ (GCJ-02):���ҹ涨,���ڳ���ĸ��ֵ�ͼϵͳ(����������ʽ)���������ٲ���GCJ-02�Ե���λ�ý����״μ��ܡ��ߵµ�ͼ����Ѷ��ͼ����ʹ��
// �ٶȵ�ͼ����ϵ(BD09):�ٶȱ�׼���ٶ� SDK���ٶȵ�ͼ���ٶ�GeoCoding ʹ�ã�����GCJ-02�����ϵĶ��μ���
class COORDINATESYSTEMCONVERT_API CWgs84MapServiceConvert {
public:
	enum TRANSWAY {
		WGS2GCJ,  // wgs84��γ��ת��������ϵ
		CCJ2WGS,  // ��������ϵתwgs84��γ��
		WGS2BD,   // wgs84��γ��ת�ٶ�����ϵ
		BD2WGS    // �ٶ�����ϵתwgs84��γ��
	};
	bool Transform(double& east, double& north, TRANSWAY way);
};


