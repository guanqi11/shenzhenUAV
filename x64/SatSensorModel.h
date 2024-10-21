
//////////////////////////
//luluping06023@qq.com
////////////////////////// 

//SatSensorModel.h

#pragma once
#include <string>

#ifdef SATSENSORMODEL_EXPORTS
#define SATSENSORMODEL_API __declspec(dllexport)
#else
#define SATSENSORMODEL_API __declspec(dllimport)
#endif


// 卫星传感器模型
#ifndef SAT_SENSOR_MODEL
#define SAT_SENSOR_MODEL
enum SatSensorModel {
	UNKNOWN_SENSORMODEL = 0,
	RFM,    // RPC model
	BROOM   // 线阵推扫姿轨模型
};
#endif

class SATSENSORMODEL_API CSatSensorModel
{
public:
	CSatSensorModel();
	~CSatSensorModel();

	virtual bool Open(const char* lpstrPathName, const char* proj_share);
	virtual bool Save() = 0;

	virtual bool GetOriParaFile(std::string& FilePath) = 0;

	//gx, gy, gz in WGS84_LBH
	virtual bool Pixel_2_LBH(double sx, double sy, double alt, double* gx, double* gy, double* gz) = 0;
	virtual bool LBH_2_Pixel(double gx, double gy, double gz, double* sx, double* sy) = 0;

	//gx, gy, gz in WGS84_ECEF
	virtual bool Pixel_2_CXYZ(double sx, double sy, double alt, double* gx, double* gy, double* gz) = 0;
	virtual bool CXYZ_2_Pixel(double gx, double gy, double gz, double* sx, double* sy) = 0;

	//gx, gy, gz in any cartesian world coordinate system
	virtual bool Pixel_2_XYZ(double sx, double sy, double distance, double* gx, double* gy, double* gz) = 0;
	virtual bool XYZ_2_Pixel(double gx, double gy, double gz, double* sx, double* sy) = 0;

	//RpcXY is without aop para
	virtual bool RpcXY_2_LBH(double sx, double sy, double alt, double* gx, double* gy, double* gz) = 0;
	virtual bool LBH_2_RpcXY(double gx, double gy, double gz, double* sx, double* sy) = 0;

	virtual bool RpcXY_2_CXYZ(double sx, double sy, double alt, double* gx, double* gy, double* gz) = 0;
	virtual bool CXYZ_2_RpcXY(double gx, double gy, double gz, double* sx, double* sy) = 0;

	virtual bool RpcXY_2_XYZ(double sx, double sy, double distance, double* gx, double* gy, double* gz) = 0;
	virtual bool XYZ_2_RpcXY(double gx, double gy, double gz, double* sx, double* sy) = 0;

	virtual bool RpcXY_2_PixelXY(double rx, double ry, double* sx, double* sy) = 0;
	virtual bool PixelXY_2_RpcXY(double sx, double sy, double* rx, double* ry) = 0;

	virtual bool GetViewDirection(double sx, double sy, double& Vx, double& Vy, double& Vz) = 0;
	virtual bool GetProjectionCenter(double sx, double sy, double& Xs, double& Ys, double& Zs) = 0;

	SatSensorModel GetSensorType() { return m_nSensorType; };
	void SetSensorType(SatSensorModel ModelType) { m_nSensorType = ModelType; };

	virtual bool Test() = 0;

protected:
	std::string    m_ImagePath = "";
	SatSensorModel m_nSensorType = SatSensorModel::UNKNOWN_SENSORMODEL;
	void* m_pCoorCvtCXYZToWGS84 = nullptr;
};

SATSENSORMODEL_API CSatSensorModel* CreateSatSensorModel(SatSensorModel ModelType, const char* lpstrPathName, const char* proj_share);
SATSENSORMODEL_API void ReleaseSatSensorModel(CSatSensorModel* pSensorModel);

SATSENSORMODEL_API bool LoadAop(const char* AopFile, double* Aop6);
SATSENSORMODEL_API bool LoadImgAOP(const char* ImgFilePath, std::string& AopFile, double* Aop6);

