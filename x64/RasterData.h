//////////////////////////
//luluping06023@qq.com
////////////////////////// 

//RasterData.h
#pragma once

#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include "CoordinateSystemConvert.h"
//#include "SatSensorModel.h"

#ifdef RASTERDATA_EXPORTS
#define RASTERDATA_API __declspec(dllexport)
#else
#define RASTERDATA_API __declspec(dllimport)
#endif


#ifndef _DATA_FRAME
#define _DATA_FRAME
struct DataFrame {
	double Lon[4] = { 0 }, Lat[4] = { 0 };        //, ϣ£;     
	double CenterLon = 0, CenterLat = 0;
};
#endif

//RasterBuffer holds a sub block of a raster in memory
class RASTERDATA_API CRasterBuffer
{
public:
	CRasterBuffer();
	~CRasterBuffer();

	bool Create(int nBand, int sRow, int sCol, int nRows, int nCols, GDALDataType eType, int* band_map, bool bHasNoData, double NoDataValue);
	bool SetData(void** pBufferIn);
	bool Release();

	bool GetData(int px, int py, double* pValue);
	bool Interpolate(double px, double py, double* pValue);
	bool LinearStretchingToByte(unsigned char** pOutput); //for each band
	bool ConvertToGray(); //combine all band to one grey band and reset CRasterBuffer
	bool ConvertToGray(unsigned char* pOutput); //combine all band to one grey band
	bool ZoomOutToGray(int Zoom, unsigned char* pOutput); //Zoom should be >= 1

	void** pBuffer = nullptr;
	int m_nBand = 0;
	int m_sRow = 0;
	int m_sCol = 0;
	int m_nRows = 0;
	int m_nCols = 0;
	GDALDataType m_eType = GDT_Unknown;
	int* m_band_map = nullptr;
	bool m_bHasNoData = false;
	double m_NoDataValue = 0;
};

//CRasterData is the base class of a raster
class RASTERDATA_API CRasterData
{
public:
	CRasterData();
	~CRasterData();

	bool Open(const char* lpstrPathName);	//read only
	bool Update(const char* lpstrPathName); //modify
	void Close();

	bool Read(int nBand, int sRow, int sCol, int nRows, int nCols, GDALDataType eType, int* band_map);
	bool ReadGrayData(int sRow, int sCol, int nRows, int nCols);

	bool InterpolateByPixelCoor(double px, double py, double* pValue);

	bool Create(int nCols, int nRows, int nBand, GDALDataType eType); //create an image in memory and save later
	bool SaveAsGeoTiff(const char* lpstrPathName, char** papszOptions = nullptr);
	bool SaveAsCOG(const char* lpstrPathName, char** papszOptions = nullptr);
	bool SaveAsJPEG(const char* lpstrPathName, char** papszOptions = nullptr);

	bool CreateGeoTiff(const char* lpstrPathName, int nCols, int nRows, int nBand, GDALDataType eType, char** papszOptions = nullptr);
	bool Write(void** pBuffer, int nBand, int sRow, int sCol, int nRows, int nCols, GDALDataType eType, int* band_map);

	int	         GetRows();
	int	         GetCols();
	int	         GetBands();
	GDALDataType GetDataType();
	std::string  GetPathName();

	//All Band use same NoDataValue
	bool         GetNoDataValue(double& NoDataValue);
	bool         SetNoDataValue(double NoDataValue);

	//Each Band uses different NoDataValue
	bool         GetNoDataValue(bool* bHasNoDataValue, double* NoDataValue);
	bool         SetNoDataValue(double* NoDataValue);

	CRasterBuffer m_pData; //hold the read data
protected:
	std::string m_DataPath = "";
	GDALDatasetUniquePtr m_pDataset = nullptr;
};


//DEM
class RASTERDATA_API CDEM : public CRasterData
{
public:
	CDEM();
	~CDEM();

	bool Open(const char* lpstrPathName, const char* proj_share = nullptr);	//read only
	bool Init(const char* proj_share);   //can be used after CRasterData.Create

	bool PixelCoorToOriginalXYZ(double px, double py, double& X, double& Y, double& Z); //XYZ in dem itself
	bool PixelCoorToWGS84LBH(double px, double py, double& Lon, double& Lat, double& Height);
	bool PixelCoorToWGS84CXYZ(double px, double py, double& CX, double& CY, double& CZ);

	bool OriginalXYZToPixelCoor(double X, double Y, double Z, double& px, double& py);
	bool WGS84LBHToPixelCoor(double Lon, double Lat, double Height, double& px, double& py);
	bool WGS84CXYZToPixelCoor(double CX, double CY, double CZ, double& px, double& py);

	bool InterpolateByWGS84(double Lon, double Lat, double* pValue);
	bool InterpolateByPixelCoor(double px, double py, double* pValue);

	bool         GetGeoTransform(double* pGeoMatrix6);
	bool         SetGeoTransform(double* pGeoMatrix6);
	const char* GetProjectionRef();
	bool         SetProjection(const char* projection);
	bool         SetSpatialRef(OGRSpatialReference* poSRS);

	bool         GetGeoRangeInWGS84(int sRow, int sCol, int nRows, int nCols, double X[4], double Y[4]);
	bool         GetGeoRangeInOriginalXYZ(int sRow, int sCol, int nRows, int nCols, double X[4], double Y[4]);

	double       GetXGsd();
	double       GetYGsd();

	double px_t;
	double py_t;
private:
	double m_GeoMatrix[6] = { 0 };
	DataFrame m_GeoRange;
	CCoordinateSystemConvert* m_pCoorCvtToWGS84 = nullptr;
	CCoordinateSystemConvert* m_pCoorCvtToCXYZ = nullptr;
};

//DOM
class RASTERDATA_API CDOM : public CRasterData
{
public:
	CDOM();
	~CDOM();

	bool Open(const char* lpstrPathName, const char* proj_share = nullptr);	//read only
	bool Init(const char* proj_share);   //can be used after CRasterData.Create

	bool PixelCoorToOriginalXYZ(double px, double py, double& X, double& Y, double& Z); //XYZ in dom itself, Z is always 0
	bool PixelCoorToWGS84LBH(double px, double py, double& Lon, double& Lat, double& Height); //Height is always 0

	bool OriginalXYZToPixelCoor(double X, double Y, double Z, double& px, double& py);
	bool WGS84LBHToPixelCoor(double Lon, double Lat, double Height, double& px, double& py);

	bool InterpolateByWGS84(double Lon, double Lat, double* pValue);
	bool InterpolateByPixelCoor(double px, double py, double* pValue);

	bool         GetGeoTransform(double* pGeoMatrix6);
	bool         SetGeoTransform(double* pGeoMatrix6);
	const char* GetProjectionRef();
	bool         SetProjection(const char* projection);
	bool         SetSpatialRef(OGRSpatialReference* poSRS);

	bool         GetGeoRangeInWGS84(int sRow, int sCol, int nRows, int nCols, double X[4], double Y[4]);
	bool         GetGeoRangeInOriginalXYZ(int sRow, int sCol, int nRows, int nCols, double X[4], double Y[4]);

	double       GetXGsd();
	double       GetYGsd();

private:
	double m_GeoMatrix[6] = { 0 };
	DataFrame m_GeoRange;
	CCoordinateSystemConvert* m_pCoorCvtToWGS84 = nullptr;
};

//Satellite Image
//class RASTERDATA_API CSat : public CRasterData
//{
//public:
//	CSat();
//	~CSat();
//
//	bool Open(SatSensorModel ModelType, const char* lpstrPathName, const char* proj_share = nullptr);	//read only
//
//	bool SetSensorModel(CSatSensorModel* pModel);
//	CSatSensorModel* GetSensorModel();
//
//	bool InterpolateByWGS84(double gx, double gy, double gz, double* pValue);
//	bool InterpolateByCXYZ(double gx, double gy, double gz, double* pValue);
//	bool InterpolateByXYZ(double gx, double gy, double gz, double* pValue);
//	bool InterpolateByPixelCoor(double px, double py, double* pValue);
//
//	bool GetGeoRangeInWGS84(int sRow, int sCol, int nRows, int nCols, double gz, double X[4], double Y[4]);
//
//	DataFrame m_GeoRange;
//
//private:
//	CSatSensorModel* m_pSensorModel = nullptr;
//};

RASTERDATA_API bool MeterGsdToDegreeGsd(double GsdInMeter, double LatInDegree, double& LonGsdInDegree, double& LatGsdInDegree);
RASTERDATA_API bool MergeDEMToGeoTiff(char* DemListFile, char* ResultFilePath);
RASTERDATA_API bool OrthoRectification(char* proj_share, char* SatFile, char* DemFile, char* DomFile, double GsdInMeter, double NoDataValue);
RASTERDATA_API bool OrthoRectificationGivenPara(char* proj_share, char* SatFile, char* DemFile, char* DomFile, char* ParaFromDom);
RASTERDATA_API bool PansharpenImagery(char* proj_share, char* PANDomFile, char* MSSDomFile, char* PansharpenDomFile);
RASTERDATA_API bool Convert2ByteCOG(char* proj_share, char* InputDomFile, char* OutputDomFile, bool bDeleteInputDomFile);
RASTERDATA_API bool Convert2ByteTIF(char* InputImgFile, char* OutputImgFile, bool bDeleteInputImgFile);
RASTERDATA_API bool Convert2ThreeBandsTIF(char* InputImgFile, char* OutputImgFile, bool bDeleteInputImgFile);
//RASTERDATA_API bool SatellitePhoto_2_HugeDEM(CSatSensorModel* pSatModel, CDEM* pDem, double sample, double line, double* Lon84, double* Lat84, double* Alt84);

