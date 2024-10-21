
#define _CRT_SECURE_NO_WARNINGS

#include "RasterData.h"
#include "gdal_priv.h"
#include "IndirectAdjustmentX.h"
#include <Windows.h>
#include <algorithm>
#include <io.h>

void AtxZoomOutGrayData(unsigned char* pSrc, int w, int h, int x0, int y0, unsigned char* pTar, int bx, int by, int w1, int h1, int zoom) {
	unsigned char* p0 = NULL;
	unsigned char* p = pSrc + y0 * w + x0;
	unsigned char* p1 = pTar + by * w1 + bx;
	float d = (float)(zoom * zoom);

	for (int i = 0; i < h1; i++) {
		for (int j = 0; j < w1; j++) {
			p0 = p + (i * w + j) * zoom;

			int all = 0;
			for (int k = 0; k < zoom; k++) {
				for (int l = 0; l < zoom; l++) {
					all += *(p0 + l);
				}
				p0 += w;
			}
			*(p1 + j) = (unsigned char)(all / d);
		}
		p1 += w1;
	}
};

bool IsNoValue(double Value, double NoDataValue)
{
	if (Value == NoDataValue || Value < -3.4028e+038) {
		return true;
	}
	else {
		return false;
	}
}

CRasterBuffer::CRasterBuffer()
{
}

CRasterBuffer::~CRasterBuffer()
{
	Release();
}

bool CRasterBuffer::Create(int nBand, int sRow, int sCol, int nRows, int nCols, GDALDataType eType, int* band_map, bool bHasNoData, double NoDataValue)
{
	Release();

	unsigned long long Total = (unsigned long long)nCols * (unsigned long long)nRows;

	if (eType == GDT_Byte)
	{
		pBuffer = (void**)new unsigned char* [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new unsigned char[Total];
			memset(pBuffer[i], 0, Total * sizeof(unsigned char));
		}
	}
	else if (eType == GDT_UInt16)
	{
		pBuffer = (void**)new unsigned short* [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new unsigned short[Total];
			memset(pBuffer[i], 0, Total * sizeof(unsigned short));
		}
	}
	else if (eType == GDT_Int16)
	{
		pBuffer = (void**)new short* [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new short[Total];
			memset(pBuffer[i], 0, Total * sizeof(short));
		}
	}
	else if (eType == GDT_UInt32)
	{
		pBuffer = (void**)new unsigned int* [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new unsigned int[Total];
			memset(pBuffer[i], 0, Total * sizeof(unsigned int));
		}
	}
	else if (eType == GDT_Int32)
	{
		pBuffer = (void**)new int* [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new int[Total];
			memset(pBuffer[i], 0, Total * sizeof(int));
		}
	}
	/*else if (eType == GDT_UInt64)
	{
		pBuffer = (void**)new GUInt64 * [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new GUInt64[Total];
			memset(pBuffer[i], 0, Total * sizeof(GUInt64));
		}
	}
	else if (eType == GDT_Int64)
	{
		pBuffer = (void**)new GInt64 * [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new GInt64[Total];
			memset(pBuffer[i], 0, Total * sizeof(GInt64));
		}
	}*/
	else if (eType == GDT_Float32)
	{
		pBuffer = (void**)new float* [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new float[Total];
			memset(pBuffer[i], 0, Total * sizeof(float));
		}
	}
	else if (eType == GDT_Float64)
	{
		pBuffer = (void**)new double* [nBand];
		for (int i = 0; i < nBand; i++) {
			pBuffer[i] = new double[Total];
			memset(pBuffer[i], 0, Total * sizeof(double));
		}
	}
	else {
		return false;
	}

	m_nBand = nBand;
	m_sRow = sRow;
	m_sCol = sCol;
	m_nRows = nRows;
	m_nCols = nCols;
	m_eType = eType;
	m_bHasNoData = bHasNoData;
	m_NoDataValue = NoDataValue;
	m_band_map = new int[nBand];
	memcpy(m_band_map, band_map, nBand * sizeof(int));

	return true;
}

bool CRasterBuffer::SetData(void** pBufferIn)
{
	if (pBuffer == nullptr) {
		return 0;
	}

	unsigned long long Total = (unsigned long long)m_nCols * (unsigned long long)m_nRows;

	if (m_eType == GDT_Byte)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(unsigned char));
		}
	}
	else if (m_eType == GDT_UInt16)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(unsigned short));
		}
	}
	else if (m_eType == GDT_Int16)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(short));
		}
	}
	else if (m_eType == GDT_UInt32)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(unsigned int));
		}
	}
	else if (m_eType == GDT_Int32)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(int));
		}
	}
	/*else if (m_eType == GDT_UInt64)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(GUInt64));
		}
	}
	else if (m_eType == GDT_Int64)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(GInt64));
		}
	}*/
	else if (m_eType == GDT_Float32)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(float));
		}
	}
	else if (m_eType == GDT_Float64)
	{
		for (int i = 0; i < m_nBand; i++) {
			memcpy(pBuffer[i], pBufferIn[i], Total * sizeof(double));
		}
	}
	else {
		return false;
	}

	return 1;
}

bool CRasterBuffer::Release()
{
	if (pBuffer) {
		for (int i = 0; i < m_nBand; i++) {
			delete[] pBuffer[i]; pBuffer[i] = nullptr;
		}
		delete[] pBuffer; pBuffer = nullptr;
	}

	if (m_band_map) {
		delete[] m_band_map; m_band_map = nullptr;
	}

	m_nBand = 0;
	m_sRow = 0;
	m_sCol = 0;
	m_nRows = 0;
	m_nCols = 0;
	m_eType = GDT_Unknown;
	m_bHasNoData = false;
	m_NoDataValue = 0;

	return true;
}

template<typename T>
bool BilinearInterpolation(T** pBuffer, int nBand, int nRows, int nCols, bool bHasNoData, double NoDataValue, double px, double py, double* pValue) {
	size_t col = static_cast<size_t>(px);
	size_t row = static_cast<size_t>(py);

	if (col < 0 || col >= nCols - 1 || row < 0 || row >= nRows - 1) {
		return false;
	}

	for (int j = 0; j < nBand; j++) {
		T* ptr_buffer = pBuffer[j];

		T* z1 = ptr_buffer + row * nCols + col;
		T* z2 = z1 + 1;
		T* z3 = z1 + nCols;
		T* z4 = z3 + 1;

		if (bHasNoData) {
			if (IsNoValue(*z1, NoDataValue) || IsNoValue(*z2, NoDataValue) || IsNoValue(*z3, NoDataValue) || IsNoValue(*z4, NoDataValue)) {
				return false;
			}
		}

		double px1 = px - col;
		double py1 = py - row;
		double px2 = 1 - px1;
		double py2 = 1 - py1;
		pValue[j] = px2 * py2 * (*z1) + px1 * py2 * (*z2) + px2 * py1 * (*z3) + px1 * py1 * (*z4);
	}

	return true;
}

bool CRasterBuffer::GetData(int px, int py, double* pValue)
{
	if (px < 0 || px >= m_nCols || py < 0 || py >= m_nRows) {
		return false;
	}

	if (m_eType == GDT_Byte) {
		for (int j = 0; j < m_nBand; j++) {
			GByte* ptr_buffer = (GByte*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else if (m_eType == GDT_UInt16) {
		for (int j = 0; j < m_nBand; j++) {
			GUInt16* ptr_buffer = (GUInt16*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else if (m_eType == GDT_Int16) {
		for (int j = 0; j < m_nBand; j++) {
			GInt16* ptr_buffer = (GInt16*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else if (m_eType == GDT_UInt32) {
		for (int j = 0; j < m_nBand; j++) {
			GUInt32* ptr_buffer = (GUInt32*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else if (m_eType == GDT_Int32) {
		for (int j = 0; j < m_nBand; j++) {
			GInt32* ptr_buffer = (GInt32*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else if (m_eType == GDT_UInt64) {
		for (int j = 0; j < m_nBand; j++) {
			GUInt64* ptr_buffer = (GUInt64*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else if (m_eType == GDT_Int64) {
		for (int j = 0; j < m_nBand; j++) {
			GInt64* ptr_buffer = (GInt64*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else if (m_eType == GDT_Float32) {
		for (int j = 0; j < m_nBand; j++) {
			float* ptr_buffer = (float*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else if (m_eType == GDT_Float64) {
		for (int j = 0; j < m_nBand; j++) {
			double* ptr_buffer = (double*)pBuffer[j];
			pValue[j] = (double)(ptr_buffer[py * m_nCols + px]);
			if (IsNoValue(pValue[j], m_NoDataValue)) {
				return false;
			}
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool CRasterBuffer::Interpolate(double px, double py, double* pValue)
{
	if (m_eType == GDT_Byte)
	{
		GByte** pTemp = (GByte**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}
	else if (m_eType == GDT_UInt16)
	{
		GUInt16** pTemp = (GUInt16**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}
	else if (m_eType == GDT_Int16)
	{
		GInt16** pTemp = (GInt16**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}
	else if (m_eType == GDT_UInt32)
	{
		GUInt32** pTemp = (GUInt32**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}
	else if (m_eType == GDT_Int32)
	{
		GInt32** pTemp = (GInt32**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}
	/*else if (m_eType == GDT_UInt64)
	{
		GUInt64** pTemp = (GUInt64**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}
	else if (m_eType == GDT_Int64)
	{
		GInt64** pTemp = (GInt64**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}*/
	else if (m_eType == GDT_Float32)
	{
		float** pTemp = (float**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}
	else if (m_eType == GDT_Float64)
	{
		double** pTemp = (double**)pBuffer;
		return BilinearInterpolation(pTemp, m_nBand, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, px, py, pValue);
	}
	else
	{
		return false;
	}
}

template<typename T>
bool FindMinMax(T* ptr_buffer, int nRows, int nCols, bool bHasNoData, double NoDataValue, double& inputMin, double& inputMax) {
	inputMin = DBL_MAX;
	inputMax = DBL_MIN;
	unsigned long nCount = nRows * nCols;

	for (unsigned long i = 0; i < nCount; i++) {
		if (bHasNoData) {
			if (ptr_buffer[i] == NoDataValue || ptr_buffer[i] < -3.4028e+038) {
				continue;
			}
		}

		if (ptr_buffer[i] < inputMin) {
			inputMin = (double)ptr_buffer[i];
		}
		if (ptr_buffer[i] > inputMax) {
			inputMax = ptr_buffer[i];
		}
	}

	return true;
}

template<typename T>
bool LinearStretching(T* ptr_buffer, int nRows, int nCols, bool bHasNoData, double NoDataValue, double inputMin, double inputMax, unsigned char* pOutput) {
	if (inputMin > inputMax)
	{
		return false;
	}

	double scale = 255.0f / (inputMax - inputMin);

	unsigned long nCount = nRows * nCols;
	for (unsigned long i = 0; i < nCount; i++)
	{
		if (bHasNoData) {
			if (ptr_buffer[i] == NoDataValue) {
				pOutput[i] = 0;
				continue;
			}
		}
		pOutput[i] = (GByte)(scale * (ptr_buffer[i] - inputMin));
	}
	return true;
}

bool CRasterBuffer::LinearStretchingToByte(unsigned char** pOutput)
{
	double inputMin = DBL_MAX;
	double inputMax = DBL_MIN;

	if (m_eType == GDT_Byte)
	{
		for (int j = 0; j < m_nBand; j++) {
			GByte* pInputTemp = (GByte*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}
	else if (m_eType == GDT_UInt16)
	{
		for (int j = 0; j < m_nBand; j++) {
			GUInt16* pInputTemp = (GUInt16*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}
	else if (m_eType == GDT_Int16)
	{
		for (int j = 0; j < m_nBand; j++) {
			GInt16* pInputTemp = (GInt16*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}
	else if (m_eType == GDT_UInt32)
	{
		for (int j = 0; j < m_nBand; j++) {
			GUInt32* pInputTemp = (GUInt32*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}
	else if (m_eType == GDT_Int32)
	{
		for (int j = 0; j < m_nBand; j++) {
			GInt32* pInputTemp = (GInt32*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}
	/*else if (m_eType == GDT_UInt64)
	{
		for (int j = 0; j < m_nBand; j++) {
			GUInt64* pInputTemp = (GUInt64*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}
	else if (m_eType == GDT_Int64)
	{
		for (int j = 0; j < m_nBand; j++) {
			GInt64* pInputTemp = (GInt64*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}*/
	else if (m_eType == GDT_Float32)
	{
		for (int j = 0; j < m_nBand; j++) {
			float* pInputTemp = (float*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}
	else if (m_eType == GDT_Float64)
	{
		for (int j = 0; j < m_nBand; j++) {
			double* pInputTemp = (double*)pBuffer[j];

			FindMinMax(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax);
			LinearStretching(pInputTemp, m_nRows, m_nCols, m_bHasNoData, m_NoDataValue, inputMin, inputMax, pOutput[j]);
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool CRasterBuffer::ConvertToGray()
{
	unsigned char** pGrayData = (unsigned char**)new unsigned char* [1];
	unsigned long long Total = (unsigned long long)m_nCols * (unsigned long long)m_nRows;
	pGrayData[0] = new unsigned char[Total];
	memset(pGrayData[0], 0, Total * sizeof(unsigned char));

	ConvertToGray(pGrayData[0]);

	if (pBuffer) {
		for (int i = 0; i < m_nBand; i++) {
			delete[] pBuffer[i]; pBuffer[i] = nullptr;
		}
		delete[] pBuffer; pBuffer = nullptr;
	}

	if (m_band_map) {
		delete[] m_band_map; m_band_map = nullptr;
	}

	m_nBand = 1;
	m_eType = GDT_Byte;
	m_band_map = new int[1];
	m_band_map[0] = 1;

	pBuffer = (void**)pGrayData;

	return true;
}

bool CRasterBuffer::ConvertToGray(unsigned char* pOutput)
{
	unsigned char** pByte = new unsigned char* [m_nBand];
	for (int i = 0; i < m_nBand; i++) {
		unsigned long long Total = (unsigned long long)m_nCols * (unsigned long long)m_nRows;
		pByte[i] = new unsigned char[Total];
		memset(pByte[i], 0, Total * sizeof(unsigned char));
	}

	LinearStretchingToByte(pByte);

	long index = 0;
	for (int i = 0; i < m_nRows; i++) {
		for (int j = 0; j < m_nCols; j++) {
			double Sum = 0;
			for (int b = 0; b < m_nBand; b++) {
				Sum += pByte[b][index];
			}
			unsigned char Grey = (unsigned char)(Sum / m_nBand);
			pOutput[index] = Grey;
			index++;
		}
	}

	if (pByte) {
		for (int i = 0; i < m_nBand; i++) {
			delete[] pByte[i]; pByte[i] = nullptr;
		}
		delete[] pByte; pByte = nullptr;
	}

	return true;
}

bool CRasterBuffer::ZoomOutToGray(int Zoom, unsigned char* pOutput)
{
	if (Zoom < 1) {
		return false;
	}

	unsigned long long Total = (unsigned long long)m_nCols * (unsigned long long)m_nRows;
	unsigned char* pByte = new unsigned char[Total];
	memset(pByte, 0, Total * sizeof(unsigned char));
	ConvertToGray(pByte);

	int Cols = m_nCols / Zoom;
	int Rows = m_nRows / Zoom;
	AtxZoomOutGrayData(pByte, m_nCols, m_nRows, 0, 0, pOutput, 0, 0, Cols, Rows, Zoom);

	delete[] pByte; pByte = nullptr;
	return true;
}

/////////////////////////////////////////////////////////////

CRasterData::CRasterData()
{
	GDALAllRegister();                                 // GDAL所有操作都需要先注册格式
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO"); // 支持中文路径
}

CRasterData::~CRasterData()
{
	Close();
}

bool CRasterData::Open(const char* lpstrPathName)
{
	Close();

	const GDALAccess eAccess = GA_ReadOnly;
	m_pDataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen(lpstrPathName, eAccess)));
	if (!m_pDataset)
	{
		printf("Error: Can't Open file %s !\n\n", lpstrPathName);
		return false;
	}

	m_DataPath = lpstrPathName;
	return true;
}

bool CRasterData::Update(const char* lpstrPathName)
{
	Close();

	const GDALAccess eAccess = GA_Update;
	m_pDataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen(lpstrPathName, eAccess)));
	if (!m_pDataset)
	{
		printf("Error: Can't update file %s !\n\n", lpstrPathName);
		return false;
	}

	m_DataPath = lpstrPathName;
	return true;
}

void CRasterData::Close()
{
	if (m_pDataset) {
		m_DataPath = "";
		m_pDataset.reset();
	}
}

bool CRasterData::Create(int nCols, int nRows, int nBand, GDALDataType eType)
{
	Close();

	GDALDriver* poDriverMem = GetGDALDriverManager()->GetDriverByName("MEM");
	if (poDriverMem) {
		GDALDataset* poDstDSMem = poDriverMem->Create("", nCols, nRows, nBand, eType, NULL);

		if (poDstDSMem != NULL)
		{
			m_pDataset.reset(poDstDSMem);
			return true;
		}
	}

	printf("Error: Can't create!\n\n");
	return false;
}

bool CRasterData::CreateGeoTiff(const char* lpstrPathName, int nCols, int nRows, int nBand, GDALDataType eType, char** papszOptions)
{
	Close();

	GDALDriver* poDriver = GetGDALDriverManager()->GetDriverByName("GTiff");

	if (poDriver) {
		GDALDataset* poDstDS = poDriver->Create(lpstrPathName, nCols, nRows, nBand, eType, papszOptions);

		if (poDstDS != NULL)
		{
			m_pDataset.reset(poDstDS);
			return true;
		}
	}

	printf("Error: Can't create!\n\n");
	return false;
}

bool CRasterData::SaveAsGeoTiff(const char* lpstrPathName, char** papszOptions)
{
	if (!m_pDataset)
	{
		printf("Error: Open/Create file first!\n\n");
		return false;
	}

	//配置图像信息
	papszOptions = CSLSetNameValue(papszOptions, "TILED", "YES");
	papszOptions = CSLSetNameValue(papszOptions, "COMPRESS", "LZW");
	papszOptions = CSLSetNameValue(papszOptions, "INTERLEAVE", "BAND");
	papszOptions = CSLSetNameValue(papszOptions, "COPY_SRC_OVERVIEWS", "YES");
	//papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "YES");	
	//papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "IF_NEEDED");
	papszOptions = CSLSetNameValue(papszOptions, "NUM_THREADS", "4");

	GDALDriver* pDstDriver = (GDALDriver*)GDALGetDriverByName("GTIFF");
	if (pDstDriver != NULL)
	{
		GDALDataset* pDataSet = pDstDriver->CreateCopy(lpstrPathName, m_pDataset.get(), FALSE, papszOptions, GDALTermProgress, NULL);
		if (pDataSet != NULL)
		{
			GDALClose((GDALDatasetH)pDataSet);
			return true;
		}
	}

	printf("SaveAsGeoTiff Error!\n\n");
	return false;
}

bool CRasterData::SaveAsCOG(const char* lpstrPathName, char** papszOptions)
{
	if (!m_pDataset)
	{
		printf("Error: Open/Create file first!\n\n");
		return false;
	}

	papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "YES");

	GDALDriver* pDstDriver = (GDALDriver*)GDALGetDriverByName("COG");
	if (pDstDriver != NULL)
	{
		GDALDataset* pDataSet = pDstDriver->CreateCopy(lpstrPathName, m_pDataset.get(), FALSE, papszOptions, GDALTermProgress, NULL);
		if (pDataSet != NULL)
		{
			GDALClose((GDALDatasetH)pDataSet);
			return true;
		}
	}

	printf("SaveAsCOG Error!\n\n");
	return false;
}

bool CRasterData::SaveAsJPEG(const char* lpstrPathName, char** papszOptions)
{
	if (!m_pDataset)
	{
		printf("Error: Open/Create file first!\n\n");
		return false;
	}

	GDALDriver* pDstDriver = (GDALDriver*)GDALGetDriverByName("JPEG");
	if (pDstDriver != NULL)
	{
		GDALDataset* pDataSet = pDstDriver->CreateCopy(lpstrPathName, m_pDataset.get(), FALSE, papszOptions, GDALTermProgress, NULL);
		if (pDataSet != NULL)
		{
			GDALClose((GDALDatasetH)pDataSet);
			return true;
		}
	}

	printf("SaveAsJPEG Error!\n\n");
	return false;
}

bool CRasterData::Read(int nBand, int sRow, int sCol, int nRows, int nCols, GDALDataType eType, int* band_map)
{
	if (!m_pDataset)
	{
		printf("Error: Open file first!\n\n");
		return false;
	}

	int nBandSrc = m_pDataset->GetRasterCount();
	if (nBand > nBandSrc) {
		printf("Error: nBand > nBandSrc!\n\n");
		return false;
	}
	for (int i = 0; i < nBand; i++) {
		int BandIndex = band_map[i];
		if (BandIndex <1 || BandIndex > nBandSrc) {
			printf("Error: invalid band_map!\n\n");
			return false;
		}
	}

	int nColsSrc = GetCols();
	int nRowsSrc = GetRows();
	if (sRow > nRowsSrc - 1) {
		printf("Error: invalid sRow!\n\n");
		return false;
	}
	if (sCol > nColsSrc - 1) {
		printf("Error: invalid sCol!\n\n");
		return false;
	}
	if (sRow < 0) {
		sRow = 0;
	}
	if (sCol < 0) {
		sCol = 0;
	}
	if (sRow + nRows > nRowsSrc) {
		nRows = nRowsSrc - sRow;
	}
	if (sCol + nCols > nColsSrc) {
		nCols = nColsSrc - sCol;
	}
	if (nRows < 1 || nCols < 1) {
		printf("Error: invalid nRows or nCols!\n\n");
		return false;
	}

	bool bHasNoData = false;
	double NoDataValue = 0;
	bHasNoData = GetNoDataValue(NoDataValue);

	if (!m_pData.Create(nBand, sRow, sCol, nRows, nCols, eType, band_map, bHasNoData, NoDataValue)) {
		printf("Error: m_pData.Create()!\n\n");
		return false;
	}

	for (int i = 0; i < nBand; i++) {
		int BandIndex = band_map[i];
		if (BandIndex <= nBandSrc) {
			GDALRasterBand* poBand;
			poBand = m_pDataset->GetRasterBand(BandIndex);
			CPLErr State = poBand->RasterIO(GF_Read, sCol, sRow, nCols, nRows, m_pData.pBuffer[i], nCols, nRows, eType, 0, 0);
			if (State == CE_Failure) {
				printf("Error: Read data error!\n\n");
				return false;
			}
		}
	}

	return true;
}

bool CRasterData::ReadGrayData(int sRow, int sCol, int nRows, int nCols)
{
	int nBand = GetBands();
	if (nBand > 3) {
		nBand = 3;
	}
	int* band_map = new int[nBand];
	for (int i = 0; i < nBand; i++) {
		band_map[i] = i + 1;
	}
	GDALDataType eType = GetDataType();

	if (!Read(nBand, sRow, sCol, nRows, nCols, eType, band_map)) {
		if (band_map) {
			delete[] band_map; band_map = nullptr;
		}
		return false;
	}

	m_pData.ConvertToGray();

	if (band_map) {
		delete[] band_map; band_map = nullptr;
	}

	return true;
}

bool CRasterData::InterpolateByPixelCoor(double px, double py, double* pValue)
{
	if (m_pData.pBuffer) {
		double local_px = px - m_pData.m_sCol;
		double local_py = py - m_pData.m_sRow;
		return m_pData.Interpolate(local_px, local_py, pValue);
	}
	else {
		printf("Read Image first!\n\n");
		return false;
	}
}

bool CRasterData::Write(void** pBuffer, int nBand, int sRow, int sCol, int nRows, int nCols, GDALDataType eType, int* band_map)
{
	if (!m_pDataset)
	{
		printf("Error: Open/Create file first!\n\n");
		return false;
	}

	int nBandSrc = m_pDataset->GetRasterCount();
	for (int i = 0; i < nBand; i++) {
		int BandIndex = band_map[i];
		if (BandIndex <= nBandSrc) {
			GDALRasterBand* poBand;
			poBand = m_pDataset->GetRasterBand(BandIndex);
			CPLErr State = poBand->RasterIO(GF_Write, sCol, sRow, nCols, nRows, pBuffer[i], nCols, nRows, eType, 0, 0);
			if (State == CE_Failure) {
				printf("Error: Write data error!\n\n");
				return false;
			}
		}
	}

	return true;
}


int CRasterData::GetRows()
{
	if (m_pDataset) {
		return m_pDataset->GetRasterYSize();
	}
	return 0;
}

int CRasterData::GetCols()
{
	if (m_pDataset) {
		return m_pDataset->GetRasterXSize();
	}
	return 0;
}

int CRasterData::GetBands()
{
	if (m_pDataset) {
		return m_pDataset->GetRasterCount();
	}
	return 0;
}

GDALDataType CRasterData::GetDataType()
{
	if (m_pDataset) {
		return GDALGetRasterDataType(m_pDataset->GetRasterBand(1));
	}
	return GDT_Unknown;
}

std::string CRasterData::GetPathName()
{
	return m_DataPath;
}

bool CRasterData::GetNoDataValue(double& NoDataValue)
{
	int bSuc = 0;
	double data = 0;

	if (m_pDataset) {
		data = m_pDataset->GetRasterBand(1)->GetNoDataValue(&bSuc);
		if (bSuc == true) {
			NoDataValue = data;
			return true;
		}
	}

	return false;
}

bool CRasterData::SetNoDataValue(double NoDataValue)
{
	if (m_pDataset) {
		int nBand = m_pDataset->GetRasterCount();
		for (int i = 0; i < nBand; i++) {
			m_pDataset->GetRasterBand(i + 1)->SetNoDataValue(NoDataValue);
		}
	}
	return true;
}

bool CRasterData::GetNoDataValue(bool* bHasNoDataValue, double* NoDataValue)
{
	if (m_pDataset) {
		int nBand = m_pDataset->GetRasterCount();

		for (int i = 0; i < nBand; i++) {
			NoDataValue[i] = 0;
			bHasNoDataValue[i] = false;

			int bSuc = 0;
			double data = 0;
			data = m_pDataset->GetRasterBand(i + 1)->GetNoDataValue(&bSuc);
			if (bSuc == true) {
				NoDataValue[i] = data;
				bHasNoDataValue[i] = true;
			}
		}

	}

	return true;
}

bool CRasterData::SetNoDataValue(double* NoDataValue)
{
	if (m_pDataset) {
		int nBand = m_pDataset->GetRasterCount();
		for (int i = 0; i < nBand; i++) {
			m_pDataset->GetRasterBand(i + 1)->SetNoDataValue(NoDataValue[i]);
		}
	}
	return true;
}

/////////////////////////////////////////

CDEM::CDEM()
{

}

CDEM::~CDEM()
{
	if (m_pCoorCvtToWGS84) {
		delete m_pCoorCvtToWGS84; m_pCoorCvtToWGS84 = nullptr;
	}
	if (m_pCoorCvtToCXYZ) {
		delete m_pCoorCvtToCXYZ; m_pCoorCvtToCXYZ = nullptr;
	}
}

bool CDEM::Open(const char* lpstrPathName, const char* proj_share)
{
	if (CRasterData::Open(lpstrPathName)) {

		if (proj_share == nullptr) {
			char exe[256] = { 0 }, drive[256] = { 0 }, path[256] = { 0 };
			::GetModuleFileNameA(NULL, exe, 256);
			_splitpath(exe, drive, path, NULL, NULL);
			std::string dbPath = std::string(drive) + std::string(path) + "proj-share";
			if (_access(dbPath.c_str(), 0) != 0) {
				return false;
			}

			return Init(dbPath.c_str());
		}
		else {
			return Init(proj_share);
		}
	}
	return false;
}

bool CDEM::Init(const char* proj_share)
{
	GetGeoTransform(m_GeoMatrix);

	const char* pszSRS_WKT = GetProjectionRef();
	m_pCoorCvtToWGS84 = new CCoordinateSystemConvert(proj_share);
	m_pCoorCvtToWGS84->SetInputCoordByWKT(pszSRS_WKT);
	m_pCoorCvtToWGS84->SetOutputCoordByEPSG(4326);

	m_pCoorCvtToCXYZ = new CCoordinateSystemConvert(proj_share);
	m_pCoorCvtToCXYZ->SetInputCoordByWKT(pszSRS_WKT);
	m_pCoorCvtToCXYZ->SetOutputCoordByEPSG(4978); //wgs84 geocentric

	int nCols = GetCols();
	int nRows = GetRows();
	GetGeoRangeInWGS84(0, 0, nRows, nCols, m_GeoRange.Lon, m_GeoRange.Lat);
	m_GeoRange.CenterLon = (m_GeoRange.Lon[0] + m_GeoRange.Lon[1] + m_GeoRange.Lon[2] + m_GeoRange.Lon[3]) / 4.0;
	m_GeoRange.CenterLat = (m_GeoRange.Lat[0] + m_GeoRange.Lat[1] + m_GeoRange.Lat[2] + m_GeoRange.Lat[3]) / 4.0;

	return true;
}

bool CDEM::PixelCoorToOriginalXYZ(double px, double py, double& X, double& Y, double& Z)
{
	double East = 0, North = 0, Height = 0;
	East = m_GeoMatrix[0] + px * m_GeoMatrix[1] + py * m_GeoMatrix[2];
	North = m_GeoMatrix[3] + px * m_GeoMatrix[4] + py * m_GeoMatrix[5];
	if (InterpolateByPixelCoor(px, py, &Height)) {
		X = East;
		Y = North;
		Z = Height;
		return true;
	}

	return false;
}

bool CDEM::PixelCoorToWGS84LBH(double px, double py, double& Lon, double& Lat, double& Height)
{
	double East = 0, North = 0, Z = 0;
	East = m_GeoMatrix[0] + px * m_GeoMatrix[1] + py * m_GeoMatrix[2];
	North = m_GeoMatrix[3] + px * m_GeoMatrix[4] + py * m_GeoMatrix[5];
	Lon = East;
	Lat = North;
	//if (InterpolateByPixelCoor(px, py, &Z)) {
		if (m_pCoorCvtToWGS84->Transform(1, &East, &North, &Z)) {
			Lon = East;
			Lat = North;
			Height = Z;
			return true;
		}
	//}
	return false;
	//return true;
}

bool CDEM::PixelCoorToWGS84CXYZ(double px, double py, double& CX, double& CY, double& CZ)
{
	double East = 0, North = 0, Z = 0;
	East = m_GeoMatrix[0] + px * m_GeoMatrix[1] + py * m_GeoMatrix[2];
	North = m_GeoMatrix[3] + px * m_GeoMatrix[4] + py * m_GeoMatrix[5];
	if (InterpolateByPixelCoor(px, py, &Z)) {
		if (m_pCoorCvtToCXYZ->Transform(1, &East, &North, &Z)) {
			CX = East;
			CY = North;
			CZ = Z;
			return true;
		}
	}
	px_t = East;
	py_t = North;
	return false;
}

bool CDEM::OriginalXYZToPixelCoor(double X, double Y, double Z, double& px, double& py)
{
	//assume m_GeoMatrix[2] = 0, m_GeoMatrix[4] =0
	px = (X - m_GeoMatrix[0]) / m_GeoMatrix[1];
	py = (Y - m_GeoMatrix[3]) / m_GeoMatrix[5];
	return true;
}

bool CDEM::WGS84LBHToPixelCoor(double Lon, double Lat, double Height, double& px, double& py)
{
	//assume m_GeoMatrix[2] = 0, m_GeoMatrix[4] =0
	if (m_pCoorCvtToWGS84->InverseTransform(1, &Lon, &Lat, &Height)) {
		px = (Lon - m_GeoMatrix[0]) / m_GeoMatrix[1];
		py = (Lat - m_GeoMatrix[3]) / m_GeoMatrix[5];
		return true;
	}
	return false;
}

bool CDEM::WGS84CXYZToPixelCoor(double CX, double CY, double CZ, double& px, double& py)
{
	//assume m_GeoMatrix[2] = 0, m_GeoMatrix[4] =0
	if (m_pCoorCvtToCXYZ->InverseTransform(1, &CX, &CY, &CZ)) {
		px = (CX - m_GeoMatrix[0]) / m_GeoMatrix[1];
		py = (CY - m_GeoMatrix[3]) / m_GeoMatrix[5];
		return true;
	}

	return false;
}

bool CDEM::InterpolateByWGS84(double Lon, double Lat, double* pValue)
{
	double Height = 0;
	double px = 0, py = 0;
	if (m_pCoorCvtToWGS84->InverseTransform(1, &Lon, &Lat, &Height)) {
		px = (Lon - m_GeoMatrix[0]) / m_GeoMatrix[1];
		py = (Lat - m_GeoMatrix[3]) / m_GeoMatrix[5];

		if (InterpolateByPixelCoor(px, py, pValue)) {
			return true;
		}
	}

	return false;
}

bool CDEM::InterpolateByPixelCoor(double px, double py, double* pValue)
{
	if (m_pData.pBuffer) {
		double local_px = px - m_pData.m_sCol;
		double local_py = py - m_pData.m_sRow;
		return m_pData.Interpolate(local_px, local_py, pValue);
	}
	else {
		printf("Read DEM first!\n\n");
		return false;
	}
}

bool CDEM::GetGeoTransform(double* pGeoMatrix6)
{
	if (m_pDataset) {
		if (CE_None == m_pDataset->GetGeoTransform(pGeoMatrix6)) {
			return true;
		}
	}
	return false;
}

bool CDEM::SetGeoTransform(double* pGeoMatrix6)
{
	if (m_pDataset) {
		if (CE_None == m_pDataset->SetGeoTransform(pGeoMatrix6)) {
			return true;
		}
	}
	return false;
}

const char* CDEM::GetProjectionRef()
{
	if (m_pDataset) {
		return m_pDataset->GetProjectionRef();
	}
	return nullptr;
}

bool CDEM::SetProjection(const char* projection)
{
	if (m_pDataset) {
		if (CE_None == m_pDataset->SetProjection(projection)) {
			return true;
		}
	}
	return false;
}

bool CDEM::SetSpatialRef(OGRSpatialReference* poSRS)
{
	if (m_pDataset) {
		if (CE_None == m_pDataset->SetSpatialRef(poSRS)) {
			return true;
		}
	}
	return false;
}

bool CDEM::GetGeoRangeInWGS84(int sRow, int sCol, int nRows, int nCols, double X[4], double Y[4])
{
	X[0] = m_GeoMatrix[0] + sCol * m_GeoMatrix[1];
	Y[0] = m_GeoMatrix[3] + sRow * m_GeoMatrix[5];
	X[1] = X[0] + m_GeoMatrix[1] * nCols;
	Y[1] = Y[0];
	X[2] = X[1];
	Y[2] = Y[1] + m_GeoMatrix[5] * nRows;
	X[3] = X[0];
	Y[3] = Y[2];

	double Z[4] = { 0 };
	if (m_pCoorCvtToWGS84->Transform(4, X, Y, Z)) {
		return true;
	}

	return false;
}

bool CDEM::GetGeoRangeInOriginalXYZ(int sRow, int sCol, int nRows, int nCols, double X[4], double Y[4])
{
	X[0] = m_GeoMatrix[0] + sCol * m_GeoMatrix[1];
	Y[0] = m_GeoMatrix[3] + sRow * m_GeoMatrix[5];
	X[1] = X[0] + m_GeoMatrix[1] * nCols;
	Y[1] = Y[0];
	X[2] = X[1];
	Y[2] = Y[1] + m_GeoMatrix[5] * nRows;
	X[3] = X[0];
	Y[3] = Y[2];

	return true;
}

double CDEM::GetXGsd()
{
	return m_GeoMatrix[1];
}

double CDEM::GetYGsd()
{
	return -m_GeoMatrix[5];
}

////////////////////////////////////////////////////////

CDOM::CDOM()
{

}

CDOM::~CDOM()
{
	if (m_pCoorCvtToWGS84) {
		delete m_pCoorCvtToWGS84; m_pCoorCvtToWGS84 = nullptr;
	}
}

bool CDOM::Open(const char* lpstrPathName, const char* proj_share)
{
	if (CRasterData::Open(lpstrPathName)) {

		if (proj_share == nullptr) {
			char exe[256] = { 0 }, drive[256] = { 0 }, path[256] = { 0 };
			::GetModuleFileNameA(NULL, exe, 256);
			_splitpath(exe, drive, path, NULL, NULL);
			std::string dbPath = std::string(drive) + std::string(path) + "proj-share";
			if (_access(dbPath.c_str(), 0) != 0) {
				return false;
			}

			return Init(dbPath.c_str());
		}
		else {
			return Init(proj_share);
		}
	}
	return false;
}

bool CDOM::Init(const char* proj_share)
{
	GetGeoTransform(m_GeoMatrix);

	const char* pszSRS_WKT = GetProjectionRef();
	m_pCoorCvtToWGS84 = new CCoordinateSystemConvert(proj_share);
	m_pCoorCvtToWGS84->SetInputCoordByWKT(pszSRS_WKT);
	m_pCoorCvtToWGS84->SetOutputCoordByEPSG(4326);

	int nCols = GetCols();
	int nRows = GetRows();
	GetGeoRangeInWGS84(0, 0, nRows, nCols, m_GeoRange.Lon, m_GeoRange.Lat);
	m_GeoRange.CenterLon = (m_GeoRange.Lon[0] + m_GeoRange.Lon[1] + m_GeoRange.Lon[2] + m_GeoRange.Lon[3]) / 4.0;
	m_GeoRange.CenterLat = (m_GeoRange.Lat[0] + m_GeoRange.Lat[1] + m_GeoRange.Lat[2] + m_GeoRange.Lat[3]) / 4.0;

	return true;
}

bool CDOM::PixelCoorToOriginalXYZ(double px, double py, double& X, double& Y, double& Z)
{
	double East = 0, North = 0;
	East = m_GeoMatrix[0] + px * m_GeoMatrix[1] + py * m_GeoMatrix[2];
	North = m_GeoMatrix[3] + px * m_GeoMatrix[4] + py * m_GeoMatrix[5];

	X = East;
	Y = North;
	Z = 0;
	return true;
}

bool CDOM::PixelCoorToWGS84LBH(double px, double py, double& Lon, double& Lat, double& Height)
{
	double East = 0, North = 0, Z = 0;
	East = m_GeoMatrix[0] + px * m_GeoMatrix[1] + py * m_GeoMatrix[2];
	North = m_GeoMatrix[3] + px * m_GeoMatrix[4] + py * m_GeoMatrix[5];
	if (m_pCoorCvtToWGS84->Transform(1, &East, &North, &Z)) {
		Lon = East;
		Lat = North;
		Height = Z;
		return true;
	}

	return false;
}

bool CDOM::OriginalXYZToPixelCoor(double X, double Y, double Z, double& px, double& py)
{
	//assume m_GeoMatrix[2] = 0, m_GeoMatrix[4] =0
	px = (X - m_GeoMatrix[0]) / m_GeoMatrix[1];
	py = (Y - m_GeoMatrix[3]) / m_GeoMatrix[5];
	return true;

}

bool CDOM::WGS84LBHToPixelCoor(double Lon, double Lat, double Height, double& px, double& py)
{
	//assume m_GeoMatrix[2] = 0, m_GeoMatrix[4] =0
	if (m_pCoorCvtToWGS84->InverseTransform(1, &Lon, &Lat, &Height)) {
		px = (Lon - m_GeoMatrix[0]) / m_GeoMatrix[1];
		py = (Lat - m_GeoMatrix[3]) / m_GeoMatrix[5];
		return true;
	}

	return false;
}

bool CDOM::InterpolateByWGS84(double Lon, double Lat, double* pValue)
{
	double Height = 0;
	double px = 0, py = 0;
	if (m_pCoorCvtToWGS84->InverseTransform(1, &Lon, &Lat, &Height)) {
		px = (Lon - m_GeoMatrix[0]) / m_GeoMatrix[1];
		py = (Lat - m_GeoMatrix[3]) / m_GeoMatrix[5];

		if (InterpolateByPixelCoor(px, py, pValue)) {
			return true;
		}
	}

	return false;
}

bool CDOM::InterpolateByPixelCoor(double px, double py, double* pValue)
{
	if (m_pData.pBuffer) {
		double local_px = px - m_pData.m_sCol;
		double local_py = py - m_pData.m_sRow;
		return m_pData.Interpolate(local_px, local_py, pValue);
	}
	else {
		printf("Read DOM first!\n\n");
		return false;
	}
}

bool CDOM::GetGeoTransform(double* pGeoMatrix6)
{
	if (m_pDataset) {
		if (CE_None == m_pDataset->GetGeoTransform(pGeoMatrix6)) {
			return true;
		}
	}
	return false;
}

bool CDOM::SetGeoTransform(double* pGeoMatrix6)
{
	if (m_pDataset) {
		if (CE_None == m_pDataset->SetGeoTransform(pGeoMatrix6)) {
			return true;
		}
	}
	return false;
}

const char* CDOM::GetProjectionRef()
{
	if (m_pDataset) {
		return m_pDataset->GetProjectionRef();
	}
	return nullptr;
}

bool CDOM::SetProjection(const char* projection)
{
	if (m_pDataset) {
		if (CE_None == m_pDataset->SetProjection(projection)) {
			return true;
		}
	}
	return false;
}

bool CDOM::SetSpatialRef(OGRSpatialReference* poSRS)
{
	if (m_pDataset) {
		if (CE_None == m_pDataset->SetSpatialRef(poSRS)) {
			return true;
		}
	}
	return false;
}

bool CDOM::GetGeoRangeInWGS84(int sRow, int sCol, int nRows, int nCols, double X[4], double Y[4])
{
	X[0] = m_GeoMatrix[0] + sCol * m_GeoMatrix[1];
	Y[0] = m_GeoMatrix[3] + sRow * m_GeoMatrix[5];
	X[1] = X[0] + m_GeoMatrix[1] * nCols;
	Y[1] = Y[0];
	X[2] = X[1];
	Y[2] = Y[1] + m_GeoMatrix[5] * nRows;
	X[3] = X[0];
	Y[3] = Y[2];

	double Z[4] = { 0 };
	if (m_pCoorCvtToWGS84->Transform(4, X, Y, Z)) {
		return true;
	}

	return false;
}

bool CDOM::GetGeoRangeInOriginalXYZ(int sRow, int sCol, int nRows, int nCols, double X[4], double Y[4])
{
	X[0] = m_GeoMatrix[0] + sCol * m_GeoMatrix[1];
	Y[0] = m_GeoMatrix[3] + sRow * m_GeoMatrix[5];
	X[1] = X[0] + m_GeoMatrix[1] * nCols;
	Y[1] = Y[0];
	X[2] = X[1];
	Y[2] = Y[1] + m_GeoMatrix[5] * nRows;
	X[3] = X[0];
	Y[3] = Y[2];

	return true;
}

double CDOM::GetXGsd()
{
	return m_GeoMatrix[1];
}

double CDOM::GetYGsd()
{
	return -m_GeoMatrix[5];
}

///////////////////////////////////////

//CSat::CSat()
//{
//
//}
//
//CSat::~CSat()
//{
//	//ReleaseSatSensorModel(m_pSensorModel);
//}

//bool CSat::Open(SatSensorModel ModelType, const char* lpstrPathName, const char* proj_share)
//{
//	if (!CRasterData::Open(lpstrPathName)) {
//		return false;
//	}
//
//	ReleaseSatSensorModel(m_pSensorModel);
//
//	if (proj_share == nullptr) {
//		char exe[256] = { 0 }, drive[256] = { 0 }, path[256] = { 0 };
//		::GetModuleFileNameA(NULL, exe, 256);
//		_splitpath(exe, drive, path, NULL, NULL);
//		std::string dbPath = std::string(drive) + std::string(path) + "proj-share";
//		if (_access(dbPath.c_str(), 0) != 0) {
//			return false;
//		}
//
//		m_pSensorModel = CreateSatSensorModel(ModelType, lpstrPathName, dbPath.c_str());
//		if (m_pSensorModel == nullptr) {
//			return false;
//		}
//	}
//	else {
//		m_pSensorModel = CreateSatSensorModel(ModelType, lpstrPathName, proj_share);
//		if (m_pSensorModel == nullptr) {
//			return false;
//		}
//	}
//
//	int nCols = GetCols();
//	int nRows = GetRows();
//	double gz = 0;
//	GetGeoRangeInWGS84(0, 0, nRows, nCols, gz, m_GeoRange.Lon, m_GeoRange.Lat);
//	m_GeoRange.CenterLon = (m_GeoRange.Lon[0] + m_GeoRange.Lon[1] + m_GeoRange.Lon[2] + m_GeoRange.Lon[3]) / 4.0;
//	m_GeoRange.CenterLat = (m_GeoRange.Lat[0] + m_GeoRange.Lat[1] + m_GeoRange.Lat[2] + m_GeoRange.Lat[3]) / 4.0;
//
//	return true;
//}
//
//bool CSat::SetSensorModel(CSatSensorModel* pModel)
//{
//	ReleaseSatSensorModel(m_pSensorModel);
//	m_pSensorModel = pModel;
//	return true;
//}
//
//CSatSensorModel* CSat::GetSensorModel()
//{
//	return m_pSensorModel;
//}
//
//bool CSat::InterpolateByWGS84(double gx, double gy, double gz, double* pValue)
//{
//	if (m_pSensorModel) {
//		double px = 0, py = 0;
//		if (m_pSensorModel->LBH_2_Pixel(gx, gy, gz, &px, &py)) {
//			return InterpolateByPixelCoor(px, py, pValue);
//		}
//	}
//
//	return false;
//}
//
//bool CSat::InterpolateByCXYZ(double gx, double gy, double gz, double* pValue)
//{
//	if (m_pSensorModel) {
//		double px = 0, py = 0;
//		if (m_pSensorModel->CXYZ_2_Pixel(gx, gy, gz, &px, &py)) {
//			return InterpolateByPixelCoor(px, py, pValue);
//		}
//	}
//	return false;
//}
//
//bool CSat::InterpolateByXYZ(double gx, double gy, double gz, double* pValue)
//{
//	if (m_pSensorModel) {
//		double px = 0, py = 0;
//		if (m_pSensorModel->XYZ_2_Pixel(gx, gy, gz, &px, &py)) {
//			return InterpolateByPixelCoor(px, py, pValue);
//		}
//	}
//	return false;
//}
//
//bool CSat::InterpolateByPixelCoor(double px, double py, double* pValue)
//{
//	if (m_pData.pBuffer) {
//		double local_px = px - m_pData.m_sCol;
//		double local_py = py - m_pData.m_sRow;
//		return m_pData.Interpolate(local_px, local_py, pValue);
//	}
//	else {
//		printf("Read Sat first!\n\n");
//		return false;
//	}
//}
//
//bool CSat::GetGeoRangeInWGS84(int sRow, int sCol, int nRows, int nCols, double gz, double X[4], double Y[4])
//{
//	if (m_pSensorModel == nullptr) {
//		return false;
//	}
//
//	m_pSensorModel->Pixel_2_LBH(sCol, sRow, gz, &X[0], &Y[0], &gz);
//	m_pSensorModel->Pixel_2_LBH(sCol + nCols - 1, sRow, gz, &X[1], &Y[1], &gz);
//	m_pSensorModel->Pixel_2_LBH(sCol + nCols - 1, sRow + nRows - 1, gz, &X[2], &Y[2], &gz);
//	m_pSensorModel->Pixel_2_LBH(sCol, sRow + nRows - 1, gz, &X[3], &Y[3], &gz);
//	return true;
//}
//
////////////////////////////////////////////////////////////
//
//bool MeterGsdToDegreeGsd(double GsdInMeter, double LatInDegree, double& LonGsdInDegree, double& LatGsdInDegree)
//{
//	//long axis: 6378137, short axis: 6356752
//	double PI = 3.141592653589793238;
//	double PI2 = 2 * PI;
//	double MeridianCircleLength = PI2 * 6378137.0;
//	double MeterPerDegreeForLat = MeridianCircleLength / 360.0;
//	double MeterPerDegreeForLon = MeterPerDegreeForLat * cos(LatInDegree * PI / 180.0);
//
//	LatGsdInDegree = GsdInMeter / MeterPerDegreeForLat;
//	LonGsdInDegree = GsdInMeter / MeterPerDegreeForLon;
//
//	return true;
//}
//
//bool MergeDEMToGeoTiff(char* DemListFile, char* ResultFilePath)
//{
//	char drive[256], path[256], c_exe_path[256];
//	GetModuleFileNameA(NULL, c_exe_path, 256);
//	_splitpath(c_exe_path, drive, path, NULL, NULL);
//	std::string VrtExePath = std::string(drive) + std::string(path) + "gdalbuildvrt.exe";
//	std::string TranslateExePath = std::string(drive) + std::string(path) + "gdal_translate.exe";
//
//	char drive1[256], path1[256];
//	_splitpath(ResultFilePath, drive1, path1, NULL, NULL);
//	std::string VrtFilePath = std::string(drive1) + std::string(path1) + "Merge.vrt";
//
//	std::string CmdLine1 = VrtExePath + " -input_file_list " + DemListFile + " " + VrtFilePath;
//	std::string CmdLine2 = TranslateExePath + " -of GTiff -co \"TILED = YES\" " + VrtFilePath + " " + ResultFilePath;
//
//	::system(CmdLine1.c_str());
//	::system(CmdLine2.c_str());
//
//	return true;
//}

//bool OrthoResampling(CSat* pSat, CDEM* pDem, CDOM* pDom, size_t begin_col, size_t begin_row, size_t cols, size_t rows,
//	double NoDataValue, int* BandMap, double** pData)
//{
//	CSatSensorModel* pSatModel = pSat->GetSensorModel();
//
//	// Calculate the geographic coordinate range of Orthophoto
//	double X[4] = { 0 }, Y[4] = { 0 };
//	pDom->GetGeoRangeInWGS84(begin_row, begin_col, rows, cols, X, Y);
//
//	// Reading DEM data according to geographic coordinate range
//	double MinLon = X[0], MaxLon = X[2], MinLat = Y[2], MaxLat = Y[0];
//	double px0{}, py0{}, px1{}, py1{};
//	pDem->WGS84LBHToPixelCoor(MinLon, MaxLat, 0, px0, py0);
//	pDem->WGS84LBHToPixelCoor(MaxLon, MinLat, 0, px1, py1);
//
//	px0 -= 50;
//	py0 -= 50;
//	size_t dem_cols = (size_t)(px1 - px0 + 50);
//	size_t dem_rows = (size_t)(py1 - py0 + 50);
//	GDALDataType DEMDataType = pDem->GetDataType();
//	if (!pDem->Read(1, py0, px0, dem_rows, dem_cols, DEMDataType, BandMap)) {
//		printf("Read dem failed!\n\n");
//		return false;
//	}
//
//	// Calculate the data reading range of satellite image
//	double min_x = 1.0e+20, min_y = 1.0e+20, max_x = -1.0e+20, max_y = -1.0e+20;
//
//	unsigned long long Total = (unsigned long long)cols * (unsigned long long)rows;
//	double* pts_x = new double[Total] {};
//	double* pts_y = new double[Total] {};
//	for (size_t i = 0; i < Total; i++) {
//		pts_x[i] = pts_y[i] = -1.0;
//	}
//
//	///////////////////////////////////////	
//	//slow version : pixel by pixel
//
//	//size_t n = 0;
//	//double y = Y[0];
//	//for (size_t i = 0; i < rows; i++) {
//	//	double x = X[0];
//	//	for (size_t j = 0; j < cols; j++) {
//	//		double z{};
//	//		if (pDem->InterpolateByWGS84(x, y, &z)) {
//	//			double px{}, py{};
//	//			pSat->LBH_2_Pixel(x, y, z, &px, &py);
//
//	//			pts_x[n] = px;
//	//			pts_y[n] = py;
//	//			if (px < min_x) min_x = px;
//	//			if (py < min_y) min_y = py;
//	//			if (px > max_x) max_x = px;
//	//			if (py > max_y) max_y = py;
//	//		}
//	//		n++;
//	//		x += pDom->GetXGsd();
//	//	}
//
//	//	y -= pDom->GetYGsd();
//	//}
//
//	///////////////////////////////////////////
//	//fast version : Block by Block
//
//	int nBlockSize = 8;
//	int xBlocks = cols / nBlockSize;
//	int yBlocks = rows / nBlockSize;
//	double XInterval = (nBlockSize - 1) * pDom->GetXGsd();
//	double YInterval = (nBlockSize - 1) * pDom->GetYGsd();
//
//	double Lx[4] = { 0 }, Ly[4] = { 0 }, Rx[4] = { 0 }, Ry[4] = { 0 };
//
//	double bY = Y[0], by = 0;
//	for (int yB = 0; yB < yBlocks; yB++) {
//		double bX = X[0], bx = 0;
//
//		for (int xB = 0; xB < xBlocks; xB++) {
//			double X[4] = { 0 }, Y[4] = { 0 }, Z[4] = { 0 };
//			X[0] = bX; X[1] = X[0] + XInterval; X[2] = X[1]; X[3] = X[0];
//			Y[0] = bY; Y[1] = Y[0]; Y[2] = Y[1] - YInterval; Y[3] = Y[2];
//
//			for (int i = 0; i < 4; i++) {
//				if (!pDem->InterpolateByWGS84(X[i], Y[i], &Z[i])) {
//					break;
//				}
//			}
//			for (int i = 0; i < 4; i++) {
//				if (!pSatModel->LBH_2_Pixel(X[i], Y[i], Z[i], &Rx[i], &Ry[i])) {
//					break;
//				}
//			}
//			for (int i = 0; i < 4; i++) {
//				if (Rx[i] < min_x) min_x = Rx[i];
//				if (Ry[i] < min_y) min_y = Ry[i];
//				if (Rx[i] > max_x) max_x = Rx[i];
//				if (Ry[i] > max_y) max_y = Ry[i];
//			}
//
//			Lx[0] = bx; Lx[1] = Lx[0] + nBlockSize - 1; Lx[2] = Lx[1]; Lx[3] = Lx[0];
//			Ly[0] = by; Ly[1] = Ly[0]; Ly[2] = Ly[1] + nBlockSize - 1; Ly[3] = Ly[2];
//
//			double xa[3], xb[3], a[3];
//			CAtxIndirectAdjustmentX adjustX;
//			adjustX.SetAdjustment(3, 0);
//			for (int m = 0; m < 4; m++) {
//				a[0] = 1;
//				a[1] = Lx[m];
//				a[2] = Ly[m];
//				adjustX.AddObservation(a, Rx[m]);
//			}
//			adjustX.SolveEquation(xa, false);
//
//			CAtxIndirectAdjustmentX adjustY;
//			adjustY.SetAdjustment(3, 0);
//			for (int m = 0; m < 4; m++) {
//				a[0] = 1;
//				a[1] = Lx[m];
//				a[2] = Ly[m];
//
//				adjustY.AddObservation(a, Ry[m]);
//			}
//			adjustY.SolveEquation(xb, false);
//
//			double x0 = Lx[0], y0 = Ly[0];
//			double x20 = xa[0] + xa[1] * x0 + xa[2] * y0;
//			double y20 = xb[0] + xb[1] * x0 + xb[2] * y0;
//			int StartIndex = Ly[0] * cols + Lx[0];
//			double x2, y2;
//
//			for (int m = (int)Ly[0]; m < (int)Ly[0] + nBlockSize; m++) {
//				x2 = x20;
//				y2 = y20;
//				int Index = StartIndex;
//
//				for (int n = (int)Lx[0]; n < (int)Lx[0] + nBlockSize; n++) {
//					pts_x[Index] = x2;
//					pts_y[Index] = y2;
//
//					x2 += xa[1];
//					y2 += xb[1];
//					Index++;
//				}
//				x20 += xa[2];
//				y20 += xb[2];
//				StartIndex += cols;
//			}
//
//			bX += (XInterval + pDom->GetXGsd());
//			bx += nBlockSize;
//		}
//		bY -= (YInterval + pDom->GetYGsd());
//		by += nBlockSize;
//	}
//
//	//last cols
//	{
//		int StartCol = xBlocks * nBlockSize;
//		int EndRow = yBlocks * nBlockSize;
//		double y = Y[0];
//		double bX = X[0] + StartCol * pDom->GetXGsd();
//		size_t n = StartCol;
//
//		for (size_t i = 0; i < EndRow; i++) {
//			double x = bX;
//			for (size_t j = StartCol; j < cols; j++) {
//				double z{};
//				if (pDem->InterpolateByWGS84(x, y, &z)) {
//					double px{}, py{};
//					pSatModel->LBH_2_Pixel(x, y, z, &px, &py);
//
//					pts_x[n] = px;
//					pts_y[n] = py;
//					if (px < min_x) min_x = px;
//					if (py < min_y) min_y = py;
//					if (px > max_x) max_x = px;
//					if (py > max_y) max_y = py;
//				}
//				n++;
//				x += pDom->GetXGsd();
//			}
//
//			n += StartCol;
//			y -= pDom->GetYGsd();
//		}
//	}
//
//	//last rows
//	{
//		int StartRow = yBlocks * nBlockSize;
//		double y = Y[0] - StartRow * pDom->GetYGsd();
//		size_t n = StartRow * cols;
//
//		for (size_t i = StartRow; i < rows; i++) {
//			double x = X[0];
//			for (size_t j = 0; j < cols; j++) {
//				double z{};
//				if (pDem->InterpolateByWGS84(x, y, &z)) {
//					double px{}, py{};
//					pSatModel->LBH_2_Pixel(x, y, z, &px, &py);
//
//					pts_x[n] = px;
//					pts_y[n] = py;
//					if (px < min_x) min_x = px;
//					if (py < min_y) min_y = py;
//					if (px > max_x) max_x = px;
//					if (py > max_y) max_y = py;
//				}
//				n++;
//				x += pDom->GetXGsd();
//			}
//			y -= pDom->GetYGsd();
//		}
//	}
//	///////////////////////////////////////////////////
//
//	//read satellite image
//	px0 = min_x - 50;
//	py0 = min_y - 50;
//	size_t sat_cols = (size_t)(max_x - px0 + 50);
//	size_t sat_rows = (size_t)(max_y - py0 + 50);
//	GDALDataType SatDataType = pSat->GetDataType();
//	int SatBands = pSat->GetBands();
//	if (!pSat->Read(SatBands, py0, px0, sat_rows, sat_cols, SatDataType, BandMap)) {
//		printf("Read sat failed!\n\n");
//		if (pts_x) delete[] pts_x;
//		if (pts_y) delete[] pts_y;
//		return false;
//	}
//
//	// resampling the ortho data
//	double* pValue = new double[SatBands];
//	size_t n = 0;
//	for (size_t i = 0; i < cols * rows; i++) {
//		for (int b = 0; b < SatBands; b++) {
//			pValue[b] = NoDataValue;
//		}
//
//		pSat->InterpolateByPixelCoor(pts_x[n], pts_y[n], pValue);
//		for (int b = 0; b < SatBands; b++) {
//			pData[b][n] = pValue[b];
//		}
//
//		n++;
//	}
//
//	if (pValue) delete[] pValue;
//	if (pts_x) delete[] pts_x;
//	if (pts_y) delete[] pts_y;
//
//	return true;
//}
//
//bool OrthoRectification(char* proj_share, char* SatFile, char* DemFile, char* DomFile, double GsdInMeter, double NoDataValue)
//{
//	CDEM DemIn;
//	if (!DemIn.Open(DemFile, proj_share)) {
//		printf("can't open dem file!\n\n");
//		return false;
//	}
//	CSat SatIn;
//	if (!SatIn.Open(SatSensorModel::RFM, SatFile, proj_share)) {
//		printf("can't open sat file!\n\n");
//		return false;
//	}
//
//	// 计算出正射影像地理范围，行数和列数
//	auto it_x = std::minmax_element(SatIn.m_GeoRange.Lon, SatIn.m_GeoRange.Lon + 4);
//	auto it_y = std::minmax_element(SatIn.m_GeoRange.Lat, SatIn.m_GeoRange.Lat + 4);
//
//	double LonGsdInDegree = 0, LatGsdInDegree = 0;
//	MeterGsdToDegreeGsd(GsdInMeter, SatIn.m_GeoRange.CenterLat, LonGsdInDegree, LatGsdInDegree);
//
//	double geo_trans[6] = { *it_x.first, LonGsdInDegree, 0, *it_y.second, 0, -LatGsdInDegree };
//	size_t DomCols = (size_t)((*it_x.second - *it_x.first) / LonGsdInDegree);
//	size_t DomRows = (size_t)((*it_y.second - *it_y.first) / LatGsdInDegree);
//
//	// 创建正射影像
//	int DomBands = SatIn.GetBands();
//	GDALDataType DomDataType = SatIn.GetDataType();
//	CDOM ImgOut;
//	if (!ImgOut.Create(DomCols, DomRows, DomBands, DomDataType)) {
//		printf("can't create dom file!\n\n");
//		return false;
//	}
//	ImgOut.SetNoDataValue(NoDataValue);
//	ImgOut.SetGeoTransform(geo_trans);
//	OGRSpatialReference srs;
//	srs.importFromEPSG(4326);
//	ImgOut.SetSpatialRef(&srs);
//	ImgOut.Init(proj_share);
//
//	// 分块采样正射影像
//	size_t num_of_lines = 1024;
//	size_t num_of_loops = DomRows / num_of_lines;
//	size_t num_of_lasts = DomRows % num_of_lines;
//
//	double** pData = new double* [DomBands];
//	for (int i = 0; i < DomBands; i++) {
//		unsigned long long Total = (unsigned long long)num_of_lines * (unsigned long long)DomCols;
//		pData[i] = new double[Total];
//	}
//	int* BandMap = new int[DomBands];
//	for (int i = 0; i < DomBands; i++) {
//		BandMap[i] = i + 1;
//	}
//
//	size_t begin_line = 0;
//	for (int j = 0; j < num_of_loops; j++) {
//		for (int i = 0; i < DomBands; i++) {
//			memset(pData[i], 0, sizeof(double) * num_of_lines * DomCols);
//		}
//
//		if (OrthoResampling(&SatIn, &DemIn, &ImgOut, 0, begin_line, DomCols, num_of_lines, NoDataValue, BandMap, pData)) {
//			ImgOut.Write((void**)pData, DomBands, begin_line, 0, num_of_lines, DomCols, GDT_Float64, BandMap);
//		}
//
//		begin_line += num_of_lines;
//	}
//
//	if (num_of_lasts) {
//		for (int i = 0; i < DomBands; i++) {
//			memset(pData[i], 0, sizeof(double) * num_of_lines * DomCols);
//		}
//
//		if (OrthoResampling(&SatIn, &DemIn, &ImgOut, 0, begin_line, DomCols, num_of_lasts, NoDataValue, BandMap, pData)) {
//			ImgOut.Write((void**)pData, DomBands, begin_line, 0, num_of_lasts, DomCols, GDT_Float64, BandMap);
//		}
//	}
//
//	if (pData) {
//		for (int i = 0; i < DomBands; i++) {
//			delete[] pData[i]; pData[i] = nullptr;
//		}
//		delete[] pData; pData = nullptr;
//	}
//	if (BandMap) {
//		delete[] BandMap; BandMap = nullptr;
//	}
//
//	char** papszOptions = nullptr;
//	papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "YES");
//	//papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "IF_NEEDED");
//
//	ImgOut.SaveAsGeoTiff(DomFile);
//	//ImgOut.SaveAsCOG(DomFile);
//
//	return true;
//}
//
//bool OrthoRectificationGivenPara(char* proj_share, char* SatFile, char* DemFile, char* DomFile, char* ParaFromDom)
//{
//	CDEM DemIn;
//	if (!DemIn.Open(DemFile, proj_share)) {
//		printf("can't open dem file!\n\n");
//		return false;
//	}
//	CSat SatIn;
//	if (!SatIn.Open(SatSensorModel::RFM, SatFile, proj_share)) {
//		printf("can't open sat file!\n\n");
//		return false;
//	}
//	CDOM DomIN;
//	if (!DomIN.Open(ParaFromDom, proj_share)) {
//		printf("can't open ParaFromDom file!\n\n");
//		return false;
//	}
//
//	// 创建正射影像
//	size_t DomCols = DomIN.GetCols();
//	size_t DomRows = DomIN.GetRows();
//	GDALDataType DomDataType = DomIN.GetDataType();
//	int DomBands = SatIn.GetBands();
//	CDOM ImgOut;
//	if (!ImgOut.Create(DomCols, DomRows, DomBands, DomDataType)) {
//		printf("can't create dom file!\n\n");
//		return false;
//	}
//	double geo_trans[6] = { 0 };
//	DomIN.GetGeoTransform(geo_trans);
//	ImgOut.SetGeoTransform(geo_trans);
//	const char* srs = DomIN.GetProjectionRef();
//	ImgOut.SetProjection(srs);
//	double NoDataValue = 0;
//	if (DomIN.GetNoDataValue(NoDataValue)) {
//		ImgOut.SetNoDataValue(NoDataValue);
//	}
//	ImgOut.Init(proj_share);
//
//	// 分块采样正射影像
//	size_t num_of_lines = 1024;
//	size_t num_of_loops = DomRows / num_of_lines;
//	size_t num_of_lasts = DomRows % num_of_lines;
//
//	double** pData = new double* [DomBands];
//	for (int i = 0; i < DomBands; i++) {
//		unsigned long long Total = (unsigned long long)num_of_lines * (unsigned long long)DomCols;
//		pData[i] = new double[Total];
//	}
//	int* BandMap = new int[DomBands];
//	for (int i = 0; i < DomBands; i++) {
//		BandMap[i] = i + 1;
//	}
//
//	size_t begin_line = 0;
//	for (int j = 0; j < num_of_loops; j++) {
//		for (int i = 0; i < DomBands; i++) {
//			memset(pData[i], 0, sizeof(double) * num_of_lines * DomCols);
//		}
//
//		if (OrthoResampling(&SatIn, &DemIn, &ImgOut, 0, begin_line, DomCols, num_of_lines, NoDataValue, BandMap, pData)) {
//			ImgOut.Write((void**)pData, DomBands, begin_line, 0, num_of_lines, DomCols, GDT_Float64, BandMap);
//		}
//
//		begin_line += num_of_lines;
//	}
//
//	if (num_of_lasts) {
//		for (int i = 0; i < DomBands; i++) {
//			memset(pData[i], 0, sizeof(double) * num_of_lines * DomCols);
//		}
//
//		if (OrthoResampling(&SatIn, &DemIn, &ImgOut, 0, begin_line, DomCols, num_of_lasts, NoDataValue, BandMap, pData)) {
//			ImgOut.Write((void**)pData, DomBands, begin_line, 0, num_of_lasts, DomCols, GDT_Float64, BandMap);
//		}
//	}
//
//	if (pData) {
//		for (int i = 0; i < DomBands; i++) {
//			delete[] pData[i]; pData[i] = nullptr;
//		}
//		delete[] pData; pData = nullptr;
//	}
//	if (BandMap) {
//		delete[] BandMap; BandMap = nullptr;
//	}
//
//	char** papszOptions = nullptr;
//	papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "YES");
//	//papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "IF_NEEDED");
//
//	ImgOut.SaveAsGeoTiff(DomFile);
//	//ImgOut.SaveAsCOG(DomFile);
//
//	return true;
//}

bool Pansharpen(CDOM* pDomPAN, CDOM* pDomMSS, size_t begin_col, size_t begin_row, size_t cols, size_t rows,
	double NoDataValue, int nBands, int* BandMap, double** pData)
{
	//read DomPAN image
	GDALDataType PANDataType = pDomPAN->GetDataType();
	int PANBands = 1;
	int PANBandMap[1] = { 1 };
	if (!pDomPAN->Read(PANBands, begin_row, begin_col, rows, cols, PANDataType, PANBandMap)) {
		printf("Read DomPAN failed!\n\n");
		return false;
	}

	//read DomMSS image
	GDALDataType MSSDataType = pDomMSS->GetDataType();
	if (!pDomMSS->Read(nBands, begin_row, begin_col, rows, cols, MSSDataType, BandMap)) {
		printf("Read DomMSS failed!\n\n");
		return false;
	}

	// resampling the data
	double* pValuePAN = new double[PANBands];
	double* pValueMSS = new double[nBands];
	double* pValue = new double[nBands];
	size_t n = 0;
	for (size_t i = 0; i < rows; i++) {
		for (size_t j = 0; j < cols; j++) {
			for (int b = 0; b < nBands; b++) {
				pValue[b] = NoDataValue;
			}

			if (pDomPAN->m_pData.GetData(j, i, pValuePAN)) {
				if (pDomMSS->m_pData.GetData(j, i, pValueMSS)) {

					double Ave = 0, Sum = 0;
					for (int b = 0; b < nBands; b++) {
						Sum += pValueMSS[b];
					}
					Ave = Sum / nBands;

					//paper: Best Tradeoff for High-Resolution Image Fusion to Preserve Spatial Details and Minimize Color Distortion
					//double l = 5;
					//double v1 = (l - 1) / l;
					//double v2 = 1 / l;
					//double Inew = v1 * pValuePAN[0] + v2 * Ave;
					//double Ratio = 1;
					//if (Inew > 0) {
					//	Ratio = pValuePAN[0] / Inew;
					//} 
					//for (int b = 0; b < nBands; b++) {
					//	pValue[b] = Ratio * (pValueMSS[b] + (Inew - Ave));  //	GIHS				
					//}

					for (int b = 0; b < nBands; b++) {
						//pValue[b] = pValueMSS[b] + (pValuePAN[0] - Ave);  //FIHS						

						//pValue[b] = (pValueMSS[b] / Sum) * pValuePAN[0];  //Brovey
						pValue[b] = (pValueMSS[b] / Ave) * pValuePAN[0];  //Brovey

						//pValue[b] = pValueMSS[b];  
					}

					for (int b = 0; b < nBands; b++) {
						pData[b][n] = pValue[b];
					}
				}
			}
			n++;
		}
	}

	if (pValuePAN) delete[] pValuePAN;
	if (pValueMSS) delete[] pValueMSS;
	if (pValue) delete[] pValue;

	return true;
}

bool PansharpenImagery(char* proj_share, char* PANDomFile, char* MSSDomFile, char* PansharpenDomFile)
{
	CDOM DomPAN;
	if (!DomPAN.Open(PANDomFile, proj_share)) {
		printf("can't open PANDomFile file!\n\n");
		return false;
	}
	CDOM DomMSS;
	if (!DomMSS.Open(MSSDomFile, proj_share)) {
		printf("can't open MSSDomFile file!\n\n");
		return false;
	}

	// 创建融合后的正射影像
	size_t DomCols = DomPAN.GetCols();
	size_t DomRows = DomPAN.GetRows();
	GDALDataType DomDataType = DomPAN.GetDataType();
	int DomBands = 3;  // DomMSS.GetBands();  
	CDOM ImgOut;
	if (!ImgOut.Create(DomCols, DomRows, DomBands, DomDataType)) {
		printf("can't create dom file!\n\n");
		return false;
	}
	double geo_trans[6] = { 0 };
	DomPAN.GetGeoTransform(geo_trans);
	ImgOut.SetGeoTransform(geo_trans);
	const char* srs = DomPAN.GetProjectionRef();
	ImgOut.SetProjection(srs);
	bool bHasNoData = false;
	double NoDataValue = 0;
	if (DomPAN.GetNoDataValue(NoDataValue)) {
		bHasNoData = true;
		ImgOut.SetNoDataValue(NoDataValue);
	}
	ImgOut.Init(proj_share);

	// 分块处理
	size_t num_of_lines = 1024;
	size_t num_of_loops = DomRows / num_of_lines;
	size_t num_of_lasts = DomRows % num_of_lines;

	double** pData = new double* [DomBands];
	for (int i = 0; i < DomBands; i++) {
		unsigned long long Total = (unsigned long long)num_of_lines * (unsigned long long)DomCols;
		pData[i] = new double[Total];
	}

	int* BandMap = new int[DomBands];
	for (int i = 0; i < DomBands; i++) {
		BandMap[i] = i + 1;
	}

	size_t begin_line = 0;
	for (int j = 0; j < num_of_loops; j++) {
		for (int i = 0; i < DomBands; i++) {
			memset(pData[i], 0, sizeof(double) * num_of_lines * DomCols);
		}

		if (Pansharpen(&DomPAN, &DomMSS, 0, begin_line, DomCols, num_of_lines, NoDataValue, DomBands, BandMap, pData)) {
			ImgOut.Write((void**)pData, DomBands, begin_line, 0, num_of_lines, DomCols, GDT_Float64, BandMap);
		}

		begin_line += num_of_lines;
	}

	if (num_of_lasts) {
		for (int i = 0; i < DomBands; i++) {
			memset(pData[i], 0, sizeof(double) * num_of_lines * DomCols);
		}

		if (Pansharpen(&DomPAN, &DomMSS, 0, begin_line, DomCols, num_of_lasts, NoDataValue, DomBands, BandMap, pData)) {
			ImgOut.Write((void**)pData, DomBands, begin_line, 0, num_of_lasts, DomCols, GDT_Float64, BandMap);
		}
	}

	if (pData) {
		for (int i = 0; i < DomBands; i++) {
			delete[] pData[i]; pData[i] = nullptr;
		}
		delete[] pData; pData = nullptr;
	}

	if (BandMap) {
		delete[] BandMap; BandMap = nullptr;
	}

	char** papszOptions = nullptr;
	papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "YES");
	//papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "IF_NEEDED");

	ImgOut.SaveAsGeoTiff(PansharpenDomFile);

	return true;
}

bool Convert2ByteCOG(char* proj_share, char* InputDomFile, char* OutputDomFile, bool bDeleteInputDomFile)
{
	CDOM DomTemp;
	if (!DomTemp.Open(InputDomFile, proj_share)) {
		printf("can't open TempDomFile file!\n\n");
		return false;
	}

	size_t DomCols = DomTemp.GetCols();
	size_t DomRows = DomTemp.GetRows();
	GDALDataType DomDataType = DomTemp.GetDataType();
	int DomBands = DomTemp.GetBands();
	double geo_trans[6] = { 0 };
	DomTemp.GetGeoTransform(geo_trans);
	const char* srs = DomTemp.GetProjectionRef();
	bool bHasNoData = false;
	double NoDataValue = 0;
	if (DomTemp.GetNoDataValue(NoDataValue)) {
		bHasNoData = true;
	}

	int* BandMap = new int[DomBands];
	for (int i = 0; i < DomBands; i++) {
		BandMap[i] = i + 1;
	}
	if (!DomTemp.Read(DomBands, 0, 0, DomRows, DomCols, DomDataType, BandMap)) {
		printf("can't read TempDomFile file!\n\n");
		return false;
	}

	CDOM ImgOut;
	if (!ImgOut.Create(DomCols, DomRows, DomBands, GDT_Byte)) {
		printf("can't create dom file!\n\n");
		return false;
	}
	ImgOut.SetGeoTransform(geo_trans);
	ImgOut.SetProjection(srs);
	if (bHasNoData) {
		ImgOut.SetNoDataValue(NoDataValue);
	}

	if (DomDataType != GDT_Byte) {
		unsigned char** pDataAll = new unsigned char* [DomBands];
		for (int i = 0; i < DomBands; i++) {
			unsigned long long Total = (unsigned long long)DomRows * (unsigned long long)DomCols;
			pDataAll[i] = new unsigned char[Total];
		}
		DomTemp.m_pData.LinearStretchingToByte(pDataAll);
		ImgOut.Write((void**)pDataAll, DomBands, 0, 0, DomRows, DomCols, GDT_Byte, BandMap);

		if (pDataAll) {
			for (int i = 0; i < DomBands; i++) {
				delete[] pDataAll[i]; pDataAll[i] = nullptr;
			}
			delete[] pDataAll; pDataAll = nullptr;
		}
	}
	else {
		ImgOut.Write((void**)DomTemp.m_pData.pBuffer, DomBands, 0, 0, DomRows, DomCols, GDT_Byte, BandMap);
	}

	ImgOut.SaveAsCOG(OutputDomFile);
	ImgOut.Close();

	DomTemp.Close();
	if (bDeleteInputDomFile) {
		::remove(InputDomFile);
	}

	if (BandMap) {
		delete[] BandMap; BandMap = nullptr;
	}

	return true;
}

//bool Convert2ByteTIF(char* InputImgFile, char* OutputImgFile, bool bDeleteInputImgFile)
//{
//	CRasterData SatIn;
//	if (!SatIn.Open(InputImgFile)) {
//		return 0;
//	}
//
//	int nColsIn = SatIn.GetCols();
//	int nRowsIn = SatIn.GetRows();
//	int nBandIn = SatIn.GetBands();
//	GDALDataType DataTypeIn = SatIn.GetDataType();
//
//	int* BandMap = new int[nBandIn];
//	for (int i = 0; i < nBandIn; i++) {
//		BandMap[i] = i + 1;
//	}
//
//	if (true == SatIn.Read(nBandIn, 0, 0, nRowsIn, nColsIn, DataTypeIn, BandMap)) {
//		unsigned char** pDataAll = new unsigned char* [nBandIn];
//		for (int i = 0; i < nBandIn; i++) {
//			unsigned long long Total = (unsigned long long)nRowsIn * (unsigned long long)nColsIn;
//			pDataAll[i] = new unsigned char[Total];
//		}
//		SatIn.m_pData.LinearStretchingToByte(pDataAll);
//
//		CSat ImgOut;
//		if (true == ImgOut.Create(nColsIn, nRowsIn, nBandIn, GDT_Byte)) {
//			ImgOut.Write((void**)(pDataAll), nBandIn, 0, 0, nRowsIn, nColsIn, GDT_Byte, BandMap);
//
//			char** papszOptions = nullptr;
//			//papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "YES");
//			papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "IF_NEEDED");
//
//			ImgOut.SaveAsGeoTiff(OutputImgFile);
//			ImgOut.Close();
//		}
//
//		if (pDataAll) {
//			for (int i = 0; i < nBandIn; i++) {
//				delete[] pDataAll[i]; pDataAll[i] = nullptr;
//			}
//			delete[] pDataAll; pDataAll = nullptr;
//		}
//	}
//	SatIn.Close();
//
//	if (BandMap) {
//		delete[] BandMap; BandMap = nullptr;
//	}
//
//	if (bDeleteInputImgFile) {
//		::remove(InputImgFile);
//	}
//
//	return 1;
//}

//RASTERDATA_API bool Convert2ThreeBandsTIF(char* InputImgFile, char* OutputImgFile, bool bDeleteInputImgFile)
//{
//	CRasterData SatIn;
//	if (!SatIn.Open(InputImgFile)) {
//		return 0;
//	}
//
//	int nColsIn = SatIn.GetCols();
//	int nRowsIn = SatIn.GetRows();
//	int nBandIn = 3; // SatIn.GetBands();
//	GDALDataType DataTypeIn = SatIn.GetDataType();
//
//	int* BandMap = new int[nBandIn];
//	for (int i = 0; i < nBandIn; i++) {
//		BandMap[i] = i + 1;
//	}
//
//	if (true == SatIn.Read(nBandIn, 0, 0, nRowsIn, nColsIn, DataTypeIn, BandMap)) {
//		//unsigned char** pDataAll = new unsigned char* [nBandIn];
//		//for (int i = 0; i < nBandIn; i++) {
//		//	unsigned long Total = nRowsIn * nColsIn;
//		//	pDataAll[i] = new unsigned char[Total];
//		//}
//		//SatIn.m_pData.LinearStretchingToByte(pDataAll);
//
//		CSat ImgOut;
//		if (true == ImgOut.Create(nColsIn, nRowsIn, nBandIn, DataTypeIn)) {
//			ImgOut.Write((void**)(SatIn.m_pData.pBuffer), nBandIn, 0, 0, nRowsIn, nColsIn, DataTypeIn, BandMap);
//
//			char** papszOptions = nullptr;
//			//papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "YES");
//			papszOptions = CSLSetNameValue(papszOptions, "BIGTIFF", "IF_NEEDED");
//
//			ImgOut.SaveAsGeoTiff(OutputImgFile);
//			ImgOut.Close();
//		}
//
//		//if (pDataAll) {
//		//	for (int i = 0; i < nBandIn; i++) {
//		//		delete[] pDataAll[i]; pDataAll[i] = nullptr;
//		//	}
//		//	delete[] pDataAll; pDataAll = nullptr;
//		//}
//	}
//	SatIn.Close();
//
//	if (BandMap) {
//		delete[] BandMap; BandMap = nullptr;
//	}
//
//	if (bDeleteInputImgFile) {
//		::remove(InputImgFile);
//	}
//
//	return 1;
//}
//
//RASTERDATA_API bool SatellitePhoto_2_HugeDEM(CSatSensorModel* pSatModel, CDEM* pDem, double sample, double line, double* Lon84, double* Lat84, double* Alt84)
//{
//	if (!pSatModel || !pDem) {
//		return false;
//	}
//
//	int BandMap[1] = { 1 };
//	GDALDataType DEMDataType = pDem->GetDataType();
//	double dLon = pDem->GetXGsd();
//	double dLat = pDem->GetYGsd();
//
//	*Lon84 = 0;
//	*Lat84 = 0;
//	*Alt84 = 0;
//
//	double lon = 0, lat = 0, alt0 = 0, alt = 0;
//	int iter = 0;
//	do {
//		if (!pSatModel->Pixel_2_LBH(sample, line, alt0, &lon, &lat, &alt0)) {
//			return false;
//		}
//
//		// Reading DEM data according to geographic coordinate range
//		double MinLon = lon - 10 * dLon, MaxLon = lon + 10 * dLon, MinLat = lat - 10 * dLat, MaxLat = lat + 10 * dLat;
//		double px0{}, py0{}, px1{}, py1{};
//		pDem->WGS84LBHToPixelCoor(MinLon, MaxLat, 0, px0, py0);
//		pDem->WGS84LBHToPixelCoor(MaxLon, MinLat, 0, px1, py1);
//
//		px0 -= 50;
//		py0 -= 50;
//		size_t dem_cols = (size_t)(px1 - px0 + 50);
//		size_t dem_rows = (size_t)(py1 - py0 + 50);
//		if (!pDem->Read(1, py0, px0, dem_rows, dem_cols, DEMDataType, BandMap)) {
//			printf("Read dem failed!\n\n");
//			return false;
//		}
//
//		if (!pDem->InterpolateByWGS84(lon, lat, &alt)) {
//			return false;
//		}
//
//		if (fabs(alt0 - alt) < 1.0e-02)
//			break;
//
//		alt0 = alt;
//		iter++;
//	} while (iter < 20);
//
//	*Lon84 = lon;
//	*Lat84 = lat;
//	*Alt84 = alt;
//
//	return (iter < 20) ? true : false;
//}


