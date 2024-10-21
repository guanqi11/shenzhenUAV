#include "DroneNest.h"
#include <iostream>
#include <iomanip> 
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <codecvt>
#include <cstring>
#include <unordered_set>
#include <chrono>
#include <hpdf.h>

DroneNest::DroneNest()
{
    //std::string path = "D:\\shenzhen\\Depends\\bin\\Debug\\proj-share";//proj.db�����ļ���
    //const char* proj_path[] = { path.c_str(), nullptr };
    //OSRSetPROJSearchPaths(proj_path);
    const char* proj = "D:\\shenzhen\\Depends\\data\\proj-share";
    pcvt = new CCoordinateSystemConvert(proj);
}

DroneNest::~DroneNest(){}

std::string FeatureMatched(OGRFeature* feature, std::string field_name)
{
	std::string filed_string = feature->GetFieldAsString(field_name.c_str());
	return filed_string;
}

void printStringBytes(const std::string& str) {
	std::cout << "String bytes: ";
	for (unsigned char c : str) {
		std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)c << ' ';
	}
	std::cout << std::dec << std::endl;
}

std::string GbkToUtf8(const char* src_str)
{
    int len = MultiByteToWideChar(CP_ACP, 0, src_str, -1, NULL, 0);
    wchar_t* wstr = new wchar_t[len + 1];
    memset(wstr, 0, len + 1);
    MultiByteToWideChar(CP_ACP, 0, src_str, -1, wstr, len);
    len = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, NULL, 0, NULL, NULL);
    char* str = new char[len + 1];
    memset(str, 0, len + 1);
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, str, len, NULL, NULL);
    std::string strTemp = str;
    if (wstr) delete[] wstr;
    if (str) delete[] str;
    return strTemp;
}

std::string Utf8ToGbk(const std::string& src_str)
{
    // Convert UTF-8 to wide character string
    int wlen = MultiByteToWideChar(CP_UTF8, 0, src_str.c_str(), -1, NULL, 0);
    if (wlen == 0) {
        return "";
    }
    std::wstring wstr(wlen - 1, 0); // -1 to exclude the null terminator
    MultiByteToWideChar(CP_UTF8, 0, src_str.c_str(), -1, &wstr[0], wlen);

    // Convert wide character string to GBK
    int blen = WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), -1, NULL, 0, NULL, NULL);
    if (blen == 0) {
        return "";
    }
    std::string str(blen - 1, 0); // -1 to exclude the null terminator
    WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), -1, &str[0], blen, NULL, NULL);

    return str;
}

std::wstring Utf8ToWstring(const std::string& utf8Str) {
    // ��ȡת������Ŀ��ַ�����
    int wideLen = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, NULL, 0);
    if (wideLen == 0) {
        return std::wstring(); // ת��ʧ�ܣ����ؿ��ַ���
    }

    // ������ַ��ַ������ڴ�
    std::wstring wideStr(wideLen - 1, 0); // -1 ȥ�� null ��ֹ��
    MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, &wideStr[0], wideLen);

    return wideStr;
}

bool DroneNest::openBoundaryFile(std::string boundaryFile) {
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
    CPLSetConfigOption("SHAPE_ENCODING", "");
    GDALDataset* shpDataset = (GDALDataset*)GDALOpenEx(boundaryFile.c_str(), GDAL_OF_UPDATE, nullptr, nullptr, nullptr);
    if (shpDataset == nullptr) {
        GDALClose(shpDataset);
        return false;
    }

    int input(4547), output(4326);
    pcvt->SetInputCoordByEPSG(input);
    pcvt->SetOutputCoordByEPSG(output);

    shpDataset->GetGeoTransform(_adf_geo_transform);  // ����������Ϣ
    OGRLayer* poLayer = (OGRLayer*)shpDataset->GetLayer(0);
    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // �ҵ� ID �ֶε�����
    for (int i = 0; i < featureDefn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = featureDefn->GetFieldDefn(i);
        if (std::string(fieldDefn->GetNameRef()) == "Id") {
            idFieldIndex = i;
            break;
        }
    }

    if (idFieldIndex == -1) {
        return false;
    }

    int idFieldIndex1 = featureDefn->GetFieldIndex("Id");

    if (idFieldIndex1 == -1) {
        GDALClose(shpDataset);
        return false;
    }

    OGRFeature* poFeature;
    int newID = 0;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {

        poFeature->SetField(idFieldIndex, newID);

        if (poLayer->SetFeature(poFeature) != OGRERR_NONE) {
            std::cerr << "Failed to update feature ID" << std::endl;
        }
        newID++;

        int id = poFeature->GetFieldAsInteger(idFieldIndex1);

        OGRGeometry* poGeometry = poFeature->GetGeometryRef();
        if (poGeometry != nullptr) {
            OGRwkbGeometryType geomType = poGeometry->getGeometryType();
            BuildingInfo info;

            if (geomType == wkbLineString) {
                OGRLineString* poLine = (OGRLineString*)poGeometry;
                int numPoints = poLine->getNumPoints();
                std::vector<Point2D> points;
                for (int i = 0; i < numPoints; ++i) {
                    Point2D pt;
                    pt._x = poLine->getX(i);
                    pt._y = poLine->getY(i);
                    double z = 0;
                    if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                        return 1;
                    }
                    points.push_back(pt);
                }
                info.points = points;
            }
            else if (geomType == wkbMultiLineString) {
                OGRMultiLineString* poMultiLine = (OGRMultiLineString*)poGeometry;
                int numGeometries = poMultiLine->getNumGeometries();
                for (int i = 0; i < numGeometries; ++i) {
                    OGRLineString* poLine = (OGRLineString*)poMultiLine->getGeometryRef(i);
                    int numPoints = poLine->getNumPoints();
                    std::vector<Point2D> points;
                    for (int j = 0; j < numPoints; ++j) {
                        Point2D pt;
                        pt._x = poLine->getX(j);
                        pt._y = poLine->getY(j);
                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    info.points = points;
                }
            }
            boundaryMap[id] = info;
        }
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(shpDataset);

    for (std::map<int, BuildingInfo>::const_iterator it = boundaryMap.begin(); it != boundaryMap.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        boundaryScale[id] = info;
    }
    calculateBuildingPixels(boundaryScale);
    return true;
}

void DroneNest::calculateBuildingPixels(const std::map<int, BuildingInfo>& boundaryScale) {
    for (const auto& entry : boundaryScale) {
        int buildingId = entry.first;
        const BuildingInfo& buildingInfo = entry.second;

        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::min();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::min();

        // �������εı߽�
        for (const auto& vertex : buildingInfo.points) {
            minX = custom_min(minX, vertex._x);
            maxX = custom_max(maxX, vertex._x);
            minY = custom_min(minY, vertex._y);
            maxY = custom_max(maxY, vertex._y);
        }

        int pixelCount = 0;

        // �����߽��ڵ���������
        for (int x = static_cast<int>(minX); x <= static_cast<int>(maxX); ++x) {
            for (int y = static_cast<int>(minY); y <= static_cast<int>(maxY); ++y) {
                if (isPointInPolygon({ static_cast<double>(x), static_cast<double>(y) }, buildingInfo)) {
                    ++pixelCount;
                }
            }
        }
        boundaryPixelCount = pixelCount;
    }
}

bool DroneNest::openShpFile(std::string buildFile) {
    //std::string path = "C:\\Program Files\\PROJ\\share\\proj";//proj.db�����ļ���
    const char* proj_path[] = { projPath.c_str(), nullptr };
    OSRSetPROJSearchPaths(proj_path);

    //const char* path[] = { projPath.c_str() };
    //pj_set_searchpath(1, path);

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
	CPLSetConfigOption("SHAPE_ENCODING", "");
	GDALDataset* shpDataset1 = (GDALDataset*)GDALOpenEx(buildFile.c_str(), GDAL_OF_UPDATE, nullptr, nullptr, nullptr);
	if (shpDataset1 == nullptr) {
		GDALClose(shpDataset1);
		return false;
	}

    int input(4547), output(4326);
    pcvt->SetInputCoordByEPSG(input);
    pcvt->SetOutputCoordByEPSG(output);

	_geo_projection = shpDataset1->GetProjectionRef(); // ͶӰ��Ϣ
	shpDataset1->GetGeoTransform(_adf_geo_transform);  // ����������Ϣ
	//_eType = GDALGetRasterDataType(shpDataset1->GetRasterBand(1));
	OGRLayer* poLayer = (OGRLayer*)shpDataset1->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // �ҵ� ID �ֶε�����
    for (int i = 0; i < featureDefn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = featureDefn->GetFieldDefn(i);
        if (std::string(fieldDefn->GetNameRef()) == "Id") {
            idFieldIndex = i;
            break;
        }
    }

    if (idFieldIndex == -1) {
        //std::cerr << "ID field not found!" << std::endl;
        return false;
    }

    /*std::string type = "type";
    std::string height = "height";*/
    std::string type = GbkToUtf8("����");
    std::string height = GbkToUtf8("¥��");

    int idFieldIndex1 = featureDefn->GetFieldIndex("Id");
    int typeFieldIndex1 = featureDefn->GetFieldIndex(type.c_str());
    int heightFieldIndex1 = featureDefn->GetFieldIndex(height.c_str());

    if (idFieldIndex1 == -1 || typeFieldIndex1 == -1 || heightFieldIndex1 == -1) {
        //std::cerr << "One or more fields not found!" << std::endl;
        GDALClose(shpDataset1);
        return false;
    }

    OGRFeature* poFeature;
    int newID = 0;
    //int newID1 = 0;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {

        poFeature->SetField(idFieldIndex, newID);

        if (poLayer->SetFeature(poFeature) != OGRERR_NONE) {
            std::cerr << "Failed to update feature ID" << std::endl;
        }
        newID++;
        //newID1++;
        int id = poFeature->GetFieldAsInteger(idFieldIndex1);
        std::wstring type = Utf8ToWstring(poFeature->GetFieldAsString(typeFieldIndex1));
        std::wstring result;
        size_t charCount = 0;
        for (size_t i = 0; i < type.size(); ++i) {
            if (type[i] >= 0x4E00 && type[i] <= 0x9FFF) { //�����ַ���Χ
                ++charCount;
                result += type[i];
                if (charCount >= 2) {
                    break;
                }
            } 
        }
        double height = poFeature->GetFieldAsDouble(heightFieldIndex1);
        //std::wstring wideType = Utf8ToWstring(type);
        std::wstring wideString = L"���";
        std::wstring wideString_t = L"����";
        if (result == wideString) {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            if (poGeometry != nullptr && poGeometry->getGeometryType() == wkbPolygon) {
                BuildingInfo info;
                info.type = result;
                info.height = height;

                // �������ε�ÿ����
                OGRPolygon* poPolygon = (OGRPolygon*)poGeometry;
                OGRLinearRing* poRing = poPolygon->getExteriorRing();
                if (poRing != nullptr) {
                    int numPoints = poRing->getNumPoints();
                    std::vector<Point2D> points;
                    for (int i = 0; i < numPoints; ++i) {
                        Point2D pt;
                        pt._x = poRing->getX(i);
                        pt._y = poRing->getY(i);

                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    info.points = points;
                }
                transSubMap[id] = info;
            }
        }
        else if(result != wideString_t){
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            if (poGeometry != nullptr && poGeometry->getGeometryType() == wkbPolygon) {
                BuildingInfo info;
                info.type = type;
                info.height = height;

                // �������ε�ÿ����
                OGRPolygon* poPolygon = (OGRPolygon*)poGeometry;
                OGRLinearRing* poRing = poPolygon->getExteriorRing();
                if (poRing != nullptr) {
                    int numPoints = poRing->getNumPoints();
                    std::vector<Point2D> points;
                    for (int i = 0; i < numPoints; ++i) {
                        Point2D pt;
                        pt._x = poRing->getX(i);
                        pt._y = poRing->getY(i);
                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    //info.centroid = CalculatePolygonCentroid(points);
                    info.points = points;
                }
                buildingMap[id] = info;
            }
        }
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(shpDataset1);
    return true;
}

bool DroneNest::openShpFile_allBuild(std::string buildFile)
{
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
    CPLSetConfigOption("SHAPE_ENCODING", "");
    GDALDataset* shpDataset1 = (GDALDataset*)GDALOpenEx(buildFile.c_str(), GDAL_OF_UPDATE, nullptr, nullptr, nullptr);
    if (shpDataset1 == nullptr) {
        GDALClose(shpDataset1);
        return false;
    }

    int input(4547), output(4326);
    pcvt->SetInputCoordByEPSG(input);
    pcvt->SetOutputCoordByEPSG(output);

    _geo_projection = shpDataset1->GetProjectionRef(); // ͶӰ��Ϣ
    shpDataset1->GetGeoTransform(_adf_geo_transform);  // ����������Ϣ
    //_eType = GDALGetRasterDataType(shpDataset1->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset1->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // �ҵ� ID �ֶε�����
    for (int i = 0; i < featureDefn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = featureDefn->GetFieldDefn(i);
        if (std::string(fieldDefn->GetNameRef()) == "Id") {
            idFieldIndex = i;
            break;
        }
    }

    if (idFieldIndex == -1) {
        //std::cerr << "ID field not found!" << std::endl;
        return false;
    }

    std::string type = GbkToUtf8("����");
    std::string height = GbkToUtf8("¥��");
    std::string name = GbkToUtf8("����");
    //std::string type = "type";
    //std::string height = "height";

    int idFieldIndex1 = featureDefn->GetFieldIndex("Id");
    int typeFieldIndex1 = featureDefn->GetFieldIndex(type.c_str());
    int heightFieldIndex1 = featureDefn->GetFieldIndex(height.c_str());
    int nameFieldIndex = featureDefn->GetFieldIndex(name.c_str());

    if (idFieldIndex1 == -1 || typeFieldIndex1 == -1 || heightFieldIndex1 == -1) {
        //std::cerr << "One or more fields not found!" << std::endl;
        GDALClose(shpDataset1);
        return false;
    }
    std::wstring wideString = L"�ص�";
    OGRFeature* poFeature;
    int newID = 0;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {

        poFeature->SetField(idFieldIndex, newID);

        if (poLayer->SetFeature(poFeature) != OGRERR_NONE) {
            std::cerr << "Failed to update feature ID" << std::endl;
        }
        newID++;
        int id = poFeature->GetFieldAsInteger(idFieldIndex1);
        std::wstring type = Utf8ToWstring(poFeature->GetFieldAsString(typeFieldIndex1));
        std::wstring type1 = Utf8ToWstring(poFeature->GetFieldAsString(nameFieldIndex));
        std::wstring result;
        size_t charCount = 0;
        for (size_t i = 0; i < type1.size(); ++i) {
            if (type1[i] >= 0x4E00 && type1[i] <= 0x9FFF) { // �����������ַ���Χ
                ++charCount;
                result += type1[i];
                if (charCount >= 2) {
                    break;
                }
            }
        }
        double height = poFeature->GetFieldAsDouble(heightFieldIndex1);
        if (result == wideString) {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            if (poGeometry != nullptr && poGeometry->getGeometryType() == wkbPolygon) {
                BuildingInfo info;
                info.type = type1;
                info.height = height;
                OGRPolygon* poPolygon = (OGRPolygon*)poGeometry;
                OGRLinearRing* poRing = poPolygon->getExteriorRing();
                if (poRing != nullptr) {
                    int numPoints = poRing->getNumPoints();
                    std::vector<Point2D> points;
                    for (int i = 0; i < numPoints; ++i) {
                        Point2D pt;
                        pt._x = poRing->getX(i);
                        pt._y = poRing->getY(i);

                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    info.points = points;
                }
                keyBuildingMap[id] = info;
            }
        }
        OGRGeometry* poGeometry = poFeature->GetGeometryRef();
        if (poGeometry != nullptr && poGeometry->getGeometryType() == wkbPolygon) {
            BuildingInfo info;
            info.type = type;
            info.height = height;

            // �������ε�ÿ����
            OGRPolygon* poPolygon = (OGRPolygon*)poGeometry;
            OGRLinearRing* poRing = poPolygon->getExteriorRing();
            if (poRing != nullptr) {
                int numPoints = poRing->getNumPoints();
                std::vector<Point2D> points;
                for (int i = 0; i < numPoints; ++i) {
                    Point2D pt;
                    pt._x = poRing->getX(i);
                    pt._y = poRing->getY(i);

                    double z = 0;
                    if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                        return 1;
                    }
                    points.push_back(pt);
                }
                info.points = points;
            }
            allBuild[id] = info;
        }
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(shpDataset1);
    return true;
}

bool DroneNest::openShpFile_point(std::string pointFile)
{
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
    CPLSetConfigOption("SHAPE_ENCODING", "");
    GDALDataset* shpDataset = (GDALDataset*)GDALOpenEx(pointFile.c_str(), GDAL_OF_UPDATE, nullptr, nullptr, nullptr);
    if (shpDataset == nullptr) {
        GDALClose(shpDataset);
        return false;
    }

    int input(4547), output(4326);
    pcvt->SetInputCoordByEPSG(input);
    pcvt->SetOutputCoordByEPSG(output);

    shpDataset->GetGeoTransform(_adf_geo_transform);  // ����������Ϣ
    //_eType = GDALGetRasterDataType(shpDataset->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // �ҵ� ID �ֶε�����
    for (int i = 0; i < featureDefn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = featureDefn->GetFieldDefn(i);
        if (std::string(fieldDefn->GetNameRef()) == "Id") {
            idFieldIndex = i;
            break;
        }
    }

    if (idFieldIndex == -1) {
        return false;
    }

    std::string type = GbkToUtf8("����");

    int idFieldIndex1 = featureDefn->GetFieldIndex("Id");
    int typeFieldIndex1 = featureDefn->GetFieldIndex(type.c_str());

    if (idFieldIndex1 == -1 || typeFieldIndex1 == -1) {
        GDALClose(shpDataset);
        return false;
    }

    OGRFeature* poFeature;
    int newID = 0;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {

        poFeature->SetField(idFieldIndex, newID);

        if (poLayer->SetFeature(poFeature) != OGRERR_NONE) {
            std::cerr << "Failed to update feature ID" << std::endl;
        }
        newID++;

        int id = poFeature->GetFieldAsInteger(idFieldIndex1);
        std::string type = poFeature->GetFieldAsString(typeFieldIndex1);

        OGRGeometry* poGeometry = poFeature->GetGeometryRef();
        if (poGeometry != nullptr && poGeometry->getGeometryType() == wkbPoint) {
            // ����㼸��
            OGRPoint* poPoint = (OGRPoint*)poGeometry;
            Point2D pt;
            pt._x = poPoint->getX();
            pt._y = poPoint->getY();
          
            double z = 0;
            if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                return 1;
            }

            pointCollection.push_back(pt);  // ����洢��������
        }
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(shpDataset);
    return true;
}

bool DroneNest::openShpFile_road(std::string lineFile)
{
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
    CPLSetConfigOption("SHAPE_ENCODING", "");
    GDALDataset* shpDataset = (GDALDataset*)GDALOpenEx(lineFile.c_str(), GDAL_OF_UPDATE, nullptr, nullptr, nullptr);
    if (shpDataset == nullptr) {
        GDALClose(shpDataset);
        return false;
    }

    int input(4547), output(4326);
    pcvt->SetInputCoordByEPSG(input);
    pcvt->SetOutputCoordByEPSG(output);

    shpDataset->GetGeoTransform(_adf_geo_transform);  // ����������Ϣ
    //_eType = GDALGetRasterDataType(shpDataset->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // �ҵ� ID �ֶε�����
    for (int i = 0; i < featureDefn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = featureDefn->GetFieldDefn(i);
        if (std::string(fieldDefn->GetNameRef()) == "Id") {
            idFieldIndex = i;
            break;
        }
    }

    if (idFieldIndex == -1) {
        return false;
    }

    std::string type = GbkToUtf8("����");
    std::string type1 = GbkToUtf8("����");

    int idFieldIndex1 = featureDefn->GetFieldIndex("Id");
    int typeFieldIndex1 = featureDefn->GetFieldIndex(type.c_str());
    int typeFieldIndex2 = featureDefn->GetFieldIndex(type1.c_str());

    if (idFieldIndex1 == -1 || typeFieldIndex1 == -1 || typeFieldIndex2 == -1) {
        GDALClose(shpDataset);
        return false;
    }

    OGRFeature* poFeature;
    int newID = 0;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {

        poFeature->SetField(idFieldIndex, newID);

        if (poLayer->SetFeature(poFeature) != OGRERR_NONE) {
            std::cerr << "Failed to update feature ID" << std::endl;
        }
        newID++;

        int id = poFeature->GetFieldAsInteger(idFieldIndex1);
        std::wstring type = Utf8ToWstring(poFeature->GetFieldAsString(typeFieldIndex1));
        std::wstring result;
        size_t charCount = 0;
        for (size_t i = 0; i < type.size(); ++i) {
            if (type[i] >= 0x4E00 && type[i] <= 0x9FFF) { // �����������ַ���Χ
                ++charCount;
                result += type[i];
                if (charCount >= 2) {
                    break;
                }
            }
        }

        std::wstring type1 = Utf8ToWstring(poFeature->GetFieldAsString(typeFieldIndex2));
        //std::wstring result1;
        //size_t charCount1 = 0;
        //for (size_t i = 0; i < type1.size(); ++i) {
        //    if (type1[i] >= 0x4E00 && type1[i] <= 0x9FFF) { // �����������ַ���Χ
        //        ++charCount1;
        //        result1 += type1[i];
        //        if (charCount1 >= 2) {
        //            break;
        //        }
        //    }
        //}
        std::wstring wideString1 = L"�������";
        std::wstring wideString = L"��·";
        std::wstring wideString_t = L"���";
        //std::wstring wideString_r = L"��·";
        if (result == wideString) {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            if (poGeometry != nullptr) {
                OGRwkbGeometryType geomType = poGeometry->getGeometryType();
                BuildingInfo info;
                info.type = result;

                if (geomType == wkbLineString) {
                    OGRLineString* poLine = (OGRLineString*)poGeometry;
                    int numPoints = poLine->getNumPoints();
                    std::vector<Point2D> points;
                    for (int i = 0; i < numPoints; ++i) {
                        Point2D pt;
                        pt._x = poLine->getX(i);
                        pt._y = poLine->getY(i);
                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    info.points = points;
                }
                else if (geomType == wkbMultiLineString) {
                    OGRMultiLineString* poMultiLine = (OGRMultiLineString*)poGeometry;
                    int numGeometries = poMultiLine->getNumGeometries();
                    for (int i = 0; i < numGeometries; ++i) {
                        OGRLineString* poLine = (OGRLineString*)poMultiLine->getGeometryRef(i);
                        int numPoints = poLine->getNumPoints();
                        std::vector<Point2D> points;
                        for (int j = 0; j < numPoints; ++j) {
                            Point2D pt;
                            pt._x = poLine->getX(j);
                            pt._y = poLine->getY(j);
                            double z = 0;
                            if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                                return 1;
                            }
                            points.push_back(pt);
                        }
                        info.points = points;
                    }
                }
                railwayMap[id] = info;
            }
        }
        else if(result == wideString_t){
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            if (poGeometry != nullptr) {
                OGRwkbGeometryType geomType = poGeometry->getGeometryType();
                BuildingInfo info;
                info.type = type;

                if (geomType == wkbLineString) {
                    OGRLineString* poLine = (OGRLineString*)poGeometry;
                    int numPoints = poLine->getNumPoints();
                    std::vector<Point2D> points;
                    for (int i = 0; i < numPoints; ++i) {
                        Point2D pt;
                        pt._x = poLine->getX(i);
                        pt._y = poLine->getY(i);
                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    info.points = points;
                }
                else if (geomType == wkbMultiLineString) {
                    OGRMultiLineString* poMultiLine = (OGRMultiLineString*)poGeometry;
                    int numGeometries = poMultiLine->getNumGeometries();
                    for (int i = 0; i < numGeometries; ++i) {
                        OGRLineString* poLine = (OGRLineString*)poMultiLine->getGeometryRef(i);
                        int numPoints = poLine->getNumPoints();
                        std::vector<Point2D> points;
                        for (int j = 0; j < numPoints; ++j) {
                            Point2D pt;
                            pt._x = poLine->getX(j);
                            pt._y = poLine->getY(j);
                            double z = 0;
                            if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                                return 1;
                            }
                            points.push_back(pt);
                        }
                        info.points = points;
                    }
                }
                trainsMap[id] = info;
            }
        }
        /*else if (result == wideString_r) {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            if (poGeometry != nullptr) {
                OGRwkbGeometryType geomType = poGeometry->getGeometryType();
                BuildingInfo info;
                info.type = type;

                if (geomType == wkbLineString) {
                    OGRLineString* poLine = (OGRLineString*)poGeometry;
                    int numPoints = poLine->getNumPoints();
                    std::vector<Point2D> points;
                    for (int i = 0; i < numPoints; ++i) {
                        Point2D pt;
                        pt._x = poLine->getX(i);
                        pt._y = poLine->getY(i);
                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    info.points = points;
                }
                else if (geomType == wkbMultiLineString) {
                    OGRMultiLineString* poMultiLine = (OGRMultiLineString*)poGeometry;
                    int numGeometries = poMultiLine->getNumGeometries();
                    for (int i = 0; i < numGeometries; ++i) {
                        OGRLineString* poLine = (OGRLineString*)poMultiLine->getGeometryRef(i);
                        int numPoints = poLine->getNumPoints();
                        std::vector<Point2D> points;
                        for (int j = 0; j < numPoints; ++j) {
                            Point2D pt;
                            pt._x = poLine->getX(j);
                            pt._y = poLine->getY(j);
                            double z = 0;
                            if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                                return 1;
                            }
                            points.push_back(pt);
                        }
                        info.points = points;
                    }
                }
                roadMap[id] = info;
            }
        }*/
        else if (type1 == wideString1) {//�������
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            if (poGeometry != nullptr) {
                OGRwkbGeometryType geomType = poGeometry->getGeometryType();
                BuildingInfo info;
                info.type = type1;

                if (geomType == wkbLineString) {
                    OGRLineString* poLine = (OGRLineString*)poGeometry;
                    int numPoints = poLine->getNumPoints();
                    std::vector<Point2D> points;
                    for (int i = 0; i < numPoints; ++i) {
                        Point2D pt;
                        pt._x = poLine->getX(i);
                        pt._y = poLine->getY(i);
                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    info.points = points;
                }
                else if (geomType == wkbMultiLineString) {
                    OGRMultiLineString* poMultiLine = (OGRMultiLineString*)poGeometry;
                    int numGeometries = poMultiLine->getNumGeometries();
                    for (int i = 0; i < numGeometries; ++i) {
                        OGRLineString* poLine = (OGRLineString*)poMultiLine->getGeometryRef(i);
                        int numPoints = poLine->getNumPoints();
                        std::vector<Point2D> points;
                        for (int j = 0; j < numPoints; ++j) {
                            Point2D pt;
                            pt._x = poLine->getX(j);
                            pt._y = poLine->getY(j);
                            double z = 0;
                            if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                                return 1;
                            }
                            points.push_back(pt);
                        }
                        info.points = points;
                    }
                }
                tunnelMap[id] = info;
            }
        }
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(shpDataset);
    return true;
}

std::string convertWideToMultiByte(const std::wstring& wideStr) {
    int size_needed = WideCharToMultiByte(CP_ACP, 0, &wideStr[0], (int)wideStr.size(), NULL, 0, NULL, NULL);
    std::vector<char> str(size_needed, 0);
    WideCharToMultiByte(CP_ACP, 0, &wideStr[0], (int)wideStr.size(), &str[0], size_needed, NULL, NULL);
    return std::string(&str[0], size_needed);
}

std::wstring removeFileName(const std::wstring& path) {
    std::wstring::size_type pos = path.find_last_of(L"\\/");
    if (pos != std::wstring::npos) {
        return path.substr(0, pos); // ����ȥ���ļ�����·��
    }
    return path;
}

bool DroneNest::openRoadShpFile(std::string roadFile)
{
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
    CPLSetConfigOption("SHAPE_ENCODING", "");
    GDALDataset* shpDataset = (GDALDataset*)GDALOpenEx(roadFile.c_str(), GDAL_OF_UPDATE, nullptr, nullptr, nullptr);
    if (shpDataset == nullptr) {
        GDALClose(shpDataset);
        return false;
    }

    int input(4547), output(4326);
    pcvt->SetInputCoordByEPSG(input);
    pcvt->SetOutputCoordByEPSG(output);

    shpDataset->GetGeoTransform(_adf_geo_transform);  // ����������Ϣ
    //_eType = GDALGetRasterDataType(shpDataset->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // �ҵ� ID �ֶε�����
    for (int i = 0; i < featureDefn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = featureDefn->GetFieldDefn(i);
        if (std::string(fieldDefn->GetNameRef()) == "Id") {
            idFieldIndex = i;
            break;
        }
    }

    if (idFieldIndex == -1) {
        return false;
    }

    int idFieldIndex1 = featureDefn->GetFieldIndex("Id");

    if (idFieldIndex1 == -1) {
        GDALClose(shpDataset);
        return false;
    }

    OGRFeature* poFeature;
    int newID = 0;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {

        poFeature->SetField(idFieldIndex, newID);

        if (poLayer->SetFeature(poFeature) != OGRERR_NONE) {
            std::cerr << "Failed to update feature ID" << std::endl;
        }
        newID++;

        int id = poFeature->GetFieldAsInteger(idFieldIndex1);

        OGRGeometry* poGeometry = poFeature->GetGeometryRef();
        if (poGeometry != nullptr) {
            OGRwkbGeometryType geomType = poGeometry->getGeometryType();
            BuildingInfo info;

            if (geomType == wkbLineString) {
                OGRLineString* poLine = (OGRLineString*)poGeometry;
                int numPoints = poLine->getNumPoints();
                std::vector<Point2D> points;
                for (int i = 0; i < numPoints; ++i) {
                    Point2D pt;
                    pt._x = poLine->getX(i);
                    pt._y = poLine->getY(i);
                    double z = 0;
                    if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                        return 1;
                    }
                    points.push_back(pt);
                }
                info.points = points;
            }
            else if (geomType == wkbMultiLineString) {
                OGRMultiLineString* poMultiLine = (OGRMultiLineString*)poGeometry;
                int numGeometries = poMultiLine->getNumGeometries();
                for (int i = 0; i < numGeometries; ++i) {
                    OGRLineString* poLine = (OGRLineString*)poMultiLine->getGeometryRef(i);
                    int numPoints = poLine->getNumPoints();
                    std::vector<Point2D> points;
                    for (int j = 0; j < numPoints; ++j) {
                        Point2D pt;
                        pt._x = poLine->getX(j);
                        pt._y = poLine->getY(j);
                        double z = 0;
                        if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                            return 1;
                        }
                        points.push_back(pt);
                    }
                    info.points = points;
                }
            }
            roadMap_t[id] = info;
        }
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(shpDataset);

    for (std::map<int, BuildingInfo>::const_iterator it = roadMap_t.begin(); it != roadMap_t.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        roadscale_t[id] = info;
    }
    return true;
}

bool DroneNest::openSportShpFile(std::string sportFile)
{
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
    CPLSetConfigOption("SHAPE_ENCODING", "");
    GDALDataset* shpDataset1 = (GDALDataset*)GDALOpenEx(sportFile.c_str(), GDAL_OF_UPDATE, nullptr, nullptr, nullptr);
    if (shpDataset1 == nullptr) {
        GDALClose(shpDataset1);
        return false;
    }

    int input(4547), output(4326);
    pcvt->SetInputCoordByEPSG(input);
    pcvt->SetOutputCoordByEPSG(output);

    _geo_projection = shpDataset1->GetProjectionRef(); // ͶӰ��Ϣ
    shpDataset1->GetGeoTransform(_adf_geo_transform);  // ����������Ϣ
    //_eType = GDALGetRasterDataType(shpDataset1->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset1->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // �ҵ� ID �ֶε�����
    for (int i = 0; i < featureDefn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = featureDefn->GetFieldDefn(i);
        if (std::string(fieldDefn->GetNameRef()) == "Id") {
            idFieldIndex = i;
            break;
        }
    }

    if (idFieldIndex == -1) {
        return false;
    }

    int idFieldIndex1 = featureDefn->GetFieldIndex("Id");

    if (idFieldIndex1 == -1) {
        GDALClose(shpDataset1);
        return false;
    }

    OGRFeature* poFeature;
    int newID = 0;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {
        poFeature->SetField(idFieldIndex, newID);
        if (poLayer->SetFeature(poFeature) != OGRERR_NONE) {
            std::cerr << "Failed to update feature ID" << std::endl;
        }
        newID++;

        int id = poFeature->GetFieldAsInteger(idFieldIndex1);
        OGRGeometry* poGeometry = poFeature->GetGeometryRef();
        if (poGeometry != nullptr && poGeometry->getGeometryType() == wkbPolygon) {
            BuildingInfo info;
            OGRPolygon* poPolygon = (OGRPolygon*)poGeometry;
            OGRLinearRing* poRing = poPolygon->getExteriorRing();
            if (poRing != nullptr) {
                int numPoints = poRing->getNumPoints();
                std::vector<Point2D> points;
                for (int i = 0; i < numPoints; ++i) {
                    Point2D pt;
                    pt._x = poRing->getX(i);
                    pt._y = poRing->getY(i);

                    double z = 0;
                    if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                        return 1;
                    }
                    points.push_back(pt);
                }
                info.points = points;
            }
            sportMap[id] = info;
        }
        OGRFeature::DestroyFeature(poFeature);
    }
    GDALClose(shpDataset1);

    for (std::map<int, BuildingInfo>::const_iterator it = sportMap.begin(); it != sportMap.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        sportScale[id] = info;
    }
    return true;
}

bool DroneNest::readDsm(std::string dsmFile, std::string projshare)
{
    //test.Open("D:\\�������˻�\\DOM-1M\\DOM-1M_DSM.tif","D:\\5kmapping\\SatBlockBA@20240130\\proj-share");
    //test.Read();
    if (!test.Open(dsmFile.c_str(), projshare.c_str())) {
        return false;
    }
    int* band_map = new int[test.GetBands()];
    for (int i = 0; i < test.GetBands(); i++) {
        band_map[i] = i + 1;
    }
    if (!test.Read(test.GetBands(), 0, 0, test.GetRows(), test.GetCols(), test.GetDataType(), band_map)) {
        return false;
    }
    //if (!test.Open("D:\\�������˻�\\DOM-1M\\DOM-1M_DSM.tif", "D:\\5kmapping\\SatBlockBA@20240130\\proj-share")) {
    //    return false;
    //}
    return true;
}

bool DroneNest::readJsonFile(std::string json_notfly)
{
    std::ifstream inputFile(json_notfly);
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
        return 1;
    }

    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::string errs;

    bool parsingSuccessful = Json::parseFromStream(readerBuilder, inputFile, &root, &errs);
    if (!parsingSuccessful) {
        //cerr << "Failed to parse JSON: " << errs << endl;
        return false;
    }

    if (root.isMember("geometry") && root["geometry"].isMember("coordinates")) {
        const Json::Value& coordinates = root["geometry"]["coordinates"];

        polygons.clear();
        polygons.reserve(coordinates.size());

        for (const auto& ring : coordinates) {
            polygon polygon_t;
            double x, y;
            for (const auto& coord : ring) {
                if (coord.size() >= 2) {
                    double lon = coord[0].asDouble();
                    double lat = coord[1].asDouble();
                    test.WGS84LBHToPixelCoor(lon, lat, 0, x, y);
                    polygon_t.points.emplace_back(x, y);
                }
            }
            polygons.push_back(polygon_t);
        }
    }
    else {
        return false;
    }
    return true;
}
void DroneNest::buildPreprocessing()
{
    buildingMap_t = buildingMap;

    for (std::map<int, BuildingInfo>::const_iterator it = allBuild.begin(); it != allBuild.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        info.type = it->second.type;
        info.height = it->second.height;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        allBuildscale[id] = info;
    }
    for (std::map<int, BuildingInfo>::const_iterator it = keyBuildingMap.begin(); it != keyBuildingMap.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        info.type = it->second.type;
        info.height = it->second.height;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        keyBuildingScale[id] = info;
    }
    
    for (std::map<int, BuildingInfo>::const_iterator it = transSubMap.begin(); it != transSubMap.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        info.type = it->second.type;
        info.height = it->second.height;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        transSubscale[id] = info;
    }

    for (std::map<int, BuildingInfo>::const_iterator it = railwayMap.begin(); it != railwayMap.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        info.type = it->second.type;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        railwayscale[id] = info;
    }

    for (std::map<int, BuildingInfo>::const_iterator it = trainsMap.begin(); it != trainsMap.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        info.type = it->second.type;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        trainsscale[id] = info;
    }

    for (std::map<int, BuildingInfo>::const_iterator it = tunnelMap.begin(); it != tunnelMap.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        info.type = it->second.type;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        tunnelScale[id] = info;
    }

    for (const auto& point : pointCollection) {
        double x, y;
        test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
        Point2D s(x, y);
        pointCollectionscale.push_back(s);
    }
}

Point2D unitVector(const Point2D& a, const Point2D& b) {
    double dx = b._x - a._x;
    double dy = b._y - a._y;
    double length = std::sqrt(dx * dx + dy * dy);
    return { dx / length, dy / length };
}

void DroneNest::expandPolygon(const std::vector<Point2D>& points, double expansionDistance, std::vector<uint8_t>& data, int nCols, int nRows) {
    // �����������εı߽�
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();

    for (const auto& point : points) {
        minX = std::min(minX, point._x);
        minY = std::min(minY, point._y);
        maxX = std::max(maxX, point._x);
        maxY = std::max(maxY, point._y);
    }

    // �������εı߽�
    minX -= expansionDistance;
    minY -= expansionDistance;
    maxX += expansionDistance;
    maxY += expansionDistance;

    // �������ڵ���������Ϊ0
    int startX = std::max(0, static_cast<int>(minX));
    int startY = std::max(0, static_cast<int>(minY));
    int endX = std::min(nCols - 1, static_cast<int>(maxX));
    int endY = std::min(nRows - 1, static_cast<int>(maxY));
     
    for (int y = startY; y <= endY; ++y) {
        for (int x = startX; x <= endX; ++x) {
            data[y * nCols + x] = 0; // ����Ϊ0
        }
    }
}

void DroneNest::processBuildings(std::map<int, BuildingInfo>& allBuildscale, std::vector<uint8_t>& data) {
    int nCols = test.GetCols();
    int nRows = test.GetRows();
    
    for (auto& kv : allBuildscale) {
        BuildingInfo& building = kv.second;

        if (building.height > dis_build_g_l) {
            expandPolygon(building.points, circleRadius_t_l, data, nCols, nRows);
        }
        else if (building.height > dis_build_l) {
            expandPolygon(building.points, circleRadius_l, data, nCols, nRows);
        }
    }
}

void DroneNest::createTifWithBuildings(std::map<int, BuildingInfo>& allBuildscale, int slopeWidth, int slopeHeight) {

    processBuildings(allBuildscale, maskData);
    //GDALDataset* dataset = (GDALDataset*)GDALOpen(buildScale.c_str(), GA_Update);
    //GDALRasterBand* band = dataset->GetRasterBand(1);
    //band->RasterIO(GF_Write, 0, 0, test.GetCols(), test.GetRows(), maskData.data(), test.GetCols(), test.GetRows(), GDT_Byte, 0, 0);
    //GDALClose(dataset);
}

void DroneNest::groundPreprocessing(std::string slopeFile)
{
    GDALAllRegister();
    CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
    CPLSetConfigOption("SHAPE_ENCODING", "");

    auto start = std::chrono::high_resolution_clock::now();
    //GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    //if (driver == nullptr) {
    //    std::cerr << "GTiff driver not available." << std::endl;
    //    return;
    //}
    
    // ����һ���ڴ����ݼ�
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("MEM");
    if (driver == nullptr) {
        std::cerr << "MEM driver not available." << std::endl;
        return;
    }
    //std::string ansiPath;
    //TCHAR path[MAX_PATH];
    //if (GetModuleFileName(NULL, path, MAX_PATH) != 0) {
    //    std::wstring widePath(path);
    //    std::wstring directory = removeFileName(widePath);
    //    ansiPath = convertWideToMultiByte(directory);
    //    std::cout << "Directory path: " << ansiPath << std::endl;
    //}
    //else {
    //    std::cerr << "Failed to get program path." << std::endl;
    //}
    //ansiPath = ansiPath + "\\1.tif";
    //std::vector<char> modifiable_str(ansiPath.size() + 1);
    //std::copy(ansiPath.begin(), ansiPath.end(), modifiable_str.begin());
    //modifiable_str.back() = '\0';
    //char* modifiable_cstr = modifiable_str.data();
    //GDALDataset* dataset = driver->Create(modifiable_cstr, test.GetCols(), test.GetRows(), 1, GDT_Byte, nullptr);
    //if (dataset == nullptr) {
    //    std::cerr << "Failed to create dataset." << std::endl;
    //    return;
    //}
    GDALDataset* dataset = driver->Create("", test.GetCols(), test.GetRows(), 1, GDT_Byte, nullptr);
    if (dataset == nullptr) {
        std::cerr << "Failed to create dataset." << std::endl;
        return;
    }

    GDALRasterBand* band = dataset->GetRasterBand(1);
    if (band == nullptr) {
        std::cerr << "Failed to get raster band." << std::endl;
        GDALClose(dataset);
        return;
    }
  
    std::vector<uint8_t> data(test.GetCols() * test.GetRows(), 255); // ��������ֵ��ʼ��Ϊ 255
    CPLErr err = band->RasterIO(GF_Write, 0, 0, test.GetCols(), test.GetRows(), data.data(), test.GetCols(), test.GetRows(), GDT_Byte, 0, 0);
    if (err != CE_None) {
        std::cerr << "Failed to write raster data." << std::endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Time taken: " << elapsed.count() << " seconds" << std::endl;

    for (std::map<int, BuildingInfo>::const_iterator it = buildingMap_t.begin(); it != buildingMap_t.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        info.type = it->second.type;
        info.height = it->second.height;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        buildscale_t[id] = info;
    }

    //for (auto& entry : allBuildscale) {
    //    BuildingInfo& building = entry.second;

    //    // ���߶ȷ�Χ
    //    if (building.height > 20.0 && building.height < 50.0) {
    //        std::vector<Point2D> newPoints;

    //        // ����ÿ����
    //        for (size_t i = 0; i < building.points.size(); ++i) {
    //            Point2D& currentPoint = building.points[i];
    //            Point2D& nextPoint = building.points[(i + 1) % building.points.size()]; // ��һ���㣬ѭ���ص���һ����

    //            // �����ⷨ��������չ
    //            Point2D normal = outwardNormal(currentPoint, nextPoint);
    //            newPoints.push_back({ currentPoint._x + normal._x * 50.0, currentPoint._y + normal._y * 50.0 });
    //        }

    //        // ���¶���εĵ�
    //        building.points = std::move(newPoints);
    //    }
    //}
    data1.resize(test.GetCols() * test.GetRows(), 255);
   
    //data_t = data1;
    auto start1 = std::chrono::high_resolution_clock::now();
    
    for (const auto& kv : buildscale_t) { //����ҵ��λ
        const BuildingInfo& building = kv.second;
        setPolygonAreaToZero(band, building, dataset->GetRasterXSize(), dataset->GetRasterYSize(), data1);
    }

    for (const auto& kv : allBuildscale) {
        const BuildingInfo& building = kv.second;
        setPolygonAreaToZero(band, building, dataset->GetRasterXSize(), dataset->GetRasterYSize(), data1);
    }

    for (const auto& kv : sportScale) {
        const BuildingInfo& building = kv.second;
        setPolygonAreaToZero(band, building, dataset->GetRasterXSize(), dataset->GetRasterYSize(), data1);
    }

    populateBuildingInfo(polygons, notFly);

    for (const auto& kv : notFly) { //������
        const BuildingInfo& building = kv.second;
        setPolygonAreaToZero(band, building, dataset->GetRasterXSize(), dataset->GetRasterYSize(), data1);
    }

    CPLErr err1 = band->RasterIO(GF_Write, 0, 0, test.GetCols(), test.GetRows(), data1.data(), test.GetCols(), test.GetRows(), GDT_Byte, 0, 0);
    if (err1 != CE_None) {
        std::cerr << "Failed to write raster data." << std::endl;
    }
    //data2 = data1;
    GDALClose(dataset);
    auto end1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed1 = end1 - start1;
    std::cout << "1Time taken: " << elapsed1.count() << " seconds" << std::endl;

    //������������Ĥ
    //GDALDriver* driver1 = GetGDALDriverManager()->GetDriverByName("GTiff");
    //if (driver1 == nullptr) {
    //    std::cerr << "GTiff driver not available." << std::endl;
    //    return;
    //}
    GDALDriver* driver1 = GetGDALDriverManager()->GetDriverByName("MEM");
    if (driver1 == nullptr) {
        std::cerr << "GTiff driver not available." << std::endl;
        return;
    }
    /*std::string ansiPath1;
    TCHAR path1[MAX_PATH];
    if (GetModuleFileName(NULL, path1, MAX_PATH) != 0) {
        std::wstring widePath1(path1);
        std::wstring directory1 = removeFileName(widePath1);
        ansiPath1 = convertWideToMultiByte(directory1);
        std::cout << "Directory path: " << ansiPath1 << std::endl;
    }
    else {
        std::cerr << "Failed to get program path." << std::endl;
    }
    Coverage = ansiPath1 + "\\coverageScale.tif";;
    ansiPath1 = ansiPath1 + "\\buildScale.tif";
    buildScale = ansiPath1;
    std::vector<char> modifiable_str1(ansiPath1.size() + 1);
    std::copy(ansiPath1.begin(), ansiPath1.end(), modifiable_str1.begin());
    modifiable_str1.back() = '\0';
    char* modifiable_cstr1 = modifiable_str1.data();*/
    dataset_buildScale = driver1->Create("", test.GetCols(), test.GetRows(), 1, GDT_Byte, nullptr);
    if (dataset_buildScale == nullptr) {
        std::cerr << "Failed to create dataset." << std::endl;
        return;
    }

    band_buildScale = dataset_buildScale->GetRasterBand(1);
    if (band_buildScale == nullptr) {
        std::cerr << "Failed to get raster band." << std::endl;
        GDALClose(dataset_buildScale);
        return;
    }


    /*GDALDriver* driverss = GetGDALDriverManager()->GetDriverByName("GTiff");
    if (driverss == nullptr) {
        std::cerr << "GTiff driver not available." << std::endl;
        return;
    }*/
    GDALDriver* driverss = GetGDALDriverManager()->GetDriverByName("MEM");
    if (driverss == nullptr) {
        std::cerr << "GTiff driver not available." << std::endl;
        return;
    }
    //GDALDataset* datasetss = driverss->Create(Coverage.c_str(), test.GetCols(), test.GetRows(), 1, GDT_Byte, nullptr);
    //if (datasetss == nullptr) {
    //    std::cerr << "Failed to create dataset." << std::endl;
    //    return;
    //}
    GDALDataset* datasetss = driverss->Create("", test.GetCols(), test.GetRows(), 1, GDT_Byte, nullptr);
    if (datasetss == nullptr) {
        std::cerr << "Failed to create dataset." << std::endl;
        return;
    }
    GDALRasterBand* bandss = datasetss->GetRasterBand(1);
    if (bandss == nullptr) {
        std::cerr << "Failed to get raster band." << std::endl;
        GDALClose(datasetss);
        return;
    }
    datass.resize(test.GetCols() * test.GetRows(), 255); // ��������ֵ��ʼ��Ϊ 255
    double ss = test.m_pData.m_NoDataValue;
    for (int y = 0; y < test.GetRows(); ++y) {
        for (int x = 0; x < test.GetCols(); ++x) {
            double z;
            test.m_pData.GetData(x, y, &z);
            if (z == ss) {
                datass[y * test.GetCols() + x] = 0;
            }
        }
    }
    CPLErr errss = bandss->RasterIO(GF_Write, 0, 0, test.GetCols(), test.GetRows(), datass.data(), test.GetCols(), test.GetRows(), GDT_Byte, 0, 0);
    if (errss != CE_None) {
        std::cerr << "Failed to write raster data." << std::endl;
    }

    maskData.resize(test.GetCols() * test.GetRows(), 255);
    for (const auto& kv : allBuildscale) {
        const BuildingInfo& building = kv.second;
        setPolygonAreaToZero(band_buildScale, building, dataset_buildScale->GetRasterXSize(), dataset_buildScale->GetRasterYSize(), maskData);
    }
    CPLErr err2 = band_buildScale->RasterIO(GF_Write, 0, 0, test.GetCols(), test.GetRows(), maskData.data(), test.GetCols(), test.GetRows(), GDT_Byte, 0, 0);
    if (err2 != CE_None) {
        std::cerr << "Failed to write raster data." << std::endl;
    }
    GDALClose(dataset_buildScale);
    GDALClose(datasetss);

    GDALDataset* poDataset = (GDALDataset*)GDALOpen(slopeFile.c_str(), GA_ReadOnly);
    if (poDataset == nullptr) {
        std::cerr << "Error: Could not load slope file!" << std::endl;
        return;
    }
    int slopeWidth = poDataset->GetRasterXSize();
    int slopeHeight = poDataset->GetRasterYSize();
    slope.create(slopeHeight, slopeWidth, CV_32FC1);
    int result = poDataset->GetRasterBand(1)->RasterIO(GF_Read, 0, 0, slopeWidth, slopeHeight, slope.ptr<float>(), slopeWidth, slopeHeight, GDT_Float32, 0, 0);
    if (result != CE_None) {
        return;
    }
    GDALClose(poDataset);
}

Point2D DroneNest::CalculatePolygonCentroid(const std::vector<Point2D>& points)
{
    double area = 0.0;
    double Cx = 0.0;
    double Cy = 0.0;
    int n = points.size();

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        double xi = points[i]._x;
        double yi = points[i]._y;
        double xj = points[j]._x;
        double yj = points[j]._y;

        double temp = xi * yj - xj * yi;
        area += temp;
        Cx += (xi + xj) * temp;
        Cy += (yi + yj) * temp;
    }

    area /= 2.0;
    Cx /= (6.0 * area);
    Cy /= (6.0 * area);

    return Point2D{ Cx, Cy };
}

//bool DroneNest::2000Towgs84()
//{
//    int input(4326), output(4547);
//    pcvt->SetInputCoordByEPSG(input);
//    pcvt->SetOutputCoordByEPSG(output);
//
//    double a, b, c;
//    for (const auto& polygon1 : polygons) {
//        //std::cout << "Polygon:" << std::endl;
//        polygon new_polygon;
//        for (const auto& point : polygon1.points) {
//			double w = point.first;
//			double f = point.second;
//            double z = 0;
//
//			if (pcvt->Transform(1, &w, &f, &z) != 1) {
//			    return 1;
//			}
//
//            //std::cout << "Point - X: " << point.first << ", Y: " << point.second << std::endl;
//            test.WGS84LBHToPixelCoor(point.first, point.second,0,a,b);
//            new_polygon.points.emplace_back(test.px_t, test.py_t);
//        }
//        polygons_t.push_back(new_polygon);
//    }
//    return true;
//}

std::string squareToWKT(const Square& square) {
    char wkt[512];
    snprintf(wkt, sizeof(wkt),
        "POLYGON ((%f %f, %f %f, %f %f, %f %f, %f %f))",
        square.bottomLeft._x, square.bottomLeft._y,
        square.bottomRight._x, square.bottomRight._y,
        square.topRight._x, square.topRight._y,
        square.topLeft._x, square.topLeft._y,
        square.bottomLeft._x, square.bottomLeft._y);
    return std::string(wkt);
}

// д�� .prj �ļ������ÿռ�ο���Ϣ
void writePRJFile(const char* pszFilename, const OGRSpatialReference& oSRS) {
    std::ofstream file(std::string(pszFilename) + ".prj");
    if (file.is_open()) {
        char* pszWKT = nullptr;
        oSRS.exportToWkt(&pszWKT);
        file << pszWKT;
        CPLFree(pszWKT);
        file.close();
    }
    else {
        std::cerr << "Failed to create .prj file.\n";
    }
}

Square DroneNest::shiftSquare(const Square& square) {
    Square shiftedSquare = square;
    double x, y, z;
    test.PixelCoorToWGS84LBH(shiftedSquare.bottomLeft._x, shiftedSquare.bottomLeft._y, x, y, z);
    shiftedSquare.bottomLeft._x = x;
    shiftedSquare.bottomLeft._y = y;
    //if (pcvt->Transform(1, &shiftedSquare.bottomLeft._x, &shiftedSquare.bottomLeft._y, &z) != 1) {
    //    //return 1;
    //}
    //if (pcvt->Transform(1, &shiftedSquare.bottomRight._x, &shiftedSquare.bottomRight._y, &z) != 1) {
    //    //return 1;
    //}
    //if (pcvt->Transform(1, &shiftedSquare.topLeft._x, &shiftedSquare.topLeft._y, &z) != 1) {
    //    //return 1;
    //}
    //if (pcvt->Transform(1, &shiftedSquare.topRight._x, &shiftedSquare.topRight._y, &z) != 1) {
    //    //return 1;
    //}

    test.PixelCoorToWGS84LBH(shiftedSquare.bottomRight._x, shiftedSquare.bottomRight._y, x, y, z);
    shiftedSquare.bottomRight._x = x;
    shiftedSquare.bottomRight._y = y;

    test.PixelCoorToWGS84LBH(shiftedSquare.topLeft._x, shiftedSquare.topLeft._y, x, y, z);
    shiftedSquare.topLeft._x = x;
    shiftedSquare.topLeft._y = y;

    test.PixelCoorToWGS84LBH(shiftedSquare.topRight._x, shiftedSquare.topRight._y, x, y, z);
    shiftedSquare.topRight._x = x;
    shiftedSquare.topRight._y = y;
   
    return shiftedSquare;
}

// ���������ε����ĵ�
Point2D calculateCenter(const Square& square) {
    return {
        (square.bottomLeft._x + square.bottomRight._x + square.topLeft._x + square.topRight._x) / 4.0,
        (square.bottomLeft._y + square.bottomRight._y + square.topLeft._y + square.topRight._y) / 4.0
    };
}

bool compareDistance(const std::pair<double, std::pair<int, int>>& a, const std::pair<double, std::pair<int, int>>& b) {
    return a.first < b.first;
}

void keepFirstSquareOnly(std::map<int, std::vector<Square>>& squaresMap) {
    for (auto& pair : squaresMap) {
        auto& squares = pair.second;
        if (!squares.empty()) {
            // ������һ��������
            squares = { squares.front() };
        }
    }
}

void DroneNest::processSquaresMap(std::map<int, std::vector<Square>>& squaresMap) {
    if (squaresMap.size() < 2) {
        return;
    }
    std::vector<Point2D> centers;
    std::vector<Square> allSquares;
    std::vector<int> indexedSquares; // �洢�����ε�ԭʼ����
    keepFirstSquareOnly(squaresMap);

    // �ռ����������μ�������
    for (const auto& pair : squaresMap) {
        const std::vector<Square>& squares = pair.second;
        const Square& square = squares[0];
        indexedSquares.push_back(pair.first);
        allSquares.push_back(square);
        centers.push_back(calculateCenter(square));
    }

    std::vector<std::pair<double, std::pair<int, int>>> distances;

    // �����������ĵ�֮��ľ���
    for (size_t i = 0; i < centers.size(); ++i) {
        for (size_t j = i + 1; j < centers.size(); ++j) {
            double dist = distance(centers[i], centers[j]);
            distances.push_back({ dist, {i, j} });
        }
    }

    // ʹ��ð���������� distances
    bubbleSort(distances, compareDistance);
    size_t numSquaresToDelete = 0;

    for (size_t i = 0; i < distances.size() - 1; ++i) {
        double currentDistance = distances[i].first;
        double nextDistance = distances[i + 1].first;

        // �����һ������С��ǰһ�����������
        if (nextDistance < 3 * currentDistance && currentDistance < 1000) {
            numSquaresToDelete = i + 1; // ����Ϊ�����Ǹ�������
        }
    }

    //if (squaresMap.size() <= 7) {
    //    numSquaresToDelete = 2;
    //}
    //else {
    //    // ������Ҫɾ��������������������������������һ��
    //    numSquaresToDelete = allSquares.size() / 2 + 1;
    //}

    // �ҵ���Ҫɾ��������������
    std::set<int> toRemove;
    for (size_t i = 0; i < numSquaresToDelete && i < distances.size(); ++i) {
        //toRemove.insert(indexedSquares[distances[i].second.first]);
        //toRemove.insert(indexedSquares[distances[i].second.second]);

        auto result = toRemove.insert(indexedSquares[distances[i].second.first]);
        // ����Ƿ�ɹ�����
        if (!result.second) {
            auto result = toRemove.insert(indexedSquares[distances[i].second.second]);
        }
    }

    std::map<int, std::vector<Square>> updatedSquaresMap;

    for (const auto& pair : squaresMap) {
        const auto& squares = pair.second;
        std::vector<Square> remainingSquares;
        for (size_t i = 0; i < squares.size(); ++i) {
            // ���ԭʼ�����Ƿ��� toRemove ��
            if (toRemove.find(pair.first) == toRemove.end()) {
                remainingSquares.push_back(squares[i]);
            }
        }
        if (!remainingSquares.empty()) {
            updatedSquaresMap[pair.first] = remainingSquares;
        }
    }
    // ����ԭʼ squaresMap
    squaresMap = std::move(updatedSquaresMap);

    //for (const auto& pair : updatedSquaresMap) {
    //    int key = pair.first; // ��ȡ��ǰ�ļ�
    //    if (squaresMap.find(key) != squaresMap.end()) {
    //        // ��� squaresMap �д����������ɾ����
    //        squaresMap.erase(key);
    //    }
    //}
}

Point2D getSquareCenter(const Square& square) {
    return {
        (square.bottomLeft._x + square.topRight._x) / 2,
        (square.bottomLeft._y + square.topRight._y) / 2
    };
}

void DroneNest::screen_build(const double squareSize, const double threshold, const double circleRadius, 
    const double dis_build, const double dis_build_g,
    const double circleRadius_t, const double circleRadius_s, const double circleRadius_g, const double circleRadius_b,
    const double circleRadius_j, double heightStandard, double nestRadius,
    std::string outputJson, std::string outputShp, std::string position_str)
{
    if (!position_str.empty()) {

        std::vector<std::pair<double, double>> positions;
        std::string trimmed = position_str.substr(2, position_str.size() - 4); // ȥ������[[��]]

        std::istringstream ss(trimmed);
        std::string pair_str;

        while (std::getline(ss, pair_str, ']')) {
            if (pair_str.empty()) continue;

            // ȥ����ͷ��'['�Ϳ��ܴ��ڵĶ���
            pair_str.erase(0, pair_str.find_first_not_of("[")); // ȥ����ͷ��'['
            pair_str.erase(0, pair_str.find_first_not_of(","));
            pair_str.erase(0, pair_str.find_first_not_of("["));

            // ʹ��','�ָγ��
            std::istringstream pair_stream(pair_str);
            double longitude, latitude;
            char comma; // ���ڶ�ȡ����

            // ��ȡ���Ⱥ�γ��
            if (pair_stream >> longitude >> comma >> latitude) {
                positions.emplace_back(longitude, latitude);
            }
        }

        std::map<int, std::vector<Square>> groundsqalone;
        // ת����γ��Ϊ��������
        double a, b;
        test.WGS84LBHToPixelCoor(positions[0].first, positions[0].second, 0, a, b);
        Point2D bl(a, b);

        test.WGS84LBHToPixelCoor(positions[1].first, positions[1].second, 0, a, b);
        Point2D br(a, b);

        test.WGS84LBHToPixelCoor(positions[2].first, positions[2].second, 0, a, b);
        Point2D tl(a, b);

        test.WGS84LBHToPixelCoor(positions[3].first, positions[3].second, 0, a, b);
        Point2D tr(a, b);

        Square square(tr, tl, br, bl);
        square.build_20 = true;
        square.build_50 = true;
        square.communication = true;
        square.expressway = true;
        square.height_difference_t = true;
        square.height_t = true;
        square.railway = true;
        square.transformer = true;
        groundsqalone[0].push_back(square);

        bool heights = false;
        for (auto& ground : groundsqalone) {
            for (auto& square : ground.second) {
                Point2D center = getSquareCenter(square);
                for (const auto& building : buildscale) {
                    if (isPointInPolygon(center, building.second)) {
                        heights = true;
                        square.height = building.second.height;
                        if (square.height > heightStandard) {
                            square.height_t = false;
                        }
                        break;
                    }
                }
                if (!heights) {
                    square.height = 0;
                }
            }
        }

        processSquares_s(groundsqalone, threshold);
        filterSquaresMap_s(groundsqalone, allBuildscale, circleRadius, dis_build, true, 0);
        filterSquaresMap_s(groundsqalone, allBuildscale, circleRadius_t, dis_build_g, true, 1);
        filterSquaresMap_s(groundsqalone, transSubscale, circleRadius_s, -1, true, 2);
        filterSquaresMap_s(groundsqalone, railwayscale, circleRadius_g, -1, true, 3);
        filterSquaresMap_s(groundsqalone, trainsscale, circleRadius_b, -1, true, 4);
        filterSquaresMap_s(groundsqalone, tunnelScale, circleRadius_b, -1, true, 5);

        filterSquaresMap_t_s(groundsqalone, pointCollectionscale, circleRadius_j);

        saveSquaresToJson(groundsqalone, outputJson);
        return;
    }

    std::map<int, BuildingInfo> s = buildingMap;
    //��������20�Ľ�����
    auto it = s.begin();

    while (it != s.end()) {
        if (it->second.height > heightStandard) {
            it = s.erase(it);
        }
        else {
            ++it;
        }
    }
    if (!buildscale.empty()) {
        buildscale.clear();
    }
    for (std::map<int, BuildingInfo>::const_iterator it = s.begin(); it != s.end(); ++it) {
        int id = it->first;
        const BuildingInfo& building = it->second;
        BuildingInfo info;
        info.type = it->second.type;
        info.height = it->second.height;
        std::vector<Point2D> points;
        double x, y;
        for (const auto& point : building.points) {
            test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
            Point2D s(x, y);
            points.push_back(s);
        }
        info.points = points;
        buildscale[id] = info;
    }

    //std::string slopeFilepath = "D:\\shenzhen\\slope.tif";
    //GDALDataset* poDataset = (GDALDataset*)GDALOpen(slopeFilepath.c_str(), GA_ReadOnly);
    //if (poDataset == nullptr) {
    //    std::cerr << "Error: Could not load slope file!" << std::endl;
    //    return;
    //}
    //// ��ȡ�¶�����
    //int slopeWidth = poDataset->GetRasterXSize();
    //int slopeHeight = poDataset->GetRasterYSize();
    //
    //datas.resize(test.GetCols()* test.GetRows(), 255);
    //cv::Mat slope(slopeHeight, slopeWidth, CV_32FC1);
    //poDataset->GetRasterBand(1)->RasterIO(GF_Read, 0, 0, slopeWidth, slopeHeight, slope.ptr<float>(), slopeWidth, slopeHeight, GDT_Float32, 0, 0);
    //for (int y = 0; y < slope.rows / precision; ++y) {
    //    for (int x = 0; x < slope.cols / precision; ++x) {
    //        float slopeValue = slope.at<float>(y * precision, x * precision);
    //        if (slopeValue > threshold || slopeValue < 0) {
    //            datas[y * test.GetCols() + x] = 0;
    //        }
    //    }
    //}

    ////////////////////////////////
    //��齨����ʸ�������е�DSM�����㣨����λ�ڽ������ڵĵ㣩�Ƿ�Ϊˮƽ��
    auto start = std::chrono::high_resolution_clock::now();
    std::map<int, std::vector<Square>> squaresMap_t = findSquaresForBuildings(buildscale, squareSize);
    squaresMap = squaresMap_t;
    processSquares(squaresMap, threshold);

    for (auto it = squaresMap.begin(); it != squaresMap.end(); ++it) {
        int id = it->first;
        std::vector<Square>& squares = it->second;
        for (auto& square : squares) {
            square.build_20 = true;
            square.build_50 = true;
            square.transformer = true;
            square.railway = true;
            square.expressway = true;
            square.communication = true;
            square.height_difference_t = true;
            square.height_t = true;
        }
    }

    auto end = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed = end - start; // �����ʱ
    std::cout << "Time taken: " << elapsed.count() << " seconds" << std::endl;
    
    /////////////////////////////////////
    //����Ի���ΪԲ�ģ��뾶Ϊ50m��Բ�������ڵĽ�����ʸ����Ҫ��<=20m
    
    auto start1 = std::chrono::high_resolution_clock::now();
    filterSquaresMap(squaresMap, allBuildscale, circleRadius, dis_build, true, 0);
    auto end1 = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed1 = end1 - start1; // �����ʱ
    std::cout << "1Time taken: " << elapsed1.count() << " seconds" << std::endl;

    auto start2 = std::chrono::high_resolution_clock::now();
    //����Ի���ΪԲ�ģ��뾶Ϊ50m��Բ�������ڵĽ�����ʸ����Ҫ��<=50m
    filterSquaresMap(squaresMap, allBuildscale, circleRadius_t, dis_build_g, true, 1);
    auto end2 = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed2 = end2 - start2; // �����ʱ
    std::cout << "2Time taken: " << elapsed2.count() << " seconds" << std::endl;

    ////////////////////////////////////////
    //������վ����200m
    auto start3 = std::chrono::high_resolution_clock::now();
    filterSquaresMap(squaresMap, transSubscale, circleRadius_s, -1, true, 2);
    auto end3 = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed3 = end3 - start3; // �����ʱ
    std::cout << "3Time taken: " << elapsed3.count() << " seconds" << std::endl;

    ////////////////////////////////////////
    //������·����500m ������ٴ���300m
    auto start4 = std::chrono::high_resolution_clock::now();
    filterSquaresMap(squaresMap, railwayscale, circleRadius_g, -1, true, 3);
    auto end4 = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed4 = end4 - start4; // �����ʱ
    std::cout << "4Time taken: " << elapsed4.count() << " seconds" << std::endl;

    auto start5 = std::chrono::high_resolution_clock::now();
    filterSquaresMap(squaresMap, trainsscale, circleRadius_b, -1, true, 4);
    auto end5 = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed5 = end5 - start5; // �����ʱ
    std::cout << "5Time taken: " << elapsed5.count() << " seconds" << std::endl;

    filterSquaresMap(squaresMap, tunnelScale, circleRadius_b, -1, true, 5);//�������300m

    ////////////////////////////////////////
    //����ͨ�Ż�վ����50m
    auto start6 = std::chrono::high_resolution_clock::now();
    filterSquaresMap_t(squaresMap, pointCollectionscale, circleRadius_j);
    auto end6 = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed6 = end6 - start6; // �����ʱ
    std::cout << "6Time taken: " << elapsed6.count() << " seconds" << std::endl;

    std::map<int, std::vector<Square>> squaresMap_s = squaresMap;
    //auto start7 = std::chrono::high_resolution_clock::now();
    // 
   // processSquaresMap(squaresMap_s);
   // 
    //auto end7= std::chrono::high_resolution_clock::now(); // ������ʱ
    //std::chrono::duration<double> elapsed7 = end7 - start7; // �����ʱ
    //std::cout << "7Time taken: " << elapsed7.count() << " seconds" << std::endl;

    squaresMap_s.clear();

    auto start8 = std::chrono::high_resolution_clock::now();
    saveSquaresToJson(squaresMap_s, outputJson);
    auto end8 = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed8 = end8 - start8; // �����ʱ
    std::cout << "8Time taken: " << elapsed8.count() << " seconds" << std::endl;

    auto start9 = std::chrono::high_resolution_clock::now();
    std::map<int, std::vector<Square>> shiftedSquaresMap;
    for (const auto& pair : squaresMap_s) {
        int key = pair.first;
        const std::vector<Square>& originalSquares = pair.second;

        std::vector<Square> shiftedSquares;
        shiftedSquares.reserve(originalSquares.size());

        for (const auto& square : originalSquares) {
            Square shiftedSquare = shiftSquare(square);
            shiftedSquares.push_back(shiftedSquare);
        }
        shiftedSquaresMap[key] = shiftedSquares;
    }

    // �����µ� Shapefile
    const char* pszDriverName = "ESRI Shapefile";
    GDALDriver* poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
    if (poDriver == nullptr) {
        std::cerr << "Driver not available.\n";
        return;
    }
    // �����µ� Shapefile ���ݼ�
    const char* pszFilename = outputShp.c_str();
    GDALDataset* poDS = poDriver->Create(pszFilename, 0, 0, 0, GDT_Unknown, nullptr);
    if (poDS == nullptr) {
        std::cerr << "Creation of output file failed.\n";
        return;
    }
    // ���岢���ÿռ�ο�
    OGRSpatialReference oSRS;
    oSRS.SetWellKnownGeogCS("WGS84"); // ���ÿռ�ο�ϵͳΪ WGS84
    // ����ͼ��
    OGRLayer* poLayer = poDS->CreateLayer("squares", nullptr, wkbPolygon, nullptr);
    if (poLayer == nullptr) {
        std::cerr << "Layer creation failed.\n";
        GDALClose(poDS);
        return;
    }
    // ����ֶ�
    OGRFieldDefn oField("ID", OFTInteger);
    if (poLayer->CreateField(&oField) != OGRERR_NONE) {
        std::cerr << "Adding field failed.\n";
        GDALClose(poDS);
        return;
    }

    // ���� squaresMap �е�ÿ�� vector
    for (const auto& pair : shiftedSquaresMap) {
        const auto& squares = pair.second;
        if (!squares.empty()) { // ��� vector ��Ϊ��
            const Square& square = squares[0]; // ��ȡ vector �ĵ�һ�� Square

            // �� Square ת��Ϊ WKT ��ʽ
            std::string wkt = squareToWKT(square);
            OGRGeometry* poGeometry;
            OGRGeometryFactory::createFromWkt(wkt.c_str(), nullptr, &poGeometry);

            // ����������
            OGRFeature* poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
            poFeature->SetField("ID", pair.first); // �����ֶ�ֵΪ map �ļ�
            poFeature->SetGeometry(poGeometry);

            // ��������ӵ�ͼ��
            if (poLayer->CreateFeature(poFeature) != OGRERR_NONE) {
                std::cerr << "Failed to create feature in shapefile.\n";
            }

            // ����
            OGRFeature::DestroyFeature(poFeature);
            delete poGeometry;
        }
    }

    std::string prjFilename = pszFilename;
    size_t pos = prjFilename.rfind(".shp");
    if (pos != std::string::npos) {
        prjFilename = prjFilename.substr(0, pos); // ȥ�� .shp ��׺
    }

    auto end9 = std::chrono::high_resolution_clock::now(); // ������ʱ
    std::chrono::duration<double> elapsed9 = end9 - start9; // �����ʱ
    std::cout << "9Time taken: " << elapsed9.count() << " seconds" << std::endl;

    writePRJFile(prjFilename.c_str(), oSRS);
    GDALClose(poDS);
    return;
}

std::string to_utf8(const std::wstring& wstr) {
    int size_needed = WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), (int)wstr.size(), NULL, 0, NULL, NULL);
    std::string str(size_needed, 0);
    WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), (int)wstr.size(), &str[0], size_needed, NULL, NULL);
    return str;
}

void DroneNest::writeSquaresToPDF(const std::map<int, std::vector<Square>>& squaresMap, const std::string& filename){
    auto start8 = std::chrono::high_resolution_clock::now();

    //std::set<Point2D> uniquePixels;

    //for (const auto& entry : squaresMap) {
    //    for (const Square& square : entry.second) {
    //        Point2D centerPoint = square.center();

    //        // �������ܵ����ط�Χ
    //        for (int x = centerPoint._x - nestRadius_t; x <= centerPoint._x + nestRadius_t; ++x) {
    //            for (int y = centerPoint._y - nestRadius_t; y <= centerPoint._y + nestRadius_t; ++y) {
    //                if ((x - centerPoint._x) * (x - centerPoint._x) +
    //                    (y - centerPoint._y) * (y - centerPoint._y) <= nestRadius_t * nestRadius_t) {
    //                    uniquePixels.insert({ x, y });
    //                }
    //            }
    //        }
    //    }
    //}
    //int coverageCount = uniquePixels.size();
    //double rate = (boundaryPixelCount > 0) ? (static_cast<double>(coverageCount) / boundaryPixelCount) * 100.0 : 0.0; // �ٷֱ�
    double idx = nestRadius_t * nestRadius_t;
    int cols = test.GetCols();
    int rows = test.GetRows();
    for (const auto& entry : squaresMap) {
        const std::vector<Square>& squares = entry.second;
        const Square& square = squares[0];
        Point2D center = square.center();
        for (int y = center._y - nestRadius_t; y <= center._y + nestRadius_t; ++y) {
            for (int x = center._x - nestRadius_t; x <= center._x + nestRadius_t; ++x) {
                if (x >= 0 && x < cols && y >= 0 && y < rows) {
                    if (datass[y * cols + x] != 0) {
                        if ((x - center._x) * (x - center._x) + (y - center._y) * (y - center._y) <= idx) {
                            datass[y * cols + x] = 150;
                        }
                    }
                }
            }
        }
    }
    auto end8 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed8 = end8 - start8;
    std::cout << "8Time taken: " << elapsed8.count() << " seconds" << std::endl;

    auto start9 = std::chrono::high_resolution_clock::now();
    int count = 0;
    for (size_t i = 0; i < datass.size(); ++i) {
        if (datass[i] == 150) {
            ++count;
        }
    }
    int counts = 0;
    for (size_t i = 0; i < datass.size(); ++i) {
        if (datass[i] != test.m_pData.m_NoDataValue) {
            ++counts;
        }
    }
    double rate = (counts > 0) ? (static_cast<double>(count) / static_cast<double>(counts)) * 100.0 : 0.0;

    auto end9 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed9 = end9 - start9;
    std::cout << "9Time taken: " << elapsed9.count() << " seconds" << std::endl;

    //GDALDataset* datasetss = (GDALDataset*)GDALOpen(Coverage.c_str(), GA_Update);
    //if (datasetss == nullptr) {
    //    std::cerr << "Failed to open dataset." << std::endl;
    //    return;
    //}
    //GDALRasterBand* bandss = datasetss->GetRasterBand(1);
    //if (bandss == nullptr) {
    //    std::cerr << "Failed to get raster band." << std::endl;
    //    GDALClose(datasetss);
    //    return;
    //}
    //CPLErr errss = bandss->RasterIO(GF_Write, 0, 0, test.GetCols(), test.GetRows(), datass.data(), test.GetCols(), test.GetRows(), GDT_Byte, 0, 0);
    //if (errss != CE_None) {
    //    std::cerr << "Failed to write raster data." << std::endl;
    //}
    //GDALClose(datasetss);

    auto start10 = std::chrono::high_resolution_clock::now();
    HPDF_Doc pdf = HPDF_New(nullptr, nullptr);
    if (!pdf) {
        std::cerr << "Error: cannot create PDF object\n";
        return;
    }

    HPDF_UseCNSFonts(pdf); HPDF_UseCNTFonts(pdf); HPDF_UseCNTEncodings(pdf); HPDF_UseCNSEncodings(pdf);

    HPDF_Page page = HPDF_AddPage(pdf);
    HPDF_Page_SetSize(page, HPDF_PAGE_SIZE_A3, HPDF_PAGE_PORTRAIT);
    HPDF_REAL height = HPDF_Page_GetHeight(page);
    HPDF_REAL width = HPDF_Page_GetWidth(page);

    std::string ts = programPath + "\\file\\simhei.ttf";
    const char* font = HPDF_LoadTTFontFromFile(pdf, ts.c_str(), HPDF_TRUE);
    if (!font) {
        std::cerr << "Error: cannot load font\n";
        return;
    }

    HPDF_Page_BeginText(page);
    //HPDF_Page_SetFontAndSize(page, HPDF_GetFont(pdf, font, "UTF-8"), 10);
    HPDF_Page_SetFontAndSize(page, HPDF_GetFont(pdf, "SimSun", "GB-EUC-H"), 10);
    HPDF_Page_MoveTextPos(page, 20, 1100);

    for (const auto& pair : squaresMap) {
        int id = pair.first;
        const auto& squares = pair.second;
        if (!squares.empty()) {
            double x, y, z, a, b, c, j, k, l, v, n, m;
            const Square& square = squares[0];
            test.PixelCoorToWGS84LBH(square.topLeft._x, square.topLeft._y, x, y, z);
            test.PixelCoorToWGS84LBH(square.topRight._x, square.topRight._y, a, b, c);
            test.PixelCoorToWGS84LBH(square.bottomRight._x, square.bottomRight._y, j, k, l);
            test.PixelCoorToWGS84LBH(square.bottomLeft._x, square.bottomLeft._y, v, n, m);

            std::string text = "����id: " + std::to_string(id) + "  "
                + " (" + std::to_string(x) + ", " + std::to_string(y) + ") "
                + " (" + std::to_string(a) + ", " + std::to_string(b) + ") "
                + " (" + std::to_string(j) + ", " + std::to_string(k) + ") "
                + " (" + std::to_string(v) + ", " + std::to_string(n) + ") ";

            HPDF_Page_ShowText(page, text.c_str());
            HPDF_Page_MoveTextPos(page, 0, -15);
        }
    }
    HPDF_Page_MoveTextPos(page, 0, -10);
    std::string coverageText = "�������帲����: " + std::to_string(rate) + " %";
    HPDF_Page_ShowText(page, coverageText.c_str());

    HPDF_Page_EndText(page);
    if (HPDF_SaveToFile(pdf, filename.c_str()) != HPDF_OK) {
        //std::cout << filename << std::endl;
        std::cerr << "Error: cannot save PDF file\n";
    }
    HPDF_Free(pdf);

    auto end10 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed10 = end10 - start10;
    std::cout << "10Time taken: " << elapsed10.count() << " seconds" << std::endl;
}

void DroneNest::saveSquaresToJson(const std::map<int, std::vector<Square>>& squaresMap, const std::string& filename) {

    Json::Value root;
    root["type"] = "SquaresData";

    Json::Value squaresArray(Json::arrayValue);
    for (const auto& pair : squaresMap) {
        int id = pair.first;
        const std::vector<Square>& squares = pair.second;

        if (!squares.empty()) {
            const Square& square = squares[0];

            //Json::Value feature;
            //feature["type"] = "Feature";

            //Json::Value geometry;
            Json::Value squareData;
            squareData["type"] = "Polygon";

            Json::Value coordinates(Json::arrayValue);
            Json::Value ring(Json::arrayValue);

            double x, y, z;
            auto addCoordinate = [&](const Point2D& point) {
                test.PixelCoorToWGS84LBH(point._x, point._y, x, y, z);
                Json::Value coord(Json::arrayValue);
                coord.append(x);
                coord.append(y);
                return coord;
            };
            //auto addCoordinate = [&](const Point2D& point) {
            //    Json::Value coord(Json::arrayValue);
            //    coord.append(point._x);
            //    coord.append(point._y);
            //    return coord;
            //};

            ring.append(addCoordinate(square.bottomLeft));
            ring.append(addCoordinate(square.bottomRight));
            ring.append(addCoordinate(square.topRight));
            ring.append(addCoordinate(square.topLeft));

            ring.append(ring[0]); // �رն���λ�
            coordinates.append(ring);

            squareData["coordinates"] = coordinates;

            //feature["geometry"] = geometry;

            Json::Value properties;
            //Json::Value center(Json::arrayValue);
            //Point2D stw = square.center();
            //center.append(stw._x);
            //center.append(stw._y);
            //properties["center"] = center;
            properties["id"] = id;
            properties["height_t"] = square.height_t;
            properties["height"] = square.height;
            properties["height_difference_t"] = square.height_difference_t;
            //properties["height_difference"] = square.height_difference;

            properties["build_20"] = square.build_20;
            properties["build_50"] = square.build_50;
            properties["transformer"] = square.transformer;
            properties["railway"] = square.railway;
            properties["expressway"] = square.expressway;
            properties["communication"] = square.communication;
            properties["way"] = square.way;

            squareData["properties"] = properties;
            //features.append(feature);
            squaresArray.append(squareData);
        }
    }

    root["features"] = squaresArray;

    // �� Json::Value ����д�뵽�ļ�
    std::ofstream file(filename);
    if (file.is_open()) {
        Json::StreamWriterBuilder writerBuilder;
        writerBuilder["indentation"] = "  "; // ��ѡ������������ʽ
        std::string jsonString = Json::writeString(writerBuilder, root);
        file << jsonString;
        file.close();
        std::cout << "Data saved to " << filename << std::endl;
    }
    else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

// �������ε���Ӿ���
void calculateBoundingBox(const BuildingInfo& building, int& minX, int& maxX, int& minY, int& maxY) {
    minX = 2147483647;
    maxX = -2147483648;
    minY = 2147483647;
    maxY = -2147483648;

    for (const auto& point : building.points) {
        int x = static_cast<int>(point._x);
        int y = static_cast<int>(point._y);
        minX = custom_min(minX, x);
        maxX = custom_max(maxX, x);
        minY = custom_min(minY, y);
        maxY = custom_max(maxY, y);
    }
}
void DroneNest::setPolygonAreaToZero(GDALRasterBand* band, const BuildingInfo& building, int nXSize, int nYSize, std::vector<uint8_t> &data) {
    int minX, maxX, minY, maxY;
    calculateBoundingBox(building, minX, maxX, minY, maxY);

    // ���Ʊ�����Χ��ͼ�����Ч��Χ��
    minX = custom_max(0, minX);
    maxX = custom_min(nXSize - 1, maxX);
    minY = custom_max(0, minY);
    maxY = custom_min(nYSize - 1, maxY);

    // ����������������ֵ��Ϊ 0
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            Point2D point(x, y);
            if (isPointInPolygon(point, building)) {
                data[y * nXSize + x] = 0;
            }
        }
    }
}

// ������������Ƿ��н���
bool doRectanglesIntersect(const Square& s1, const Square& s2) {
    // ��ȡ��һ�������εı߽�
    double s1_minX = s1.bottomLeft._x;
    double s1_maxX = s1.topRight._x;
    double s1_minY = s1.bottomLeft._y;
    double s1_maxY = s1.topRight._y;

    // ��ȡ�ڶ��������εı߽�
    double s2_minX = s2.bottomLeft._x;
    double s2_maxX = s2.topRight._x;
    double s2_minY = s2.bottomLeft._y;
    double s2_maxY = s2.topRight._y;

    // ����Ƿ��н���
    if (s1_minX > s2_maxX || s2_minX > s1_maxX) return false;
    if (s1_minY > s2_maxY || s2_minY > s1_maxY) return false;

    return true;
}

std::map<int, std::vector<Square>> removeIntersectingSquares(const std::map<int, std::vector<Square>>& groundsq) {
    std::map<int, std::vector<Square>> filteredSquares;

    // �� map չƽΪ vector
    std::vector<std::pair<int, Square>> allSquares;
    for (const auto& entry : groundsq) {
        for (const auto& square : entry.second) {
            allSquares.push_back(std::make_pair(entry.first, square));
        }
    }

    std::set<int> indicesToRemove;

    // ���ÿ���������Ƿ��ཻ
    for (size_t i = 0; i < allSquares.size(); ++i) {
        if (indicesToRemove.count(allSquares[i].first)) {
            continue;
        }
        const Square& square1 = allSquares[i].second;
        for (size_t j = i + 1; j < allSquares.size(); ++j) {
            const Square& square2 = allSquares[j].second;
            if (doRectanglesIntersect(square1, square2)) {
                indicesToRemove.insert(allSquares[j].first);
            }
        }
    }

    // ���˵���Ҫ�Ƴ���������
    for (const auto& entry : groundsq) {
        std::vector<Square> validSquares;
        for (const auto& square : entry.second) {
            if (!indicesToRemove.count(entry.first)) {
                validSquares.push_back(square);
            }
        }
        if (!validSquares.empty()) {
            filteredSquares[entry.first] = validSquares;
        }
    }

    return filteredSquares;
}

void DroneNest::processGroundSquares(std::map<int, std::vector<Square>>& groundsq, const std::string& maskFilePath)
{
    //GDALAllRegister();
    //GDALDataset* dataset = (GDALDataset*)GDALOpen(maskFilePath.c_str(), GA_ReadOnly);
    //if (dataset == nullptr) {
    //    std::cerr << "Failed to open mask file." << std::endl;
    //    return;
    //}

    //GDALRasterBand* band = dataset->GetRasterBand(1);
    //int nCols = band->GetXSize();
    //int nRows = band->GetYSize();

    //// ��ȡ��ģͼ�����ݵ� std::vector<uint8_t>
    //CPLErr err = band->RasterIO(GF_Read, 0, 0, nCols, nRows, maskData.data(), nCols, nRows, GDT_Byte, 0, 0);
    //if (err != CE_None) {
    //    std::cerr << "Failed to read mask data." << std::endl;
    //    GDALClose(dataset);
    //    return;
    //}
    int nCols = test.GetCols();
    int nRows = test.GetRows();
    for (auto it = groundsq.begin(); it != groundsq.end(); ) {
        std::vector<Square>& squares = it->second;
        bool toDelete = false;

        for (const auto& square : squares) {
            int blX = static_cast<int>(square.bottomLeft._x);
            int blY = static_cast<int>(square.bottomLeft._y);
            int brX = static_cast<int>(square.bottomRight._x);
            int brY = static_cast<int>(square.bottomRight._y);
            int tlX = static_cast<int>(square.topLeft._x);
            int tlY = static_cast<int>(square.topLeft._y);
            int trX = static_cast<int>(square.topRight._x);
            int trY = static_cast<int>(square.topRight._y);

            // ����ĸ��ǵ��ֵ
            if ((maskData[blY * nCols + blX] == 0) ||
                (maskData[brY * nCols + brX] == 0) ||
                (maskData[tlY * nCols + tlX] == 0) ||
                (maskData[trY * nCols + trX] == 0)) {
                toDelete = true;
                break;
            }
        }

        // �����Ҫɾ�����Ƴ����vector
        if (toDelete) {
            it = groundsq.erase(it);
        }
        else {
            ++it;
        }
    }
    //GDALClose(dataset);
}

double calculateCircleOverlapArea(const Point2D& centerA, double radiusA, const Point2D& centerB, double radiusB) {
    double d = std::sqrt((centerA._x - centerB._x) * (centerA._x - centerB._x) +
        (centerA._y - centerB._y) * (centerA._y - centerB._y));

    // Բ��ȫ�ص�
    if (d <= std::abs(radiusA - radiusB)) {
        return M_PI * std::pow(std::min(radiusA, radiusB), 2);
    }

    // Բ��ȫ���ص�
    if (d >= radiusA + radiusB) {
        return 0.0;
    }

    // �����ص������ʹ�ù�ʽ��
    double rA = radiusA;
    double rB = radiusB;

    double alpha = std::acos((rA * rA + d * d - rB * rB) / (2 * rA * d));
    double beta = std::acos((rB * rB + d * d - rA * rA) / (2 * rB * d));

    double areaA = alpha * rA * rA;
    double areaB = beta * rB * rB;
    double areaOverlap = areaA + areaB - 0.5 * std::sqrt((-d + rA + rB) * (d + rA - rB) * (d - rA + rB) * (d + rA + rB));

    return areaOverlap;
}

std::map<int, std::vector<Square>> DroneNest::optimizeSquares(const std::map<int, std::vector<Square>>& groundsq, const std::map<int, BuildingInfo>& keyBuildingScale, double radius) {
    
    std::map<int, std::vector<Square>> closestSquares;
    for (const auto& [buildingId, building] : keyBuildingScale) {
        Point2D buildingCenter = building.center();
        std::vector<std::pair<double, Square>> distances;

        for (const auto& [groundId, squares] : groundsq) {
            for (const auto& square : squares) {
                Point2D squareCenter = square.center();
                double dist = distance(buildingCenter, squareCenter);
                distances.emplace_back(dist, square);
            }
        }

        std::sort(distances.begin(), distances.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        int count = 0;
        for (const auto& [dist, square] : distances) {
            if (dist <= radius) {
                closestSquares[buildingId].push_back(square);
                count++;
                if (count >= 2) break; // ֻ�������
            }
        }
    }
    
    std::map<int, std::vector<Square>> selectedMap; 
    for (const auto& kv : groundsq) {
        const auto& square = kv.second.front();
        Point2D center = square.center();

        // ����ѡ��¥��������
        if (square.priority == 1) {
            bool isOverlapping = false;
            for (const auto& selectedKv : selectedMap) {
                const auto& selectedSquare = selectedKv.second.front();
                if (selectedSquare.center()._x - radius < center._x && center._x < selectedSquare.center()._x + radius &&
                    selectedSquare.center()._y - radius < center._y && center._y < selectedSquare.center()._y + radius) {
                    isOverlapping = true;
                    break;
                }
            }

            // ������ص�������ӵ�ѡ�м���
            if (!isOverlapping) {
                selectedMap[kv.first].push_back(square);
            }
            continue;
        }

        // ��鵱ǰ�������Ƿ��Ѿ���ѡ�м�����
        bool isSelected = false;
        for (const auto& selectedKv : selectedMap) {
            const auto& selectedSquare = selectedKv.second.front();
            if (selectedSquare.center()._x - radius < center._x && center._x < selectedSquare.center()._x + radius &&
                selectedSquare.center()._y - radius < center._y && center._y < selectedSquare.center()._y + radius) {
                isSelected = true;
                break;
            }
        }
        // �����ǰ�����β���ѡ�м����У������
        if (!isSelected) {
            selectedMap[kv.first].push_back(square);
        }
    }
    int uniqueKeyBase = 0;
    for (const auto& [buildingId, squares] : closestSquares) {
        int currentKey = buildingId;
        while (selectedMap.find(currentKey) != selectedMap.end()) {
            currentKey = 99999 + uniqueKeyBase++;
        }
        selectedMap[currentKey].insert(selectedMap[currentKey].end(), squares.begin(), squares.end());
    }
    return selectedMap;

    //std::map<int, std::vector<Square>> selectedMap; // �洢ѡ�е�������
    //// �ȼ��ָ��������ص�
    //int ids = 99999;
    //for (const auto& kv : keyBuildingScale) {
    //    int regionId = kv.first;
    //    const BuildingInfo& building = kv.second;
    //    Point2D buildingCenter = building.center();
    //    int overlapCount = 0; // ��¼�ص�����
    //    std::vector<Square> overlappingSquares;
    //    // �������е������Σ�����뽨�����ĵ��ص�
    //    for (const auto& groundKv : groundsq) {
    //        const auto& squares = groundKv.second;

    //        // ȷ��ÿ������ֻ��һ��������
    //        if (!squares.empty()) {
    //            const Square& square = squares[0];
    //            Point2D squareCenter = square.center();

    //            // ����Ƿ��ص�
    //            if (squareCenter._x - radius < buildingCenter._x && buildingCenter._x < squareCenter._x + radius &&
    //                squareCenter._y - radius < buildingCenter._y && buildingCenter._y < squareCenter._y + radius) {
    //                overlapCount++;
    //                overlappingSquares.push_back(square);
    //            }
    //        }
    //    }
    //    // ������������������������θ��ǣ���ѡ��
    //    if (overlapCount >= 2) {
    //        int d = 0;
    //        for (auto is : overlappingSquares) {
    //            std::vector<Square> os;
    //            os.push_back(is);
    //            selectedMap[ids] = os;
    //            ++ids;
    //            ++d;
    //            if (d == 2) {
    //                break;
    //            }
    //        }
    //        overlappingSquares.clear();
    //    }
    //}
    //// ���Ŵ������������������
    //for (const auto& kv : groundsq) {
    //    int regionId = kv.first;
    //    // ����������Ѿ���ѡ�м����У�����
    //    if (selectedMap.find(regionId) != selectedMap.end()) {
    //        continue;
    //    }
    //    for (const auto& square : kv.second) {
    //        Point2D center = square.center();
    //        // ��鵱ǰ�������Ƿ��Ѿ���ѡ�м�����
    //        bool isSelected = false;
    //        for (const auto& selectedKv : selectedMap) {
    //            const auto& selectedSquare = selectedKv.second.front();
    //            if (selectedSquare.center()._x - radius < center._x && center._x < selectedSquare.center()._x + radius &&
    //                selectedSquare.center()._y - radius < center._y && center._y < selectedSquare.center()._y + radius){
    //                isSelected = true;
    //                break;
    //            }
    //        }
    //        // �����ǰ�����β���ѡ�м����У������
    //        if (!isSelected) {
    //            selectedMap[regionId].push_back(square);
    //        }
    //    }
    //}

    return selectedMap;
}

void DroneNest::screen_ground(const double threshold, const double radius, const double circleRadius, 
    const double dis_build, const double dis_build_g, const double circleRadius_t, const double circleRadius_s, 
    const double circleRadius_g, const double circleRadius_b, const double circleRadius_j, const double circleRadius_r,
    double nestRadius,
    std::string outputJson, std::string outputShp, std::string position_str, std::string outputPdf)
{
    nestRadius_t = nestRadius;
    if (!position_str.empty()) {

        std::vector<std::pair<double, double>> positions;
        std::string trimmed = position_str.substr(2, position_str.size() - 4); // ȥ������[[��]]

        std::istringstream ss(trimmed);
        std::string pair_str;

        while (std::getline(ss, pair_str, ']')) {
            if (pair_str.empty()) continue;

            // ȥ����ͷ��'['�Ϳ��ܴ��ڵĶ���
            pair_str.erase(0, pair_str.find_first_not_of("[")); // ȥ����ͷ��'['
            pair_str.erase(0, pair_str.find_first_not_of(","));
            pair_str.erase(0, pair_str.find_first_not_of("["));

            // ʹ��','�ָγ��
            std::istringstream pair_stream(pair_str);
            double longitude, latitude;
            char comma; // ���ڶ�ȡ����

            // ��ȡ���Ⱥ�γ��
            if (pair_stream >> longitude >> comma >> latitude) {
                positions.emplace_back(longitude, latitude);
            }
        }

        std::map<int, std::vector<Square>> groundsqalone;
        // ת����γ��Ϊ��������
        double a, b;
        test.WGS84LBHToPixelCoor(positions[0].first, positions[0].second, 0, a, b);
        Point2D bl(a, b);

        test.WGS84LBHToPixelCoor(positions[1].first, positions[1].second, 0, a, b);
        Point2D br(a, b);

        test.WGS84LBHToPixelCoor(positions[2].first, positions[2].second, 0, a, b);
        Point2D tl(a, b);

        test.WGS84LBHToPixelCoor(positions[3].first, positions[3].second, 0, a, b);
        Point2D tr(a, b);

        Square square(tr, tl, br, bl);
        square.build_20 = true;
        square.build_50 = true;
        square.communication = true;
        square.expressway = true;
        square.height_difference_t = true;
        square.height_t = true;
        square.railway = true;
        square.transformer = true;
        //square.way = true;
        groundsqalone[0].push_back(square);

        processSquares_s(groundsqalone, threshold);
        filterSquaresMap_s(groundsqalone, allBuildscale, circleRadius, dis_build, false, 0);
        filterSquaresMap_s(groundsqalone, allBuildscale, circleRadius_t, dis_build_g, false, 1);
        filterSquaresMap_s(groundsqalone, transSubscale, circleRadius_s, -1, false, 2);
        filterSquaresMap_s(groundsqalone, railwayscale, circleRadius_g, -1, false, 3);
        filterSquaresMap_s(groundsqalone, trainsscale, circleRadius_b, -1, false, 4);
        filterSquaresMap_s(groundsqalone, tunnelScale, circleRadius_b, -1, false, 4);
        filterSquaresMap_s(groundsqalone, roadscale_t, circleRadius_r, -1, false, 6);
        filterSquaresMap_t_s(groundsqalone, pointCollectionscale, circleRadius_j);

        saveSquaresToJson(groundsqalone, outputJson);
        return;
    }
    //auto start2 = std::chrono::high_resolution_clock::now();
    ////ȡ����ҵ��λ��Ӿ���
    //std::map<int, BuildingInfo> s;
    //std::vector<Point2D> combinedPoints = combinePoints(buildscale_t, s, notFly);// �ϲ��㼯

    //// ����ϲ��㼯��������Ӿ���
    //double minX, maxX, minY, maxY;
    //calculateBoundingBox_t(combinedPoints, minX, maxX, minY, maxY);
    //auto end2 = std::chrono::high_resolution_clock::now(); 
    //std::chrono::duration<double> elapsed2 = end2 - start2; 
    //std::cout << "2Time taken: " << elapsed2.count() << " seconds" << std::endl;

    if (!groundsq.empty()) {
        groundsq.clear();
    }
    double minX = 0;
    double maxX = test.GetCols() - radius;
    double minY = 0;
    double maxY = test.GetRows() - radius;

    auto start3 = std::chrono::high_resolution_clock::now();
    //�ҳ����а뾶Ϊ6��Բ,����ÿ��Բ��һ��������
    findValidCirclesInRange(minX, maxX, minY, maxY, radius, test.GetCols(), test.GetRows(), data1, threshold);
    auto end3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed3 = end3 - start3;
    std::cout << "3Time taken: " << elapsed3.count() << " seconds" << std::endl;

    //auto start4 = std::chrono::high_resolution_clock::now();
    //groundsq = removeIntersectingSquares(groundsq);
    //auto end4 = std::chrono::high_resolution_clock::now(); 
    //std::chrono::duration<double> elapsed4 = end4 - start4; 
    //std::cout << "4Time taken: " << elapsed4.count() << " seconds" << std::endl;

    /////////////////////////////////////
    ////����Ի���ΪԲ�ģ��뾶Ϊ50m��Բ�������ڵĽ�����ʸ����Ҫ��<=20m
    //filterSquaresMap(groundsq, allBuildscale, circleRadius, dis_build, false);
    ////����Ի���ΪԲ�ģ��뾶Ϊ200m��Բ�������ڵĽ�����ʸ����Ҫ��<=50m
    //filterSquaresMap(groundsq, allBuildscale, circleRadius_t, dis_build_g, false);
    circleRadius_l = circleRadius;
    circleRadius_t_l = circleRadius_t;
    dis_build_l = dis_build;
    dis_build_g_l = dis_build_g;
    auto start5 = std::chrono::high_resolution_clock::now();
    createTifWithBuildings(allBuildscale, test.GetCols(), test.GetRows());
    auto end5 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed5 = end5 - start5;
    std::cout << "5Time taken: " << elapsed5.count() << " seconds" << std::endl;

    auto start6 = std::chrono::high_resolution_clock::now();
    processGroundSquares(groundsq, buildScale);
    auto end6 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed6 = end6 - start6;
    std::cout << "6Time taken: " << elapsed6.count() << " seconds" << std::endl;

    auto start7 = std::chrono::high_resolution_clock::now();
    //������վ����200m
    filterSquaresMap(groundsq, transSubscale, circleRadius_s, -1, false, 2);

    //������·����500m
    filterSquaresMap(groundsq, railwayscale, circleRadius_g, -1, false, 3);

    //������ٴ���300m
    filterSquaresMap(groundsq, trainsscale, circleRadius_b, -1, false,4);
    filterSquaresMap(groundsq, tunnelScale, circleRadius_b, -1, false,5);

    //����ͨ�Ż�վ����50m
    filterSquaresMap_t(groundsq, pointCollectionscale, circleRadius_j);

    auto end7 = std::chrono::high_resolution_clock::now(); 
    std::chrono::duration<double> elapsed7 = end7 - start7; 
    std::cout << "7Time taken: " << elapsed7.count() << " seconds" << std::endl;

    //for (std::map<int, BuildingInfo>::const_iterator it = roadMap.begin(); it != roadMap.end(); ++it) {
    //    int id = it->first;
    //    const BuildingInfo& building = it->second;
    //    BuildingInfo info;
    //    info.type = it->second.type;
    //    info.height = it->second.height;
    //    std::vector<Point2D> points;
    //    double x, y;
    //    for (const auto& point : building.points) {
    //        test.WGS84LBHToPixelCoor(point._x, point._y, 0, x, y);
    //        Point2D s(x, y);
    //        points.push_back(s);
    //    }
    //    info.points = points;
    //    roadscale[id] = info;
    //}

    ////���빫·����20m
    //filterSquaresMap(groundsq, roadscale, circleRadius_r, -1, false);

    filterSquaresMap(groundsq, roadscale_t, circleRadius_r, -1, false, 6);
    //filterSquaresMap(groundsq, sportScale, 100, -1, false);
    //processSquaresMap(groundsq);

    //std::unordered_set<std::string> gridSet;
    //std::map<int, std::vector<Square>> filteredGrounds;
    //for (const auto& kv : groundsq) {
    //    const auto& squares = kv.second;
    //    for (const auto& square : squares) {
    //        Point2D center = square.center();
    //        int gridX = static_cast<int>(center._x / 2000);
    //        int gridY = static_cast<int>(center._y / 2000);

    //        // ���������Ψһ��ʶ��
    //        std::string gridKey = std::to_string(gridX) + "_" + std::to_string(gridY);

    //        // ����������û�д�������Σ��������������
    //        if (gridSet.find(gridKey) == gridSet.end()) {
    //            gridSet.insert(gridKey);
    //            filteredGrounds[kv.first].push_back(square);
    //        }
    //    }
    //}
    //groundsq = std::move(filteredGrounds);
    
    // �� squaresMap ��Ԫ�ؼ��뵽 groundsq ��
    for (auto& kv : squaresMap) {
        int key = kv.first;
        std::vector<Square>& squares = kv.second;
        while (groundsq.find(key) != groundsq.end()) {
            key++;
        }
        squares[0].priority = 1;
        groundsq[key] = squares;
    }
    groundsq = optimizeSquares(groundsq, keyBuildingScale, nestRadius + nestRadius / 4);

    //writeSquaresToPDF(groundsq, outputPdf);

    //groundsq.clear();
    //Square square1 = {  {113.94169684436703, 22.802764139077066},{113.9415797763674, 22.802764139077066}, {113.94169684436703, 22.80287205752072}, {113.9415797763674, 22.80287205752072} };
    //Square square2 = {  {113.94956989493627, 22.80625263372178},{113.9494528239397, 22.80625263372178}, {113.94956989493627, 22.80636055216543}, {113.9494528239897, 22.80636055216543} };
    //Square square3 = {  {113.95461950022589, 22.812217619963857},{113.95450242410345, 22.812217619963857}, {113.95461950022589, 22.8123255384075}, {113.95450242410345, 22.8123255384075} };
    //Square square4 = {  {113.95911394943484, 22.81147516598484},{113.95899687395053, 22.81147516598484}, {113.95911394943484, 22.811583084428484}, {113.95899687395053, 22.811583084428484} };
    //Square square5 = {  {113.9460989170848, 22.790820235493406},{113.94598185934164, 22.790820235493406}, {113.9460989170848, 22.790928153937053}, {113.94598185934164, 22.790928153937053} };
    //Square square6 = {  {113.94875958190914, 22.792132796366396},{113.94864252303921, 22.792132796366396}, {113.94875958190914, 22.792240714810042}, {113.94864252303921, 22.792240714810042} };
    //Square square7 = {  {113.95634047318478, 22.794796658734978},{113.95622341202775, 22.794796658734978}, {113.95634047318478, 22.794904577178627}, {113.95622341202775, 22.794904577178627} };
    //Square square8 = {  {113.95863482352355, 22.7969355331321},{113.95851776052993, 22.7969355331321}, {113.95863482352355, 22.79704345157575}, {113.95851776052993, 22.79704345157575} };
    //Square square9 = {  {113.96390309399753, 22.801364506791927},{113.96378602720014, 22.801364506791927}, {113.96390309399753, 22.801472425235573}, {113.96378602720014, 22.801472425235573} };
    //Square square10 = {  {113.9598692949362, 22.818124034682942},{113.95975221373658, 22.818124034682942}, {113.9598692949362, 22.81823195312659}, {113.95975221373658, 22.81823195312659} };
    //Square square11 = {  {113.93183792530722, 22.823322241278408},{113.93172083963778, 22.823322241278408}, {113.93183792530722, 22.823430159722058}, {113.93172083963778, 22.823430159722058} };
    //Square square12 = {  {113.93671015443735, 22.810025730582332},{113.93659308019866, 22.810025730582332}, {113.93671015443735, 22.810133649025982}, {113.93659308019866, 22.810133649025982} };
    //Square square13 = {  {113.94528216739637, 22.81340654839743},{113.94516509025206, 22.81340654839743}, {113.94528216739637, 22.813514466841077}, {113.94516509025206, 22.813514466841077} };
    //Square square14 = {  {113.95092378843111, 22.822051397185977},{113.95080670385457, 22.822051397185977}, {113.95092378843111, 22.822159315629627}, {113.950806703854574, 22.822159315629627} };
    //// ��ÿ�������η����Ӧ��vector
    //groundsq[0].push_back(square1);
    //groundsq[1].push_back(square2);
    //groundsq[2].push_back(square3);
    //groundsq[3].push_back(square4);
    //groundsq[4].push_back(square5);
    //groundsq[5].push_back(square6);
    //groundsq[6].push_back(square7);
    //groundsq[7].push_back(square8);
    //groundsq[8].push_back(square9);
    //groundsq[9].push_back(square10);
    //groundsq[10].push_back(square11);
    //groundsq[11].push_back(square12);
    //groundsq[12].push_back(square13);
    //groundsq[13].push_back(square14);
    saveSquaresToJson(groundsq, outputJson);

    //processSquares111(groundsq, 3200);

    std::map<int, std::vector<Square>> shiftedSquaresMap;
    for (const auto& pair : groundsq) {
        int key = pair.first;
        const std::vector<Square>& originalSquares = pair.second;
        std::vector<Square> shiftedSquares;
        shiftedSquares.reserve(originalSquares.size());

        for (const auto& square : originalSquares) {
            Square shiftedSquare = shiftSquare(square);
            shiftedSquares.push_back(shiftedSquare);
        }

        shiftedSquaresMap[key] = shiftedSquares;
    }

    // �����µ� Shapefile
    const char* pszDriverName = "ESRI Shapefile";
    GDALDriver* poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
    if (poDriver == nullptr) {
        std::cerr << "Driver not available.\n";
        return;
    }
    // �����µ� Shapefile ���ݼ�
    const char* pszFilename = outputShp.c_str();
    GDALDataset* poDS = poDriver->Create(pszFilename, 0, 0, 0, GDT_Unknown, nullptr);
    if (poDS == nullptr) {
        std::cerr << "Creation of output file failed.\n";
        return;
    }
    // ���岢���ÿռ�ο�
    OGRSpatialReference oSRS;
    oSRS.SetWellKnownGeogCS("WGS84"); // ���ÿռ�ο�ϵͳΪ WGS84
    // ����ͼ��
    OGRLayer* poLayer = poDS->CreateLayer("squares", nullptr, wkbPolygon, nullptr);
    if (poLayer == nullptr) {
        std::cerr << "Layer creation failed.\n";
        GDALClose(poDS);
        return;
    }
    // ����ֶ�
    OGRFieldDefn oField("ID", OFTInteger);
    if (poLayer->CreateField(&oField) != OGRERR_NONE) {
        std::cerr << "Adding field failed.\n";
        GDALClose(poDS);
        return;
    }

    // ���� squaresMap �е�ÿ�� vector
    for (const auto& pair : shiftedSquaresMap) {
        const auto& squares = pair.second;
        if (!squares.empty()) { // ��� vector ��Ϊ��
            const Square& square = squares[0]; // ��ȡ vector �ĵ�һ�� Square

            // �� Square ת��Ϊ WKT ��ʽ
            std::string wkt = squareToWKT(square);
            OGRGeometry* poGeometry;
            OGRGeometryFactory::createFromWkt(wkt.c_str(), nullptr, &poGeometry);

            // ����������
            OGRFeature* poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
            poFeature->SetField("ID", pair.first); // �����ֶ�ֵΪ map �ļ�
            poFeature->SetGeometry(poGeometry);

            // ��������ӵ�ͼ��
            if (poLayer->CreateFeature(poFeature) != OGRERR_NONE) {
                std::cerr << "Failed to create feature in shapefile.\n";
            }

            // ����
            OGRFeature::DestroyFeature(poFeature);
            delete poGeometry;
        }
    }
    std::string prjFilename = pszFilename;
    size_t pos = prjFilename.rfind(".shp");
    if (pos != std::string::npos) {
        prjFilename = prjFilename.substr(0, pos); // ȥ�� .shp ��׺
    }

    writePRJFile(prjFilename.c_str(), oSRS);
    GDALClose(poDS);
    return;
}

// ������Բ�ڵ�����ֵ
bool DroneNest::isCircleValid(const Point2D& center, double radius, int nXSize, int nYSize, const std::vector<uint8_t>& data, const double threshold) {
    int cx = static_cast<int>(center._x);
    int cy = static_cast<int>(center._y);

    //double minElevation = 1e308;
    //double maxElevation = -1e308;
    int y1 = static_cast<int>(cy - radius);
    int y2 = static_cast<int>(cy + radius);
    int y3 = radius / 10;

    int x1 = static_cast<int>(cx - radius);
    int x2 = static_cast<int>(cx + radius);

    for (int y = y1; y <= y2; y += y3) {
        if (y < 0 || y >= nYSize) continue; // ����ͼ��߽�

        for (int x = x1; x <= x2; x += y3) {
            if (x < 0 || x >= nXSize) continue; // ����ͼ��߽�
            //bool value = getSlopeValue(y, x, threshold, precision);
            float slopeValue = slope.at<float>(y, x);
            
            Point2D point{ static_cast<double>(x), static_cast<double>(y) };
            if (slopeValue > threshold || slopeValue < 0) {
                return false;
            }
            if (distance(center, point) <= radius) {
                if (data[y * nXSize + x] == 0) {
                    return false;
                }
            }


                //double elevation;
                //test.m_pData.GetData(x, y, &elevation);
                //if (elevation == test.m_pData.m_NoDataValue) {
                //    return false;
                //}
                //minElevation = custom_min(minElevation, elevation);
                //maxElevation = custom_max(maxElevation, elevation);
        }
    }

    //// ���̲߳��Ƿ�������Χ��
    //double elevationDifference = maxElevation - minElevation;
    //height_d = elevationDifference;
    //if (elevationDifference > threshold) {
    //    return false;
    //}
    return true;
}

// ��������ε� groundsq ��
void DroneNest::addSquareToGroundsq(std::map<int, std::vector<Square>>& groundsq, int id, const Point2D& center, double radius) {
    double diameter = 2 * radius;

    // ���������ε��ĸ��ǵ�
    Point2D bottomLeft(center._x - radius, center._y - radius);
    Point2D bottomRight(center._x + radius, center._y - radius);
    Point2D topLeft(center._x - radius, center._y + radius);
    Point2D topRight(center._x + radius, center._y + radius);

    // ���� BuildingInfo ��������ʾ������
    std::vector<Square> s;
    Square squareInfo;
    squareInfo.bottomLeft = bottomLeft;
    squareInfo.bottomRight = bottomRight;
    squareInfo.topLeft = topLeft;
    squareInfo.topRight = topRight;
    squareInfo.height_difference_t = true;
    //squareInfo.height_difference = height_d;
    squareInfo.build_20 = true;
    squareInfo.build_50 = true;
    squareInfo.communication = true;
    s.push_back(squareInfo);

    // �������δ��� groundsq ��
    groundsq[id] = s;
}

bool circlesOverlap(const Circle& c1, const Circle& c2) {
    double distance = std::sqrt(std::pow(c1.center._x - c2.center._x, 2) +
        std::pow(c1.center._y - c2.center._y, 2));
    return distance < (c1.radius + c2.radius);
}

void DroneNest::findValidCirclesInRange(double minX, double maxX, double minY, double maxY, double radius, int nXSize, int nYSize, const std::vector<uint8_t>& data, const double threshold) {
    int idCounter = 1;
    //for (int y = int(minY); y <= int(maxY); ++y) {
    //    for (int x = int(minX); x <= int(maxX); ++x) {
    //        Point2D center{ static_cast<double>(x), static_cast<double>(y) };
    //        // ȷ��Բ���ᳬ��ͼ��߽�
    //        if (center._x - radius < 0 || center._x + radius >= nXSize ||
    //            center._y - radius < 0 || center._y + radius >= nYSize) {
    //            continue;
    //        }
    //        if (isCircleValid(center, radius, nXSize, nYSize, data, threshold)) {
    //            //std::cout << "Found valid circle at (" << center._x << ", " << center._y << ") with radius " << radius << std::endl;
    //            addSquareToGroundsq(groundsq, idCounter++, center, radius);
    //            x += 40 * radius;
    //            y += 40 * radius;
    //        }
    //    }
    //}
    //int gridSize = radius; // �����С��Ϊ�뾶
    //for (int y = minY; y <= maxY; y += gridSize) {
    //    for (int x = minX; x <= maxX; x += gridSize) {
    //        for (int dy = -radius; dy <= radius; dy += radius / 2) {
    //            for (int dx = -radius; dx <= radius; dx += radius / 2) {
    //                Point2D center{ static_cast<double>(x + dx), static_cast<double>(y + dy) };
    //                if (isCircleValid(center, radius, nXSize, nYSize, data, threshold)) {
    //                    addSquareToGroundsq(groundsq, idCounter++, center, radius);
    //                }
    //            }
    //        }
    //    }
    //}
    
    int gridSize = 2 * radius; // �����С��Ϊֱ��
    std::set<Point2D> processedPoints; // �洢�Ѿ�������ĵ�
    for (int y = minY; y <= maxY; y += gridSize) {
        for (int x = minX; x <= maxX; x += gridSize) {
            Point2D center{ static_cast<double>(x), static_cast<double>(y) };
            if (isCircleValid(center, radius, nXSize, nYSize, data, threshold)) {
                addSquareToGroundsq(groundsq, idCounter++, center, radius);
                // ��¼�Ѿ�����ĵ�
                //processedPoints.insert(center);
            }
            //// �����Χ�㣬��ֻ��û�д����������½���
            //for (int dy = -radius; dy <= radius; dy += radius / 2) {
            //    for (int dx = -radius; dx <= radius; dx += radius / 2) {
            //        Point2D newCenter{ static_cast<double>(x + dx), static_cast<double>(y + dy) };
            //        if (processedPoints.find(newCenter) == processedPoints.end() &&
            //            isCircleValid(newCenter, radius, nXSize, nYSize, data, threshold)) {
            //            addSquareToGroundsq(groundsq, idCounter++, newCenter, radius);
            //            // ��¼������ĵ�
            //            processedPoints.insert(newCenter);
            //        }
            //    }
            //}
        }
    }
}

// ����㼯����Ӿ���
void DroneNest::calculateBoundingBox_t(const std::vector<Point2D>& points, double& minX, double& maxX, double& minY, double& maxY) {
    if (points.empty()) {
        minX = maxX = minY = maxY = 0;
        return;
    }

    minX = maxX = points[0]._x;
    minY = maxY = points[0]._y;

    for (const auto& point : points) {
        if (point._x < minX) minX = point._x;
        if (point._x > maxX) maxX = point._x;
        if (point._y < minY) minY = point._y;
        if (point._y > maxY) maxY = point._y;
    }
}

// �ϲ���������ĵ㼯
std::vector<Point2D> DroneNest::combinePoints(const std::map<int, BuildingInfo>& container1,
    const std::map<int, BuildingInfo>& container2,
    const std::map<int, BuildingInfo>& container3) {
    std::vector<Point2D> combinedPoints;

    auto addPointsFromContainer = [&combinedPoints](const std::map<int, BuildingInfo>& container) {
        for (const auto& kv : container) {
            const BuildingInfo& building = kv.second;
            combinedPoints.insert(combinedPoints.end(), building.points.begin(), building.points.end());
        }
    };

    addPointsFromContainer(container1);
    addPointsFromContainer(container2);
    addPointsFromContainer(container3);

    return combinedPoints;
}

void DroneNest::populateBuildingInfo(std::vector<polygon>& polygons, std::map<int, BuildingInfo>& notFly) {
    int id = 0;  
    for (const auto& poly : polygons) {
        BuildingInfo info;

        // �� polygon �ĵ�ֱ�Ӹ�ֵ�� BuildingInfo
        info.points.reserve(poly.points.size());
        for (const auto& point : poly.points) {
            info.points.push_back({ point.first, point.second });
        }
        info.type = L"Unknown";  // Ĭ������
        info.height = 0.0;       // Ĭ�ϸ߶�
        notFly[id++] = info;
    }
}

// �����Ƿ����������ڲ�
bool isPointInSquare(const Point2D& point, const Square& square) {
    return point._x >= square.bottomLeft._x && point._x <= square.bottomRight._x &&
        point._y >= square.bottomLeft._y && point._y <= square.topLeft._y;
}

// �����������е��������أ������������Сֵ�Ĳ���
bool DroneNest::isSquareValid(const Square& square, double threshold) {
    //double minValue = 1e308;
    //double maxValue = -1e308;

    double xStart = std::min(square.bottomLeft._x, square.bottomRight._x);
    double xEnd = std::max(square.bottomLeft._x, square.bottomRight._x);
    double yStart = std::min(square.bottomLeft._y, square.topLeft._y);
    double yEnd = std::max(square.bottomLeft._y, square.topLeft._y);

    for (double x = xStart; x <= xEnd; x++) {
        for (double y = yStart; y <= yEnd; y++) {
            //double value;
            //if (test.m_pData.GetData(x, y, &value)) {
            //    minValue = custom_min(minValue, value);
            //    maxValue = custom_max(maxValue, value);
            //}
            float slopeValue = slope.at<float>(y, x);
            //bool value = getSlopeValue(static_cast<int>(x), static_cast<int>(y), threshold, precision);
            if (slopeValue > threshold || slopeValue < 0) {
                return false;
            }
        }
    }
    return true;
    // ���������Сֵ�Ĳ���
    //std::cout << "Max value: " << maxValue << ", Min value: " << minValue << std::endl;

    // �жϲ����Ƿ񳬳���ֵ
    //height_d = maxValue - minValue;
    //return (maxValue - minValue) <= threshold;
}

void DroneNest::processSquares_s(std::map<int, std::vector<Square>>& squaresMap, double threshold) {

    for (std::map<int, std::vector<Square>>::iterator it = squaresMap.begin(); it != squaresMap.end(); ++it) {
        int buildingId = it->first;
        std::vector<Square>& squares = it->second;

        for (Square& square : squares) {
            bool is = isSquareValid(square, threshold);
            if (!is) {
                square.height_difference_t = true;
            }
            else {
                square.height_difference_t = false;
            }
        }
    }
}

void DroneNest::processSquares(std::map<int, std::vector<Square>>& squaresMap, double threshold) {
    std::vector<int> idsToRemove;
    for (std::map<int, std::vector<Square>>::iterator it = squaresMap.begin(); it != squaresMap.end(); ++it) {
        int buildingId = it->first;
        std::vector<Square>& squares = it->second;
        //if (buildingId == 192) {
        //    int h = 6;
        //}
        // ʹ����ʱ�������洢��Ч��������
        std::vector<Square> validSquares;
        for (Square& square : squares) {
            if (isSquareValid(square, threshold)) {
                validSquares.push_back(square);
            }
            else {
                square.height_difference_t = false;
            }
        }
        // ���û����Ч�������Σ����¼Ҫɾ���Ľ����� ID
        if (validSquares.empty()) {
            idsToRemove.push_back(buildingId);
        }
        else {
            // ���� map ֻ������Ч��������
            squaresMap[buildingId] = validSquares;
        }
    }
    // ɾ��û����Ч�����εĽ�����
    for (int id : idsToRemove) {
        squaresMap.erase(id);
    }
}

// �������ε�����
Point2D calculateCentroid(const BuildingInfo& building) {
    const auto& points = building.points;
    double sumX = 0, sumY = 0;
    int n = points.size();

    for (const auto& point : points) {
        sumX += point._x;
        sumY += point._y;
    }

    return Point2D(sumX / n, sumY / n);
}

// ���߷������Ƿ��ڶ������
bool DroneNest::isPointInPolygon(const Point2D& p, const BuildingInfo& building) {
    const auto& points = building.points;
    int n = points.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Point2D& pi = points[i];
        const Point2D& pj = points[j];
        if (((pi._y > p._y) != (pj._y > p._y)) &&
            (p._x < (pj._x - pi._x) * (p._y - pi._y) / (pj._y - pi._y) + pi._x)) {
            inside = !inside;
        }
    }
    return inside;
}

// ��������ε����е��Ƿ��ڶ������
bool DroneNest::isSquareInPolygon(const Point2D& bottomLeft, double size, const BuildingInfo& building, std::vector<Point2D>& squareCorners) {
    Point2D bottomRight(bottomLeft._x + size, bottomLeft._y);
    Point2D topLeft(bottomLeft._x, bottomLeft._y + size);
    Point2D topRight(bottomLeft._x + size, bottomLeft._y + size);

    if (isPointInPolygon(bottomLeft, building) &&
        isPointInPolygon(bottomRight, building) &&
        isPointInPolygon(topLeft, building) &&
        isPointInPolygon(topRight, building)) {

        squareCorners = { bottomLeft, bottomRight, topLeft, topRight };
        return true;
    }
    return false;
}

// Ѱ�����ĸ����������β��洢���
std::map<int, std::vector<Square>> DroneNest::findSquaresForBuildings(const std::map<int, BuildingInfo>& buildscale, double squareSize) {
    std::map<int, std::vector<Square>> squaresMap;

    for (std::map<int, BuildingInfo>::const_iterator it = buildscale.begin(); it != buildscale.end(); ++it) {
        int key = it->first;
        const BuildingInfo& building = it->second;
        Point2D centroid = calculateCentroid(building);
        double d = 30;
        double searchRadius = squareSize + d; // ���������뾶����Ӧʵ�����
        double searchRadius_t = d;

        bool foundSquare = false; // ��־λ������Ƿ��ҵ�������

        // �������ĸ�����������
        for (double y = centroid._y - searchRadius_t; y <= centroid._y + searchRadius - squareSize &&!foundSquare; y++) {
            for (double x = centroid._x - searchRadius_t; x <= centroid._x + searchRadius - squareSize && !foundSquare; x++) {
                Point2D bottomLeft(x, y);
                std::vector<Point2D> squareCorners;
                if (isSquareInPolygon(bottomLeft, squareSize, building, squareCorners)) {
                    Square square = { squareCorners[0], squareCorners[1], squareCorners[2], squareCorners[3] };
                    square.height = building.height;
                    square.height_t = true;
                    squaresMap[key].push_back(square);
                    foundSquare = true;
                }
            }
        }
    }
    return squaresMap;
}

// ����㵽��һ����ľ���
inline double DroneNest::distance(const Point2D& p1, const Point2D& p2) {
    return std::sqrt((p1._x - p2._x) * (p1._x - p2._x) + (p1._y - p2._y) * (p1._y - p2._y));
}
// ���������ε����ĵ�
Point2D calculateSquareCenter(const Square& square) {
    double centerX = (square.bottomLeft._x + square.bottomRight._x + square.topLeft._x + square.topRight._x) / 4;
    double centerY = (square.bottomLeft._y + square.bottomRight._y + square.topLeft._y + square.topRight._y) / 4;
    return Point2D(centerX, centerY);
}
// ����㵽�߶ε���̾���
double DroneNest::pointToSegmentDistance(const Point2D& point, const Point2D& segStart, const Point2D& segEnd) {
    Point2D seg = { segEnd._x - segStart._x, segEnd._y - segStart._y };
    Point2D v = { point._x - segStart._x, point._y - segStart._y };
    double segLengthSquared = seg._x * seg._x + seg._y * seg._y;
    double t = custom_max(0.0, custom_min(1.0, (v._x * seg._x + v._y * seg._y) / segLengthSquared));
    Point2D projection = { segStart._x + t * seg._x, segStart._y + t * seg._y };
    return distance(point, projection);
}
// ���Բ�Ͷ�����Ƿ��ཻ
bool DroneNest::isCircleIntersectPolygon(const Point2D& circleCenter, double radius, const BuildingInfo& building, int height, double heightbuild, bool isbuild) {
    bool intersects = false;
    if (isbuild) {
        for (size_t i = 0; i < building.points.size(); ++i) {
            Point2D p1 = building.points[i];
            Point2D p2 = building.points[(i + 1) % building.points.size()];

            // ����㵽�߶ε���̾���
            double distanceToSegment = pointToSegmentDistance(circleCenter, p1, p2);
            if (distanceToSegment <= radius) {
                if (height == -1) {
                    intersects = true;
                    break; // �ҵ�һ���߶���Բ�ཻ������������
                }
                else {
                    // ���߶�����
                    if (building.height - heightbuild >= height) {
                        intersects = true;
                        break; // �ҵ�һ�������������߶Σ�����������
                    }
                    else {
                        intersects = false; // �ҵ�һ���߶���Բ�ڵ��߶Ȳ���������
                    }
                }
            }
        }
    }
    else {
        double value;
        if (test.m_pData.GetData(circleCenter._x, circleCenter._y, &value)) {
            for (size_t i = 0; i < building.points.size(); ++i) {
                Point2D p1 = building.points[i];
                Point2D p2 = building.points[(i + 1) % building.points.size()];

                // ����㵽�߶ε���̾���
                double distanceToSegment = pointToSegmentDistance(circleCenter, p1, p2);

                if (distanceToSegment <= radius) {
                    if (height == -1) {
                        intersects = true;
                        break; // �ҵ�һ���߶���Բ�ཻ������������
                    }
                    else {
                        // ���߶�����
                        if (building.height >= height) {
                            intersects = true;
                            break; // �ҵ�һ�������������߶Σ�����������
                        }
                        else {
                            intersects = false; // �ҵ�һ���߶���Բ�ڵ��߶Ȳ���������
                        }
                    }
                }
                heightGround = value;
            }
        }
        else {
            return true;
        }
    }

    return intersects;
}

// ���� squaresMap �е����������Σ����ÿ�������ε�Բ�Ƿ��� allBuild �еĽ������ཻ�����޳��ཻ�ļ�
void DroneNest::filterSquaresMap(std::map<int, std::vector<Square>>& squaresMap,
    const std::map<int, BuildingInfo> allBuild, double circleRadius, int height, bool isbuild, int ids) {
    std::set<int> keysToRemove;
    for (auto& pair : squaresMap) {
        int buildingId = pair.first;
        std::vector<Square>& squares = pair.second;
        for (Square& square : squares) {
            Point2D center = calculateSquareCenter(square);
            bool intersects = false;
            for (const auto& buildingPair : allBuild) {
                int buildingKey = buildingPair.first;
                const BuildingInfo& building = buildingPair.second;
                if ((buildingKey == buildingId) && isbuild) {
                    continue;
                }
                BuildingInfo info;
                if (height != -1 && isbuild) {
                    info = allBuild.at(buildingId);
                }
                //if (buildingId == 18451) {
                //    int p = 5;
                //}
                if (isCircleIntersectPolygon(center, circleRadius, building, height, info.height, isbuild)) {
                    intersects = true;
                    if (ids == 0) {
                        square.build_20 = false;
                    }
                    else if (ids == 1) {
                        square.build_50 = false;
                    }
                    else if (ids == 2) {
                        square.transformer = false;
                    }
                    else if (ids == 3) {
                        square.railway = false;
                    }
                    else if (ids == 4) {
                        square.expressway = false;
                    }
                    //else if (ids == 5) {
                    //    square.railway = false;
                    //}
                    break;
                }
                else {
                    if (ids == 0) {
                        square.build_20 = true;
                    }
                    else if (ids == 1) {
                        square.build_50 = true;
                    }
                    else if (ids == 2) {
                        square.transformer = true;
                    }
                    else if (ids == 3) {
                        square.railway = true;
                    }
                    else if (ids == 4) {
                        square.expressway = true;
                    }
                }
            }
            if (intersects) {
                keysToRemove.insert(buildingId);
                break; // No need to check other squares for this buildingId
            }
            if (!isbuild) {
                square.height_t = true;
                square.height = heightGround;
            }
        }
    }
    // �޳���Ҫ�޳��ļ�
    for (int key : keysToRemove) {
        squaresMap.erase(key);
    }
}

void DroneNest::filterSquaresMap_s(std::map<int, std::vector<Square>>& squaresMap,
    const std::map<int, BuildingInfo> allBuild, double circleRadius, int height, bool isbuild, int ids) {
    std::set<int> keysToRemove;

    // �ҵ���Ҫ�޳��ļ�
    for (auto& pair : squaresMap) {
        int buildingId = pair.first;
        std::vector<Square>& squares = pair.second;

        for (Square& square : squares) {
            Point2D center = calculateSquareCenter(square);

            bool intersects = false;
            for (const auto& buildingPair : allBuild) {
                int buildingKey = buildingPair.first;
                const BuildingInfo& building = buildingPair.second;

                // ������ͬ�Ľ�����
                if ((buildingKey == buildingId) && isbuild) {
                    continue;
                }
                BuildingInfo info;
                if (height != -1 && isbuild) {
                    info = allBuild.at(buildingId);
                }
                if (isCircleIntersectPolygon(center, circleRadius, building, height, info.height, isbuild)) {
                    intersects = true;
                    if (ids == 0) {
                        square.build_20 = false;
                    }
                    else if (ids == 1) {
                        square.build_50 = false;
                    }
                    else if (ids == 2) {
                        square.transformer = false;
                    }
                    else if (ids == 3) {
                        square.railway = false;
                    }
                    else if (ids == 4) {
                        square.expressway = false;
                    }
                    break;
                }
            }
            
            if (!isbuild) {
                //square.height_t = true;
                square.height = heightGround;
            }
        }
    }
}

// ����������Ƿ���Բ�ཻ
bool isSquareIntersectCircle(const Square& square, const Point2D& circleCenter, double radius) {
    //// ȷ�������εı߽�
    //double minX = custom_min(custom_min(square.bottomLeft._x, square.bottomRight._x), custom_min(square.topLeft._x, square.topRight._x));
    //double maxX = custom_max(custom_max(square.bottomLeft._x, square.bottomRight._x), custom_max(square.topLeft._x, square.topRight._x));
    //double minY = custom_min(custom_min(square.bottomLeft._y, square.bottomRight._y), custom_min(square.topLeft._y, square.topRight._y));
    //double maxY = custom_max(custom_max(square.bottomLeft._y, square.bottomRight._y), custom_max(square.topLeft._y, square.topRight._y));

    //// ����Բ�ĵ������α߽���������
    //double closestX = custom_max(minX, custom_min(circleCenter._x, maxX));
    //double closestY = custom_max(minY, custom_min(circleCenter._y, maxY));

    //double dx = circleCenter._x - closestX;
    //double dy = circleCenter._y - closestY;

    //// ���Բ�ĵ������α߽�ľ���С�ڻ���ڰ뾶������Ϊ��������Բ�ཻ
    //return (dx * dx + dy * dy) <= (radius * radius);

    double distance = std::sqrt(std::pow(square.center()._x - circleCenter._x, 2) +
        std::pow(square.center()._y - circleCenter._y, 2));
    return distance <= radius;
}

// ���� pointCollection �е�����Բ�ĺͰ뾶Ϊ50��Բ�����ÿ��Բ�Ƿ��� squaresMap �е��������ཻ�����޳��ཻ��Ԫ��
void DroneNest::filterSquaresMap_t(std::map<int, std::vector<Square>>& squaresMap,
    const std::vector<Point2D>& pointCollection, double circleRadius) {
    std::set<int> keysToRemove;

    for (auto it = squaresMap.begin(); it != squaresMap.end(); ) {
        int buildingId = it->first;
        std::vector<Square>& squares = it->second;
        
        bool toRemove = false;
        for (Square& square : squares) {
            for (const Point2D& circleCenter : pointCollection) {
                if (isSquareIntersectCircle(square, circleCenter, circleRadius)) {
                    toRemove = true;
                    square.communication = false;
                    break;
                }
            }
            if (toRemove) {
                break;
            }
        }

        if (toRemove) {
            keysToRemove.insert(buildingId);
            it = squaresMap.erase(it); // ɾ����������һ��������
        }
        else {
            ++it; // ���û��ɾ������������
        }
    }
}

void DroneNest::filterSquaresMap_t_s(std::map<int, std::vector<Square>>& squaresMap,
    const std::vector<Point2D>& pointCollection, double circleRadius) {

    for (auto it = squaresMap.begin(); it != squaresMap.end(); ) {
        int buildingId = it->first;
        std::vector<Square>& squares = it->second;

        for (Square& square : squares) {
            for (const Point2D& circleCenter : pointCollection) {
                if (isSquareIntersectCircle(square, circleCenter, circleRadius)) {
                    square.communication = false;
                    break;
                }
            }
        }
        ++it;
    }
}

// ����һ��������е��������ص�
void DroneNest::traversePolygon(const BuildingInfo& poly)
{
    const double squareSize = 12.0; // �����εĴ�С

    const auto& points = poly.points;

    double minX = 1e308;
    double maxX = -1e308;
    double minY = 1e308;
    double maxY = -1e308;

    for (const auto& point : points) {
        minX = custom_min(minX, point._x);
        maxX = custom_max(maxX, point._x);
        minY = custom_min(minY, point._y);
        maxY = custom_max(maxY, point._y);
    }

    //for (std::map<int, BuildingInfo>::const_iterator it = buildscale.begin(); it != buildscale.end(); ++it) {
    //    int key = it->first;
    //    const BuildingInfo& building = it->second;
    //    findSquareNearCentroid(building, squareSize);
    //}



    // ������������
    for (double y = minY; y <= maxY; y++) {
        for (double x = minX; x <= maxX; x++) {
            Point2D p(x, y);
            if (isPointInPolygon(p, poly)) {
                double value;
                test.m_pData.GetData(x, y, &value);
                std::cout << value<<" ";
            }
        }
    }
}