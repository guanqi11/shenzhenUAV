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
    //std::string path = "D:\\shenzhen\\Depends\\bin\\Debug\\proj-share";//proj.db所在文件夹
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
    // 获取转换所需的宽字符长度
    int wideLen = MultiByteToWideChar(CP_UTF8, 0, utf8Str.c_str(), -1, NULL, 0);
    if (wideLen == 0) {
        return std::wstring(); // 转换失败，返回空字符串
    }

    // 分配宽字符字符串的内存
    std::wstring wideStr(wideLen - 1, 0); // -1 去除 null 终止符
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

    shpDataset->GetGeoTransform(_adf_geo_transform);  // 地理坐标信息
    OGRLayer* poLayer = (OGRLayer*)shpDataset->GetLayer(0);
    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // 找到 ID 字段的索引
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

        // 计算多边形的边界
        for (const auto& vertex : buildingInfo.points) {
            minX = custom_min(minX, vertex._x);
            maxX = custom_max(maxX, vertex._x);
            minY = custom_min(minY, vertex._y);
            maxY = custom_max(maxY, vertex._y);
        }

        int pixelCount = 0;

        // 遍历边界内的所有像素
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
    //std::string path = "C:\\Program Files\\PROJ\\share\\proj";//proj.db所在文件夹
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

	_geo_projection = shpDataset1->GetProjectionRef(); // 投影信息
	shpDataset1->GetGeoTransform(_adf_geo_transform);  // 地理坐标信息
	//_eType = GDALGetRasterDataType(shpDataset1->GetRasterBand(1));
	OGRLayer* poLayer = (OGRLayer*)shpDataset1->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // 找到 ID 字段的索引
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
    std::string type = GbkToUtf8("类型");
    std::string height = GbkToUtf8("楼高");

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
            if (type[i] >= 0x4E00 && type[i] <= 0x9FFF) { //中文字符范围
                ++charCount;
                result += type[i];
                if (charCount >= 2) {
                    break;
                }
            } 
        }
        double height = poFeature->GetFieldAsDouble(heightFieldIndex1);
        //std::wstring wideType = Utf8ToWstring(type);
        std::wstring wideString = L"变电";
        std::wstring wideString_t = L"禁飞";
        if (result == wideString) {
            OGRGeometry* poGeometry = poFeature->GetGeometryRef();
            if (poGeometry != nullptr && poGeometry->getGeometryType() == wkbPolygon) {
                BuildingInfo info;
                info.type = result;
                info.height = height;

                // 处理多边形的每个点
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

                // 处理多边形的每个点
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

    _geo_projection = shpDataset1->GetProjectionRef(); // 投影信息
    shpDataset1->GetGeoTransform(_adf_geo_transform);  // 地理坐标信息
    //_eType = GDALGetRasterDataType(shpDataset1->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset1->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // 找到 ID 字段的索引
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

    std::string type = GbkToUtf8("类型");
    std::string height = GbkToUtf8("楼高");
    std::string name = GbkToUtf8("名称");
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
    std::wstring wideString = L"重点";
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
            if (type1[i] >= 0x4E00 && type1[i] <= 0x9FFF) { // 常见的中文字符范围
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

            // 处理多边形的每个点
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

    shpDataset->GetGeoTransform(_adf_geo_transform);  // 地理坐标信息
    //_eType = GDALGetRasterDataType(shpDataset->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // 找到 ID 字段的索引
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

    std::string type = GbkToUtf8("名称");

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
            // 处理点几何
            OGRPoint* poPoint = (OGRPoint*)poGeometry;
            Point2D pt;
            pt._x = poPoint->getX();
            pt._y = poPoint->getY();
          
            double z = 0;
            if (pcvt->Transform(1, &pt._x, &pt._y, &z) != 1) {
                return 1;
            }

            pointCollection.push_back(pt);  // 将点存储到集合中
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

    shpDataset->GetGeoTransform(_adf_geo_transform);  // 地理坐标信息
    //_eType = GDALGetRasterDataType(shpDataset->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // 找到 ID 字段的索引
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

    std::string type = GbkToUtf8("类型");
    std::string type1 = GbkToUtf8("名称");

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
            if (type[i] >= 0x4E00 && type[i] <= 0x9FFF) { // 常见的中文字符范围
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
        //    if (type1[i] >= 0x4E00 && type1[i] <= 0x9FFF) { // 常见的中文字符范围
        //        ++charCount1;
        //        result1 += type1[i];
        //        if (charCount1 >= 2) {
        //            break;
        //        }
        //    }
        //}
        std::wstring wideString1 = L"公常隧道";
        std::wstring wideString = L"铁路";
        std::wstring wideString_t = L"输电";
        //std::wstring wideString_r = L"道路";
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
        else if (type1 == wideString1) {//公常隧道
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
        return path.substr(0, pos); // 返回去掉文件名的路径
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

    shpDataset->GetGeoTransform(_adf_geo_transform);  // 地理坐标信息
    //_eType = GDALGetRasterDataType(shpDataset->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // 找到 ID 字段的索引
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

    _geo_projection = shpDataset1->GetProjectionRef(); // 投影信息
    shpDataset1->GetGeoTransform(_adf_geo_transform);  // 地理坐标信息
    //_eType = GDALGetRasterDataType(shpDataset1->GetRasterBand(1));
    OGRLayer* poLayer = (OGRLayer*)shpDataset1->GetLayer(0);

    OGRFeatureDefn* featureDefn = poLayer->GetLayerDefn();
    int idFieldIndex = -1;

    // 找到 ID 字段的索引
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
    //test.Open("D:\\深圳无人机\\DOM-1M\\DOM-1M_DSM.tif","D:\\5kmapping\\SatBlockBA@20240130\\proj-share");
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
    //if (!test.Open("D:\\深圳无人机\\DOM-1M\\DOM-1M_DSM.tif", "D:\\5kmapping\\SatBlockBA@20240130\\proj-share")) {
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
    // 计算外扩矩形的边界
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

    // 外扩矩形的边界
    minX -= expansionDistance;
    minY -= expansionDistance;
    maxX += expansionDistance;
    maxY += expansionDistance;

    // 将矩形内的像素设置为0
    int startX = std::max(0, static_cast<int>(minX));
    int startY = std::max(0, static_cast<int>(minY));
    int endX = std::min(nCols - 1, static_cast<int>(maxX));
    int endY = std::min(nRows - 1, static_cast<int>(maxY));
     
    for (int y = startY; y <= endY; ++y) {
        for (int x = startX; x <= endX; ++x) {
            data[y * nCols + x] = 0; // 设置为0
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
    
    // 创建一个内存数据集
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
  
    std::vector<uint8_t> data(test.GetCols() * test.GetRows(), 255); // 所有像素值初始化为 255
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

    //    // 检查高度范围
    //    if (building.height > 20.0 && building.height < 50.0) {
    //        std::vector<Point2D> newPoints;

    //        // 外扩每个点
    //        for (size_t i = 0; i < building.points.size(); ++i) {
    //            Point2D& currentPoint = building.points[i];
    //            Point2D& nextPoint = building.points[(i + 1) % building.points.size()]; // 下一个点，循环回到第一个点

    //            // 计算外法向量并扩展
    //            Point2D normal = outwardNormal(currentPoint, nextPoint);
    //            newPoints.push_back({ currentPoint._x + normal._x * 50.0, currentPoint._y + normal._y * 50.0 });
    //        }

    //        // 更新多边形的点
    //        building.points = std::move(newPoints);
    //    }
    //}
    data1.resize(test.GetCols() * test.GetRows(), 255);
   
    //data_t = data1;
    auto start1 = std::chrono::high_resolution_clock::now();
    
    for (const auto& kv : buildscale_t) { //企事业单位
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

    for (const auto& kv : notFly) { //禁飞区
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

    //建筑物外扩掩膜
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
    datass.resize(test.GetCols() * test.GetRows(), 255); // 所有像素值初始化为 255
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

// 写入 .prj 文件以设置空间参考信息
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

// 计算正方形的中心点
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
            // 保留第一个正方形
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
    std::vector<int> indexedSquares; // 存储正方形的原始索引
    keepFirstSquareOnly(squaresMap);

    // 收集所有正方形及其索引
    for (const auto& pair : squaresMap) {
        const std::vector<Square>& squares = pair.second;
        const Square& square = squares[0];
        indexedSquares.push_back(pair.first);
        allSquares.push_back(square);
        centers.push_back(calculateCenter(square));
    }

    std::vector<std::pair<double, std::pair<int, int>>> distances;

    // 计算所有中心点之间的距离
    for (size_t i = 0; i < centers.size(); ++i) {
        for (size_t j = i + 1; j < centers.size(); ++j) {
            double dist = distance(centers[i], centers[j]);
            distances.push_back({ dist, {i, j} });
        }
    }

    // 使用冒泡排序排序 distances
    bubbleSort(distances, compareDistance);
    size_t numSquaresToDelete = 0;

    for (size_t i = 0; i < distances.size() - 1; ++i) {
        double currentDistance = distances[i].first;
        double nextDistance = distances[i + 1].first;

        // 如果后一个距离小于前一个距离的三倍
        if (nextDistance < 3 * currentDistance && currentDistance < 1000) {
            numSquaresToDelete = i + 1; // 设置为后面那个的索引
        }
    }

    //if (squaresMap.size() <= 7) {
    //    numSquaresToDelete = 2;
    //}
    //else {
    //    // 计算需要删除的正方形数量，即总正方形数量的一半
    //    numSquaresToDelete = allSquares.size() / 2 + 1;
    //}

    // 找到需要删除的正方形索引
    std::set<int> toRemove;
    for (size_t i = 0; i < numSquaresToDelete && i < distances.size(); ++i) {
        //toRemove.insert(indexedSquares[distances[i].second.first]);
        //toRemove.insert(indexedSquares[distances[i].second.second]);

        auto result = toRemove.insert(indexedSquares[distances[i].second.first]);
        // 检查是否成功插入
        if (!result.second) {
            auto result = toRemove.insert(indexedSquares[distances[i].second.second]);
        }
    }

    std::map<int, std::vector<Square>> updatedSquaresMap;

    for (const auto& pair : squaresMap) {
        const auto& squares = pair.second;
        std::vector<Square> remainingSquares;
        for (size_t i = 0; i < squares.size(); ++i) {
            // 检查原始索引是否在 toRemove 中
            if (toRemove.find(pair.first) == toRemove.end()) {
                remainingSquares.push_back(squares[i]);
            }
        }
        if (!remainingSquares.empty()) {
            updatedSquaresMap[pair.first] = remainingSquares;
        }
    }
    // 更新原始 squaresMap
    squaresMap = std::move(updatedSquaresMap);

    //for (const auto& pair : updatedSquaresMap) {
    //    int key = pair.first; // 获取当前的键
    //    if (squaresMap.find(key) != squaresMap.end()) {
    //        // 如果 squaresMap 中存在这个键，删除它
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
        std::string trimmed = position_str.substr(2, position_str.size() - 4); // 去掉外层的[[和]]

        std::istringstream ss(trimmed);
        std::string pair_str;

        while (std::getline(ss, pair_str, ']')) {
            if (pair_str.empty()) continue;

            // 去掉开头的'['和可能存在的逗号
            pair_str.erase(0, pair_str.find_first_not_of("[")); // 去掉开头的'['
            pair_str.erase(0, pair_str.find_first_not_of(","));
            pair_str.erase(0, pair_str.find_first_not_of("["));

            // 使用','分割经纬度
            std::istringstream pair_stream(pair_str);
            double longitude, latitude;
            char comma; // 用于读取逗号

            // 读取经度和纬度
            if (pair_stream >> longitude >> comma >> latitude) {
                positions.emplace_back(longitude, latitude);
            }
        }

        std::map<int, std::vector<Square>> groundsqalone;
        // 转换经纬度为像素坐标
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
    //保留低于20的建筑物
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
    //// 获取坡度数据
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
    //检查建筑物矢量轮廓中的DSM格网点（不含位于禁飞区内的点）是否为水平面
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

    auto end = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed = end - start; // 计算耗时
    std::cout << "Time taken: " << elapsed.count() << " seconds" << std::endl;
    
    /////////////////////////////////////
    //检查以机巢为圆心，半径为50m的圆形区域内的建筑物矢量，要求<=20m
    
    auto start1 = std::chrono::high_resolution_clock::now();
    filterSquaresMap(squaresMap, allBuildscale, circleRadius, dis_build, true, 0);
    auto end1 = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed1 = end1 - start1; // 计算耗时
    std::cout << "1Time taken: " << elapsed1.count() << " seconds" << std::endl;

    auto start2 = std::chrono::high_resolution_clock::now();
    //检查以机巢为圆心，半径为50m的圆形区域内的建筑物矢量，要求<=50m
    filterSquaresMap(squaresMap, allBuildscale, circleRadius_t, dis_build_g, true, 1);
    auto end2 = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed2 = end2 - start2; // 计算耗时
    std::cout << "2Time taken: " << elapsed2.count() << " seconds" << std::endl;

    ////////////////////////////////////////
    //距离变电站大于200m
    auto start3 = std::chrono::high_resolution_clock::now();
    filterSquaresMap(squaresMap, transSubscale, circleRadius_s, -1, true, 2);
    auto end3 = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed3 = end3 - start3; // 计算耗时
    std::cout << "3Time taken: " << elapsed3.count() << " seconds" << std::endl;

    ////////////////////////////////////////
    //距离铁路大于500m 距离高速大于300m
    auto start4 = std::chrono::high_resolution_clock::now();
    filterSquaresMap(squaresMap, railwayscale, circleRadius_g, -1, true, 3);
    auto end4 = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed4 = end4 - start4; // 计算耗时
    std::cout << "4Time taken: " << elapsed4.count() << " seconds" << std::endl;

    auto start5 = std::chrono::high_resolution_clock::now();
    filterSquaresMap(squaresMap, trainsscale, circleRadius_b, -1, true, 4);
    auto end5 = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed5 = end5 - start5; // 计算耗时
    std::cout << "5Time taken: " << elapsed5.count() << " seconds" << std::endl;

    filterSquaresMap(squaresMap, tunnelScale, circleRadius_b, -1, true, 5);//隧道大于300m

    ////////////////////////////////////////
    //距离通信基站大于50m
    auto start6 = std::chrono::high_resolution_clock::now();
    filterSquaresMap_t(squaresMap, pointCollectionscale, circleRadius_j);
    auto end6 = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed6 = end6 - start6; // 计算耗时
    std::cout << "6Time taken: " << elapsed6.count() << " seconds" << std::endl;

    std::map<int, std::vector<Square>> squaresMap_s = squaresMap;
    //auto start7 = std::chrono::high_resolution_clock::now();
    // 
   // processSquaresMap(squaresMap_s);
   // 
    //auto end7= std::chrono::high_resolution_clock::now(); // 结束计时
    //std::chrono::duration<double> elapsed7 = end7 - start7; // 计算耗时
    //std::cout << "7Time taken: " << elapsed7.count() << " seconds" << std::endl;

    squaresMap_s.clear();

    auto start8 = std::chrono::high_resolution_clock::now();
    saveSquaresToJson(squaresMap_s, outputJson);
    auto end8 = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed8 = end8 - start8; // 计算耗时
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

    // 创建新的 Shapefile
    const char* pszDriverName = "ESRI Shapefile";
    GDALDriver* poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
    if (poDriver == nullptr) {
        std::cerr << "Driver not available.\n";
        return;
    }
    // 创建新的 Shapefile 数据集
    const char* pszFilename = outputShp.c_str();
    GDALDataset* poDS = poDriver->Create(pszFilename, 0, 0, 0, GDT_Unknown, nullptr);
    if (poDS == nullptr) {
        std::cerr << "Creation of output file failed.\n";
        return;
    }
    // 定义并设置空间参考
    OGRSpatialReference oSRS;
    oSRS.SetWellKnownGeogCS("WGS84"); // 设置空间参考系统为 WGS84
    // 创建图层
    OGRLayer* poLayer = poDS->CreateLayer("squares", nullptr, wkbPolygon, nullptr);
    if (poLayer == nullptr) {
        std::cerr << "Layer creation failed.\n";
        GDALClose(poDS);
        return;
    }
    // 添加字段
    OGRFieldDefn oField("ID", OFTInteger);
    if (poLayer->CreateField(&oField) != OGRERR_NONE) {
        std::cerr << "Adding field failed.\n";
        GDALClose(poDS);
        return;
    }

    // 遍历 squaresMap 中的每个 vector
    for (const auto& pair : shiftedSquaresMap) {
        const auto& squares = pair.second;
        if (!squares.empty()) { // 如果 vector 不为空
            const Square& square = squares[0]; // 获取 vector 的第一个 Square

            // 将 Square 转换为 WKT 格式
            std::string wkt = squareToWKT(square);
            OGRGeometry* poGeometry;
            OGRGeometryFactory::createFromWkt(wkt.c_str(), nullptr, &poGeometry);

            // 创建新特性
            OGRFeature* poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
            poFeature->SetField("ID", pair.first); // 设置字段值为 map 的键
            poFeature->SetGeometry(poGeometry);

            // 将特性添加到图层
            if (poLayer->CreateFeature(poFeature) != OGRERR_NONE) {
                std::cerr << "Failed to create feature in shapefile.\n";
            }

            // 清理
            OGRFeature::DestroyFeature(poFeature);
            delete poGeometry;
        }
    }

    std::string prjFilename = pszFilename;
    size_t pos = prjFilename.rfind(".shp");
    if (pos != std::string::npos) {
        prjFilename = prjFilename.substr(0, pos); // 去除 .shp 后缀
    }

    auto end9 = std::chrono::high_resolution_clock::now(); // 结束计时
    std::chrono::duration<double> elapsed9 = end9 - start9; // 计算耗时
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

    //        // 遍历可能的像素范围
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
    //double rate = (boundaryPixelCount > 0) ? (static_cast<double>(coverageCount) / boundaryPixelCount) * 100.0 : 0.0; // 百分比
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

            std::string text = "机巢id: " + std::to_string(id) + "  "
                + " (" + std::to_string(x) + ", " + std::to_string(y) + ") "
                + " (" + std::to_string(a) + ", " + std::to_string(b) + ") "
                + " (" + std::to_string(j) + ", " + std::to_string(k) + ") "
                + " (" + std::to_string(v) + ", " + std::to_string(n) + ") ";

            HPDF_Page_ShowText(page, text.c_str());
            HPDF_Page_MoveTextPos(page, 0, -15);
        }
    }
    HPDF_Page_MoveTextPos(page, 0, -10);
    std::string coverageText = "机巢整体覆盖率: " + std::to_string(rate) + " %";
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

            ring.append(ring[0]); // 关闭多边形环
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

    // 将 Json::Value 对象写入到文件
    std::ofstream file(filename);
    if (file.is_open()) {
        Json::StreamWriterBuilder writerBuilder;
        writerBuilder["indentation"] = "  "; // 可选：设置缩进格式
        std::string jsonString = Json::writeString(writerBuilder, root);
        file << jsonString;
        file.close();
        std::cout << "Data saved to " << filename << std::endl;
    }
    else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

// 计算多边形的外接矩形
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

    // 限制遍历范围在图像的有效范围内
    minX = custom_max(0, minX);
    maxX = custom_min(nXSize - 1, maxX);
    minY = custom_max(0, minY);
    maxY = custom_min(nYSize - 1, maxY);

    // 将多边形区域的像素值设为 0
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            Point2D point(x, y);
            if (isPointInPolygon(point, building)) {
                data[y * nXSize + x] = 0;
            }
        }
    }
}

// 检查两个矩形是否有交集
bool doRectanglesIntersect(const Square& s1, const Square& s2) {
    // 获取第一个正方形的边界
    double s1_minX = s1.bottomLeft._x;
    double s1_maxX = s1.topRight._x;
    double s1_minY = s1.bottomLeft._y;
    double s1_maxY = s1.topRight._y;

    // 获取第二个正方形的边界
    double s2_minX = s2.bottomLeft._x;
    double s2_maxX = s2.topRight._x;
    double s2_minY = s2.bottomLeft._y;
    double s2_maxY = s2.topRight._y;

    // 检查是否有交集
    if (s1_minX > s2_maxX || s2_minX > s1_maxX) return false;
    if (s1_minY > s2_maxY || s2_minY > s1_maxY) return false;

    return true;
}

std::map<int, std::vector<Square>> removeIntersectingSquares(const std::map<int, std::vector<Square>>& groundsq) {
    std::map<int, std::vector<Square>> filteredSquares;

    // 将 map 展平为 vector
    std::vector<std::pair<int, Square>> allSquares;
    for (const auto& entry : groundsq) {
        for (const auto& square : entry.second) {
            allSquares.push_back(std::make_pair(entry.first, square));
        }
    }

    std::set<int> indicesToRemove;

    // 检查每对正方形是否相交
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

    // 过滤掉需要移除的正方形
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

    //// 读取掩模图像数据到 std::vector<uint8_t>
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

            // 检查四个角点的值
            if ((maskData[blY * nCols + blX] == 0) ||
                (maskData[brY * nCols + brX] == 0) ||
                (maskData[tlY * nCols + tlX] == 0) ||
                (maskData[trY * nCols + trX] == 0)) {
                toDelete = true;
                break;
            }
        }

        // 如果需要删除，移除这个vector
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

    // 圆完全重叠
    if (d <= std::abs(radiusA - radiusB)) {
        return M_PI * std::pow(std::min(radiusA, radiusB), 2);
    }

    // 圆完全不重叠
    if (d >= radiusA + radiusB) {
        return 0.0;
    }

    // 计算重叠面积（使用公式）
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
                if (count >= 2) break; // 只添加两个
            }
        }
    }
    
    std::map<int, std::vector<Square>> selectedMap; 
    for (const auto& kv : groundsq) {
        const auto& square = kv.second.front();
        Point2D center = square.center();

        // 优先选择楼顶正方形
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

            // 如果不重叠，则添加到选中集合
            if (!isOverlapping) {
                selectedMap[kv.first].push_back(square);
            }
            continue;
        }

        // 检查当前正方形是否已经在选中集合中
        bool isSelected = false;
        for (const auto& selectedKv : selectedMap) {
            const auto& selectedSquare = selectedKv.second.front();
            if (selectedSquare.center()._x - radius < center._x && center._x < selectedSquare.center()._x + radius &&
                selectedSquare.center()._y - radius < center._y && center._y < selectedSquare.center()._y + radius) {
                isSelected = true;
                break;
            }
        }
        // 如果当前正方形不在选中集合中，则添加
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

    //std::map<int, std::vector<Square>> selectedMap; // 存储选中的正方形
    //// 先检查指定区域的重叠
    //int ids = 99999;
    //for (const auto& kv : keyBuildingScale) {
    //    int regionId = kv.first;
    //    const BuildingInfo& building = kv.second;
    //    Point2D buildingCenter = building.center();
    //    int overlapCount = 0; // 记录重叠次数
    //    std::vector<Square> overlappingSquares;
    //    // 遍历所有的正方形，检查与建筑中心的重叠
    //    for (const auto& groundKv : groundsq) {
    //        const auto& squares = groundKv.second;

    //        // 确保每个区域只有一个正方形
    //        if (!squares.empty()) {
    //            const Square& square = squares[0];
    //            Point2D squareCenter = square.center();

    //            // 检查是否重叠
    //            if (squareCenter._x - radius < buildingCenter._x && buildingCenter._x < squareCenter._x + radius &&
    //                squareCenter._y - radius < buildingCenter._y && buildingCenter._y < squareCenter._y + radius) {
    //                overlapCount++;
    //                overlappingSquares.push_back(square);
    //            }
    //        }
    //    }
    //    // 如果建筑区域被两个或多个正方形覆盖，则选中
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
    //// 接着处理其他区域的正方形
    //for (const auto& kv : groundsq) {
    //    int regionId = kv.first;
    //    // 如果该区域已经在选中集合中，跳过
    //    if (selectedMap.find(regionId) != selectedMap.end()) {
    //        continue;
    //    }
    //    for (const auto& square : kv.second) {
    //        Point2D center = square.center();
    //        // 检查当前正方形是否已经在选中集合中
    //        bool isSelected = false;
    //        for (const auto& selectedKv : selectedMap) {
    //            const auto& selectedSquare = selectedKv.second.front();
    //            if (selectedSquare.center()._x - radius < center._x && center._x < selectedSquare.center()._x + radius &&
    //                selectedSquare.center()._y - radius < center._y && center._y < selectedSquare.center()._y + radius){
    //                isSelected = true;
    //                break;
    //            }
    //        }
    //        // 如果当前正方形不在选中集合中，则添加
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
        std::string trimmed = position_str.substr(2, position_str.size() - 4); // 去掉外层的[[和]]

        std::istringstream ss(trimmed);
        std::string pair_str;

        while (std::getline(ss, pair_str, ']')) {
            if (pair_str.empty()) continue;

            // 去掉开头的'['和可能存在的逗号
            pair_str.erase(0, pair_str.find_first_not_of("[")); // 去掉开头的'['
            pair_str.erase(0, pair_str.find_first_not_of(","));
            pair_str.erase(0, pair_str.find_first_not_of("["));

            // 使用','分割经纬度
            std::istringstream pair_stream(pair_str);
            double longitude, latitude;
            char comma; // 用于读取逗号

            // 读取经度和纬度
            if (pair_stream >> longitude >> comma >> latitude) {
                positions.emplace_back(longitude, latitude);
            }
        }

        std::map<int, std::vector<Square>> groundsqalone;
        // 转换经纬度为像素坐标
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
    ////取企事业单位外接矩形
    //std::map<int, BuildingInfo> s;
    //std::vector<Point2D> combinedPoints = combinePoints(buildscale_t, s, notFly);// 合并点集

    //// 计算合并点集的整体外接矩形
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
    //找出所有半径为6的圆,并且每个圆找一个正方形
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
    ////检查以机巢为圆心，半径为50m的圆形区域内的建筑物矢量，要求<=20m
    //filterSquaresMap(groundsq, allBuildscale, circleRadius, dis_build, false);
    ////检查以机巢为圆心，半径为200m的圆形区域内的建筑物矢量，要求<=50m
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
    //距离变电站大于200m
    filterSquaresMap(groundsq, transSubscale, circleRadius_s, -1, false, 2);

    //距离铁路大于500m
    filterSquaresMap(groundsq, railwayscale, circleRadius_g, -1, false, 3);

    //距离高速大于300m
    filterSquaresMap(groundsq, trainsscale, circleRadius_b, -1, false,4);
    filterSquaresMap(groundsq, tunnelScale, circleRadius_b, -1, false,5);

    //距离通信基站大于50m
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

    ////距离公路大于20m
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

    //        // 生成网格的唯一标识符
    //        std::string gridKey = std::to_string(gridX) + "_" + std::to_string(gridY);

    //        // 如果这个网格还没有存放正方形，则保留这个正方形
    //        if (gridSet.find(gridKey) == gridSet.end()) {
    //            gridSet.insert(gridKey);
    //            filteredGrounds[kv.first].push_back(square);
    //        }
    //    }
    //}
    //groundsq = std::move(filteredGrounds);
    
    // 将 squaresMap 的元素加入到 groundsq 中
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
    //// 将每个正方形放入对应的vector
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

    // 创建新的 Shapefile
    const char* pszDriverName = "ESRI Shapefile";
    GDALDriver* poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
    if (poDriver == nullptr) {
        std::cerr << "Driver not available.\n";
        return;
    }
    // 创建新的 Shapefile 数据集
    const char* pszFilename = outputShp.c_str();
    GDALDataset* poDS = poDriver->Create(pszFilename, 0, 0, 0, GDT_Unknown, nullptr);
    if (poDS == nullptr) {
        std::cerr << "Creation of output file failed.\n";
        return;
    }
    // 定义并设置空间参考
    OGRSpatialReference oSRS;
    oSRS.SetWellKnownGeogCS("WGS84"); // 设置空间参考系统为 WGS84
    // 创建图层
    OGRLayer* poLayer = poDS->CreateLayer("squares", nullptr, wkbPolygon, nullptr);
    if (poLayer == nullptr) {
        std::cerr << "Layer creation failed.\n";
        GDALClose(poDS);
        return;
    }
    // 添加字段
    OGRFieldDefn oField("ID", OFTInteger);
    if (poLayer->CreateField(&oField) != OGRERR_NONE) {
        std::cerr << "Adding field failed.\n";
        GDALClose(poDS);
        return;
    }

    // 遍历 squaresMap 中的每个 vector
    for (const auto& pair : shiftedSquaresMap) {
        const auto& squares = pair.second;
        if (!squares.empty()) { // 如果 vector 不为空
            const Square& square = squares[0]; // 获取 vector 的第一个 Square

            // 将 Square 转换为 WKT 格式
            std::string wkt = squareToWKT(square);
            OGRGeometry* poGeometry;
            OGRGeometryFactory::createFromWkt(wkt.c_str(), nullptr, &poGeometry);

            // 创建新特性
            OGRFeature* poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
            poFeature->SetField("ID", pair.first); // 设置字段值为 map 的键
            poFeature->SetGeometry(poGeometry);

            // 将特性添加到图层
            if (poLayer->CreateFeature(poFeature) != OGRERR_NONE) {
                std::cerr << "Failed to create feature in shapefile.\n";
            }

            // 清理
            OGRFeature::DestroyFeature(poFeature);
            delete poGeometry;
        }
    }
    std::string prjFilename = pszFilename;
    size_t pos = prjFilename.rfind(".shp");
    if (pos != std::string::npos) {
        prjFilename = prjFilename.substr(0, pos); // 去除 .shp 后缀
    }

    writePRJFile(prjFilename.c_str(), oSRS);
    GDALClose(poDS);
    return;
}

// 检查给定圆内的像素值
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
        if (y < 0 || y >= nYSize) continue; // 超出图像边界

        for (int x = x1; x <= x2; x += y3) {
            if (x < 0 || x >= nXSize) continue; // 超出图像边界
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

    //// 检查高程差是否在允许范围内
    //double elevationDifference = maxElevation - minElevation;
    //height_d = elevationDifference;
    //if (elevationDifference > threshold) {
    //    return false;
    //}
    return true;
}

// 添加正方形到 groundsq 中
void DroneNest::addSquareToGroundsq(std::map<int, std::vector<Square>>& groundsq, int id, const Point2D& center, double radius) {
    double diameter = 2 * radius;

    // 计算正方形的四个角点
    Point2D bottomLeft(center._x - radius, center._y - radius);
    Point2D bottomRight(center._x + radius, center._y - radius);
    Point2D topLeft(center._x - radius, center._y + radius);
    Point2D topRight(center._x + radius, center._y + radius);

    // 构造 BuildingInfo 对象来表示正方形
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

    // 将正方形存入 groundsq 中
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
    //        // 确保圆不会超出图像边界
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
    //int gridSize = radius; // 网格大小设为半径
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
    
    int gridSize = 2 * radius; // 网格大小设为直径
    std::set<Point2D> processedPoints; // 存储已经处理过的点
    for (int y = minY; y <= maxY; y += gridSize) {
        for (int x = minX; x <= maxX; x += gridSize) {
            Point2D center{ static_cast<double>(x), static_cast<double>(y) };
            if (isCircleValid(center, radius, nXSize, nYSize, data, threshold)) {
                addSquareToGroundsq(groundsq, idCounter++, center, radius);
                // 记录已经处理的点
                //processedPoints.insert(center);
            }
            //// 检查周围点，但只在没有处理过的情况下进行
            //for (int dy = -radius; dy <= radius; dy += radius / 2) {
            //    for (int dx = -radius; dx <= radius; dx += radius / 2) {
            //        Point2D newCenter{ static_cast<double>(x + dx), static_cast<double>(y + dy) };
            //        if (processedPoints.find(newCenter) == processedPoints.end() &&
            //            isCircleValid(newCenter, radius, nXSize, nYSize, data, threshold)) {
            //            addSquareToGroundsq(groundsq, idCounter++, newCenter, radius);
            //            // 记录处理过的点
            //            processedPoints.insert(newCenter);
            //        }
            //    }
            //}
        }
    }
}

// 计算点集的外接矩形
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

// 合并多个容器的点集
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

        // 将 polygon 的点直接赋值给 BuildingInfo
        info.points.reserve(poly.points.size());
        for (const auto& point : poly.points) {
            info.points.push_back({ point.first, point.second });
        }
        info.type = L"Unknown";  // 默认类型
        info.height = 0.0;       // 默认高度
        notFly[id++] = info;
    }
}

// 检查点是否在正方形内部
bool isPointInSquare(const Point2D& point, const Square& square) {
    return point._x >= square.bottomLeft._x && point._x <= square.bottomRight._x &&
        point._y >= square.bottomLeft._y && point._y <= square.topLeft._y;
}

// 遍历正方形中的所有像素，并检查最大和最小值的差异
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
    // 输出最大和最小值的差异
    //std::cout << "Max value: " << maxValue << ", Min value: " << minValue << std::endl;

    // 判断差异是否超出阈值
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
        // 使用临时向量来存储有效的正方形
        std::vector<Square> validSquares;
        for (Square& square : squares) {
            if (isSquareValid(square, threshold)) {
                validSquares.push_back(square);
            }
            else {
                square.height_difference_t = false;
            }
        }
        // 如果没有有效的正方形，则记录要删除的建筑物 ID
        if (validSquares.empty()) {
            idsToRemove.push_back(buildingId);
        }
        else {
            // 更新 map 只保留有效的正方形
            squaresMap[buildingId] = validSquares;
        }
    }
    // 删除没有有效正方形的建筑物
    for (int id : idsToRemove) {
        squaresMap.erase(id);
    }
}

// 计算多边形的中心
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

// 射线法检查点是否在多边形内
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

// 检查正方形的所有点是否都在多边形内
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

// 寻找中心附近的正方形并存储结果
std::map<int, std::vector<Square>> DroneNest::findSquaresForBuildings(const std::map<int, BuildingInfo>& buildscale, double squareSize) {
    std::map<int, std::vector<Square>> squaresMap;

    for (std::map<int, BuildingInfo>::const_iterator it = buildscale.begin(); it != buildscale.end(); ++it) {
        int key = it->first;
        const BuildingInfo& building = it->second;
        Point2D centroid = calculateCentroid(building);
        double d = 30;
        double searchRadius = squareSize + d; // 调整搜索半径以适应实际情况
        double searchRadius_t = d;

        bool foundSquare = false; // 标志位，检查是否找到正方形

        // 遍历中心附近的正方形
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

// 计算点到另一个点的距离
inline double DroneNest::distance(const Point2D& p1, const Point2D& p2) {
    return std::sqrt((p1._x - p2._x) * (p1._x - p2._x) + (p1._y - p2._y) * (p1._y - p2._y));
}
// 计算正方形的中心点
Point2D calculateSquareCenter(const Square& square) {
    double centerX = (square.bottomLeft._x + square.bottomRight._x + square.topLeft._x + square.topRight._x) / 4;
    double centerY = (square.bottomLeft._y + square.bottomRight._y + square.topLeft._y + square.topRight._y) / 4;
    return Point2D(centerX, centerY);
}
// 计算点到线段的最短距离
double DroneNest::pointToSegmentDistance(const Point2D& point, const Point2D& segStart, const Point2D& segEnd) {
    Point2D seg = { segEnd._x - segStart._x, segEnd._y - segStart._y };
    Point2D v = { point._x - segStart._x, point._y - segStart._y };
    double segLengthSquared = seg._x * seg._x + seg._y * seg._y;
    double t = custom_max(0.0, custom_min(1.0, (v._x * seg._x + v._y * seg._y) / segLengthSquared));
    Point2D projection = { segStart._x + t * seg._x, segStart._y + t * seg._y };
    return distance(point, projection);
}
// 检查圆和多边形是否相交
bool DroneNest::isCircleIntersectPolygon(const Point2D& circleCenter, double radius, const BuildingInfo& building, int height, double heightbuild, bool isbuild) {
    bool intersects = false;
    if (isbuild) {
        for (size_t i = 0; i < building.points.size(); ++i) {
            Point2D p1 = building.points[i];
            Point2D p2 = building.points[(i + 1) % building.points.size()];

            // 计算点到线段的最短距离
            double distanceToSegment = pointToSegmentDistance(circleCenter, p1, p2);
            if (distanceToSegment <= radius) {
                if (height == -1) {
                    intersects = true;
                    break; // 找到一个线段与圆相交，无需继续检查
                }
                else {
                    // 检查高度条件
                    if (building.height - heightbuild >= height) {
                        intersects = true;
                        break; // 找到一个符合条件的线段，无需继续检查
                    }
                    else {
                        intersects = false; // 找到一个线段在圆内但高度不符合条件
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

                // 计算点到线段的最短距离
                double distanceToSegment = pointToSegmentDistance(circleCenter, p1, p2);

                if (distanceToSegment <= radius) {
                    if (height == -1) {
                        intersects = true;
                        break; // 找到一个线段与圆相交，无需继续检查
                    }
                    else {
                        // 检查高度条件
                        if (building.height >= height) {
                            intersects = true;
                            break; // 找到一个符合条件的线段，无需继续检查
                        }
                        else {
                            intersects = false; // 找到一个线段在圆内但高度不符合条件
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

// 遍历 squaresMap 中的所有正方形，检查每个正方形的圆是否与 allBuild 中的建筑物相交，并剔除相交的键
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
    // 剔除需要剔除的键
    for (int key : keysToRemove) {
        squaresMap.erase(key);
    }
}

void DroneNest::filterSquaresMap_s(std::map<int, std::vector<Square>>& squaresMap,
    const std::map<int, BuildingInfo> allBuild, double circleRadius, int height, bool isbuild, int ids) {
    std::set<int> keysToRemove;

    // 找到需要剔除的键
    for (auto& pair : squaresMap) {
        int buildingId = pair.first;
        std::vector<Square>& squares = pair.second;

        for (Square& square : squares) {
            Point2D center = calculateSquareCenter(square);

            bool intersects = false;
            for (const auto& buildingPair : allBuild) {
                int buildingKey = buildingPair.first;
                const BuildingInfo& building = buildingPair.second;

                // 跳过相同的建筑物
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

// 检查正方形是否与圆相交
bool isSquareIntersectCircle(const Square& square, const Point2D& circleCenter, double radius) {
    //// 确定正方形的边界
    //double minX = custom_min(custom_min(square.bottomLeft._x, square.bottomRight._x), custom_min(square.topLeft._x, square.topRight._x));
    //double maxX = custom_max(custom_max(square.bottomLeft._x, square.bottomRight._x), custom_max(square.topLeft._x, square.topRight._x));
    //double minY = custom_min(custom_min(square.bottomLeft._y, square.bottomRight._y), custom_min(square.topLeft._y, square.topRight._y));
    //double maxY = custom_max(custom_max(square.bottomLeft._y, square.bottomRight._y), custom_max(square.topLeft._y, square.topRight._y));

    //// 计算圆心到正方形边界的最近距离
    //double closestX = custom_max(minX, custom_min(circleCenter._x, maxX));
    //double closestY = custom_max(minY, custom_min(circleCenter._y, maxY));

    //double dx = circleCenter._x - closestX;
    //double dy = circleCenter._y - closestY;

    //// 如果圆心到正方形边界的距离小于或等于半径，则认为正方形与圆相交
    //return (dx * dx + dy * dy) <= (radius * radius);

    double distance = std::sqrt(std::pow(square.center()._x - circleCenter._x, 2) +
        std::pow(square.center()._y - circleCenter._y, 2));
    return distance <= radius;
}

// 遍历 pointCollection 中的所有圆心和半径为50的圆，检查每个圆是否与 squaresMap 中的正方形相交，并剔除相交的元素
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
            it = squaresMap.erase(it); // 删除并返回下一个迭代器
        }
        else {
            ++it; // 如果没有删除，继续迭代
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

// 遍历一个多边形中的所有像素点
void DroneNest::traversePolygon(const BuildingInfo& poly)
{
    const double squareSize = 12.0; // 正方形的大小

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



    // 遍历所有像素
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