#ifndef DRONE_NEST_H
#define DRONE_NEST_H

#include "gdal_priv.h"
#include "ogrsf_frmts.h"
#include <string>
#include <vector>
#include <set>
#include <windows.h>
#include <json/json.h>
#include "RasterData.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

#define ACCEPT USE OF DEPRECATED PROJ API H
#include <proj_api.h>

struct PointComparator {
	bool operator()(const cv::Point& p1, const cv::Point& p2) const {
		return p1.y < p2.y || (p1.y == p2.y && p1.x < p2.x);
	}
};

template<typename T>
T custom_max(const T& a, const T& b) {
	return (a > b) ? a : b;
}

template<typename T>
T custom_min(const T& a, const T& b) {
	return (a < b) ? a : b;
}

// 冒泡排序
template <typename T>
void bubbleSort(std::vector<T>& arr, bool (*comp)(const T&, const T&)) {
	bool swapped;
	for (size_t i = 0; i < arr.size(); ++i) {
		swapped = false;
		for (size_t j = 0; j < arr.size() - 1 - i; ++j) {
			if (comp(arr[j + 1], arr[j])) {
				std::swap(arr[j], arr[j + 1]);
				swapped = true;
			}
		}
		if (!swapped) {
			break; // 提前退出
		}
	}
}

// 自定义 min_element 实现
template<typename ForwardIterator, typename Compare>
ForwardIterator custom_min_element(ForwardIterator first, ForwardIterator last, Compare comp) {
	if (first == last) return last;

	ForwardIterator smallest = first;
	++first;

	for (; first != last; ++first) {
		if (comp(*first, *smallest)) {
			smallest = first;
		}
	}

	return smallest;
}

struct Point2D {
public:
	Point2D(double x = 0, double y = 0) {
		_x = x; _y = y;
	}
	~Point2D() {}

	double _x = 0, _y = 0;

	// 添加相等比较运算符
	bool operator==(const Point2D& other) const {
		return _x == other._x && _y == other._y;
	}

	// 定义比较运算符
	bool operator<(const Point2D& other) const {
		if (_x == other._x) {
			return _y < other._y; // 按 y 值比较
		}
		return _x < other._x; // 按 x 值比较
	}
};

struct Circle {
	Point2D center;
	double radius;
};

struct Point2DHash {
	size_t operator()(const Point2D& p) const {
		return std::hash<double>()(p._x) ^ (std::hash<double>()(p._y) << 1);
	}
};

struct parameterDistance {//（根据收到参数作调整）
	double height_difference; //高度差
	double build_20;          //距离最近20m建筑物的距离
	double build_50;          //距离最近50m建筑物的距离
	double transformer;       //距离最近变电站 变压器 高压线的距离
	double railway;           //距离最近铁路的距离
	double expressway;        //距离最近高速公路的距离
	double communication;     //距离最近通信基站的距离
};

//保存正方形的四个角的坐标
struct Square {
	Point2D bottomLeft;
	Point2D bottomRight;
	Point2D topLeft;
	Point2D topRight;

	bool height_t;
	double height; //高度

	bool height_difference_t;
	//double height_difference; //高度差

	bool build_20;          //距离最近20m建筑物的距离
	bool build_50;          //距离最近50m建筑物的距离
	bool transformer;       //距离最近变电站 变压器 高压线的距离
	bool railway;           //距离最近铁路的距离
	bool expressway;        //距离最近高速公路的距离
	bool communication;     //距离最近通信基站的距离
	bool way;

	int priority;

	Point2D center() const {
		// 计算正方形的中点
		double minX = std::min({ bottomLeft._x, bottomRight._x, topLeft._x, topRight._x });
		double maxX = std::max({ bottomLeft._x, bottomRight._x, topLeft._x, topRight._x });
		double minY = std::min({ bottomLeft._y, bottomRight._y, topLeft._y, topRight._y });
		double maxY = std::max({ bottomLeft._y, bottomRight._y, topLeft._y, topRight._y });

		return { (minX + maxX) / 2.0, (minY + maxY) / 2.0 };
		//return { (bottomLeft._x + topRight._x) / 2.0, (bottomLeft._y + topRight._y) / 2.0 };
	}

	// 获取正方形中点为圆心，半径为 r 的圆
	Circle toCircle(double radius) const {
		return { center(), radius };
	}

	// 默认构造函数
	Square() = default;

	// 带参数的构造函数
	Square(const Point2D & bl, const Point2D & br, const Point2D & tl, const Point2D & tr)
		: bottomLeft(bl), bottomRight(br), topLeft(tl), topRight(tr) {}

	// 确保拷贝构造函数存在
	Square(const Square&) = default;
	Square& operator=(const Square&) = default;
};

// 定义点类型
using Point = std::pair<double, double>;

// 定义多边形类型
struct polygon {
	std::vector<Point> points;
};

struct BuildingInfo {
	std::wstring type;   // 类型字段
	double height;      // 楼高字段
	std::vector<Point2D> points;

	Point2D center() const {
		double sumX = 0.0;
		double sumY = 0.0;

		for (const auto& point : points) {
			sumX += point._x;
			sumY += point._y;
		}

		return { sumX / points.size(), sumY / points.size() };
	}
};

struct RoadInfo {
	std::wstring type;   // 名称字段
	std::vector<Point2D> points;
};

class DroneNest {
public:
    DroneNest();
    ~DroneNest();

	bool openBoundaryFile(std::string boundaryFile);
	bool openShpFile(std::string buildFile);
	bool openShpFile_allBuild(std::string buildFile);
	bool openShpFile_point(std::string pointFile);
	bool openShpFile_road(std::string lineFile);
	
	bool readDsm(std::string dsmFile, std::string projshare);
	bool readJsonFile(std::string json_notfly);

	bool openRoadShpFile(std::string roadFile);
	bool openSportShpFile(std::string sportFile);
	void screen_build(const double squareSize, const double threshold, const double circleRadius, const double dis_build, 
		const double dis_build_g, const double circleRadius_t, const double circleRadius_s, const double circleRadius_g, 
		const double circleRadius_b, const double circleRadius_j, double heightStandard, double nestRadius,std::string outputJson, std::string outputShp,
		std::string position_str);

	void screen_ground(const double threshold, const double radius, const double circleRadius,
		const double dis_build, const double dis_build_g,const double circleRadius_t, const double circleRadius_s, 
		const double circleRadius_g,const double circleRadius_b, const double circleRadius_j, 
		const double circleRadius_r, double nestRadius, std::string outputJson, std::string outputShp,
		std::string position_str, std::string outputPdf);

	void calculateBuildingPixels(const std::map<int, BuildingInfo>& boundaryScale);

	void traversePolygon(const BuildingInfo& poly);
	void addSquareToGroundsq(std::map<int, std::vector<Square>>& groundsq, int id, const Point2D& center, double radius);

	Point2D CalculatePolygonCentroid(const std::vector<Point2D>& points);//几何重心计算多边形中心点

	std::map<int, std::vector<Square>> findSquaresForBuildings(const std::map<int, BuildingInfo>& buildscale, double squareSize);
	bool isSquareValid(const Square& square, double threshold);
	void processSquares(std::map<int, std::vector<Square>>& squaresMap, double threshold);
	void processSquares_s(std::map<int, std::vector<Square>>& squaresMap, double threshold);
	void filterSquaresMap(std::map<int, std::vector<Square>>& squaresMap,
		const std::map<int, BuildingInfo> allBuild, double circleRadius, int height, bool isbuild, int ids);
	void filterSquaresMap_s(std::map<int, std::vector<Square>>& squaresMap,
		const std::map<int, BuildingInfo> allBuild, double circleRadius, int height, bool isbuild, int ids);
	void filterSquaresMap_t(std::map<int, std::vector<Square>>& squaresMap,
		const std::vector<Point2D>& pointCollection, double circleRadius);
	void filterSquaresMap_t_s(std::map<int, std::vector<Square>>& squaresMap,
		const std::vector<Point2D>& pointCollection, double circleRadius);
	bool isPointInPolygon(const Point2D& p, const BuildingInfo& building);
	void setPolygonAreaToZero(GDALRasterBand* band, const BuildingInfo& building, int nXSize, int nYSize, std::vector<uint8_t> &data);
	bool isSquareInPolygon(const Point2D& bottomLeft, double size, const BuildingInfo& building, std::vector<Point2D>& squareCorners);
	void populateBuildingInfo(std::vector<polygon>& polygons, std::map<int, BuildingInfo>& notFly);
	void findValidCirclesInRange(double minX, double maxX, double minY, double maxY, double radius, int nXSize, int nYSize, const std::vector<uint8_t>& data, const double threshold);
	std::vector<Point2D> combinePoints(const std::map<int, BuildingInfo>& container1,
		const std::map<int, BuildingInfo>& container2,
		const std::map<int, BuildingInfo>& container3);
	inline double distance(const Point2D& p1, const Point2D& p2);
	void calculateBoundingBox_t(const std::vector<Point2D>& points, double& minX, double& maxX, double& minY, double& maxY);
	bool isCircleValid(const Point2D& center, double radius, int nXSize, int nYSize, const std::vector<uint8_t>& data, const double threshold);
	bool isCircleIntersectPolygon(const Point2D& circleCenter, double radius, const BuildingInfo& building, int height, double heightbuild, bool isbuild);
	double pointToSegmentDistance(const Point2D& point, const Point2D& segStart, const Point2D& segEnd);
	void processSquares111(std::map<int, std::vector<Square>>& shiftedSquaresMap, double g);

	Square shiftSquare(const Square& square);
	void processSquaresMap(std::map<int, std::vector<Square>>& squaresMap);
	void saveSquaresToJson(const std::map<int, std::vector<Square>>& squaresMap, const std::string& filename);
	void groundPreprocessing(std::string slopeFile);
	void buildPreprocessing();

	void createTifWithBuildings(std::map<int, BuildingInfo>& allBuildscale, int slopeWidth, int slopeHeight);
	void processBuildings(std::map<int, BuildingInfo>& allBuildscale, std::vector<uint8_t>& data);
	void expandPolygon(const std::vector<Point2D>& points, double expansionDistance, std::vector<uint8_t>& data, int nCols, int nRows);
	void processGroundSquares(std::map<int, std::vector<Square>>& groundsq, const std::string& maskFilePath);
	std::map<int, std::vector<Square>> optimizeSquares(const std::map<int, std::vector<Square>>& groundsq, const std::map<int, BuildingInfo>& keyBuildingScale, double radius);
	void writeSquaresToPDF(const std::map<int, std::vector<Square>>& squaresMap, const std::string& filename);

	std::vector<float> slopeData; 
	//int slopeWidth, slopeHeight;
	//bool getSlopeValue(int x, int y, float threshold, double precisions) {
	//	if ((x * precisions) < 0 || (x * precisions) >= slopeWidth || (y * precisions) < 0 || (y * precisions) >= slopeHeight) {
	//		return false;
	//	}
	//	float slopeValue = slopeData[y * precisions * slopeWidth + x * precisions];
	//	//std::cout << slopeValue <<" ";
	//	return (slopeValue <= threshold && slopeValue >= 0);
	//}
private:
	double circleRadius_l;
	double circleRadius_t_l; 
	double dis_build_l; 
	double dis_build_g_l;
	double nestRadius_t;
	double boundaryPixelCount;
public:
	std::string programPath;
	double precision;
	std::string projPath;
	std::map<int, std::vector<Square>> groundsq;
private:
	std::vector<uint8_t> datass;
	std::string Coverage;
	cv::Mat slope;
	std::vector<uint8_t> maskData;
	GDALRasterBand* band_buildScale = nullptr;
	GDALDataset* dataset_buildScale = nullptr;
	std::string buildScale;
	std::unordered_map<int, parameterDistance> container_return;
	double height_d;
	double heightGround;

	CCoordinateSystemConvert* pcvt;

	CDEM test;
	size_t _cols;
	size_t	_rows;
	size_t _band;
	std::string _geo_projection = "";
	double _adf_geo_transform[6]{};
	GDALDataType _eType;

	std::map<int, BuildingInfo> keyBuildingMap;
	std::map<int, BuildingInfo> keyBuildingScale;

	std::map<int, BuildingInfo> buildingMap;//建筑物
	std::map<int, BuildingInfo> buildingMap_t;

	std::map<int, BuildingInfo> allBuild;
	std::map<int, BuildingInfo> allBuildscale;

	std::vector<Point2D> pointCollection;
	std::vector<Point2D> pointCollectionscale;

	std::map<int, BuildingInfo> tunnelMap;
	std::map<int, BuildingInfo> tunnelScale;

	std::map<int, BuildingInfo> roadMap;//公路
	std::map<int, BuildingInfo> roadscale;

	std::map<int, BuildingInfo> railwayMap;//铁路
	std::map<int, BuildingInfo> railwayscale;

	std::map<int, BuildingInfo> trainsMap;//输电
	std::map<int, BuildingInfo> trainsscale;


	std::vector<polygon> polygons;//禁飞区 wgs84经纬度
	std::vector<polygon> polygons_t;//禁飞区 2000经纬度
	std::map<int, BuildingInfo> notFly;

	std::map<int, BuildingInfo> transSubMap;//变电站
	std::map<int, BuildingInfo> transSubscale;

	std::map<int, BuildingInfo> buildscale;
	std::map<int, BuildingInfo> buildscale_t;

	//std::map<int, BuildingInfo> groundsq;//地面上满足高程差小于指定值的正方形

	

	std::map<int, BuildingInfo> roadMap_t;//所有道路，除了铁路高速
	std::map<int, BuildingInfo> roadscale_t;
	std::map<int, BuildingInfo> boundaryMap;//边界矢量
	std::map<int, BuildingInfo> boundaryScale;

	std::map<int, BuildingInfo> sportMap;//运动场
	std::map<int, BuildingInfo> sportScale;

	std::vector<uint8_t> data2;
	std::vector<uint8_t> data_t;
	std::vector<uint8_t> data1;
	std::vector<uint8_t> datas;
	std::map<int, std::vector<Square>> squaresMap;
};

#endif // DRONE_NEST_H
