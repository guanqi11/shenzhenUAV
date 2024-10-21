#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>   
#include <vector>   
#include <iostream>
#include <filesystem>
#include <curl/curl.h>
#include <crow.h>

#include "DroneNest.h"

std::string convertWideToMultiByte_t(const std::wstring& wideStr) {
	int size_needed = WideCharToMultiByte(CP_ACP, 0, &wideStr[0], (int)wideStr.size(), NULL, 0, NULL, NULL);
	std::vector<char> str(size_needed, 0);
	WideCharToMultiByte(CP_ACP, 0, &wideStr[0], (int)wideStr.size(), &str[0], size_needed, NULL, NULL);
	return std::string(&str[0], size_needed);
}

std::wstring removeFileName_t(const std::wstring& path) {
	std::wstring::size_type pos = path.find_last_of(L"\\/");
	if (pos != std::wstring::npos) {
		return path.substr(0, pos);
	}
	return path;
}

// 回调函数处理响应数据
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
	((std::string*)userp)->append((char*)contents, size * nmemb);
	return size * nmemb;
}

void handleRequest(double precision_l, double squareSize_build_l, double squareSize_ground_l,double threshold_l,
	double threshold_ground_l, double circleRadius_l, double dis_build_l,
	double circleRadius_t_l, double dis_build_g_l, double circleRadius_s_l,
	double circleRadius_g_l, double circleRadius_b_l, double circleRadius_j_l,
	double circleRadius_r_l, double heightStandard_l, double nestRadius_l, DroneNest& nest,
	std::string outputJson_build, std::string outputShp_build, std::string outputJson_ground, std::string outputShp_ground,
	std::string outputPdf, std::string position_str, bool isbuild) {

	double squareSize_build = squareSize_build_l / precision_l;
	double squareSize_ground = squareSize_ground_l / precision_l;
	double radius_build = squareSize_build / 2;
	double radius_ground = squareSize_ground / 2;

	double threshold = threshold_l;
	double threshold_ground = threshold_ground_l;
	double circleRadius = circleRadius_l / precision_l;
	double dis_build = dis_build_l;
	double dis_build_g = dis_build_g_l;
	double circleRadius_t = circleRadius_t_l / precision_l;
	double circleRadius_s = circleRadius_s_l / precision_l;
	double circleRadius_g = circleRadius_g_l / precision_l;
	double circleRadius_b = circleRadius_b_l / precision_l;
	double circleRadius_j = circleRadius_j_l / precision_l;
	double circleRadius_r = circleRadius_r_l / precision_l;
	double heightStandard = heightStandard_l;
	double nestRadius = nestRadius_l / precision_l;
	nest.precision = precision_l;
	if (position_str.empty()) {
		nest.screen_build(squareSize_build, threshold, circleRadius, dis_build, dis_build_g, circleRadius_t, circleRadius_s, circleRadius_g,
			circleRadius_b, circleRadius_j, heightStandard, nestRadius,outputJson_build, outputShp_build, position_str);

		nest.screen_ground(threshold_ground, radius_ground, circleRadius, dis_build, dis_build_g, circleRadius_t, circleRadius_s, 
			circleRadius_g, circleRadius_b, circleRadius_j, circleRadius_r, nestRadius,outputJson_ground, outputShp_ground, position_str, outputPdf);
	}
	else {
		if (isbuild) {
			nest.screen_build(squareSize_build, threshold, circleRadius, dis_build, dis_build_g, circleRadius_t, circleRadius_s, circleRadius_g,
				circleRadius_b, circleRadius_j, heightStandard, nestRadius, outputJson_build, outputShp_build, position_str);
		}
		else {
			nest.screen_ground(threshold_ground, radius_ground, circleRadius, dis_build, dis_build_g, circleRadius_t, circleRadius_s,
				circleRadius_g, circleRadius_b, circleRadius_j, circleRadius_r, nestRadius, outputJson_ground, outputShp_ground, position_str, outputPdf);
		}
	}
}

//std::string jsonFile1Path = "D:\\shenzhen\\droneNest_build.json";  // 第一个 JSON 文件路径 建筑物
//std::string jsonFile2Path = "D:\\shenzhen\\droneNest_ground.json";  // 第二个 JSON 文件路径 地面

// 读取两个 JSON 文件并合并内容
crow::response JsonRequest(std::string jsonFile1Path, std::string jsonFile2Path) {
	Json::Value mergedData;
	mergedData["status"] = "success";
	mergedData["data"] = Json::Value(Json::objectValue);

	// 读取第一个 JSON 文件
	{
		std::ifstream jsonFile1(jsonFile1Path);
		if (!jsonFile1.is_open()) {
			return crow::response(404, "File1 not found");
		}

		Json::Value jsonData1;
		Json::CharReaderBuilder readerBuilder;
		std::string errs;
	
		if (!Json::parseFromStream(readerBuilder, jsonFile1, &jsonData1, &errs)) {
			return crow::response(500, "Failed to parse JSON file1: " + errs);
		}

		mergedData["data"]["build"] = jsonData1;
	}

	// 读取第二个 JSON 文件
	{
		std::ifstream jsonFile2(jsonFile2Path);
		if (!jsonFile2.is_open()) {
			return crow::response(404, "File2 not found");
		}

		Json::Value jsonData2;
		Json::CharReaderBuilder readerBuilder;
		std::string errs;

		if (!Json::parseFromStream(readerBuilder, jsonFile2, &jsonData2, &errs)) {
			return crow::response(500, "Failed to parse JSON file2: " + errs);
		}

		mergedData["data"]["ground"] = jsonData2;
	}

	// 将合并后的 JSON 对象转换为字符串
	Json::StreamWriterBuilder writerBuilder;
	std::string jsonString = Json::writeString(writerBuilder, mergedData);

	// 创建响应并设置 Content-Type
	crow::response res;
	res.code = 200; // HTTP 状态码
	res.set_header("content-type", "application/json");
	res.body = jsonString; // 响应内容

	return res; // 返回响应
}

crow::response JsonRequest_s(bool isbuild, std::string jsonFile1Path, std::string jsonFile2Path) {
	auto start3 = std::chrono::high_resolution_clock::now();

	Json::Value mergedData;
	mergedData["status"] = "success";
	mergedData["data"] = Json::Value(Json::objectValue);

	std::string jsonFilePath = isbuild ? jsonFile1Path : jsonFile2Path;

	// 读取指定的 JSON 文件
	std::ifstream jsonFile(jsonFilePath);
	if (!jsonFile.is_open()) {
		return crow::response(404, "File not found: " + jsonFilePath);
	}

	Json::Value jsonData;
	Json::CharReaderBuilder readerBuilder;
	std::string errs;

	if (!Json::parseFromStream(readerBuilder, jsonFile, &jsonData, &errs)) {
		return crow::response(500, "Failed to parse JSON file: " + errs);
	}

	// 将读取的 JSON 数据添加到合并的 JSON 对象中
	if (isbuild) {
		mergedData["data"]["build"] = jsonData;
	}
	else {
		mergedData["data"]["ground"] = jsonData;
	}

	// 将合并后的 JSON 对象转换为字符串
	Json::StreamWriterBuilder writerBuilder;
	std::string jsonString = Json::writeString(writerBuilder, mergedData);

	// 创建响应并设置 Content-Type
	crow::response res;
	res.code = 200; // HTTP 状态码
	res.set_header("content-type", "application/json");
	res.body = jsonString; // 响应内容

	auto end3 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed3 = end3 - start3;
	std::cout << "2Time taken: " << elapsed3.count() << " seconds" << std::endl;

	return res; // 返回响应
}

void preprocessing(DroneNest& nest, std::string& outputJson_build,std::string& outputShp_build,std::string& outputJson_ground,std::string& outputShp_ground, std::string& outputPdf)
{
	SetConsoleOutputCP(CP_UTF8);

	std::string ansiPath;
	std::string s;
	TCHAR path[MAX_PATH];
	if (GetModuleFileName(NULL, path, MAX_PATH) != 0) {
		std::wstring widePath(path);
		std::wstring directory = removeFileName_t(widePath);
		ansiPath = convertWideToMultiByte_t(directory);
		s = ansiPath;
		std::filesystem::path path(s);
		std::filesystem::path parentPath = path.parent_path();
		s = convertWideToMultiByte_t(parentPath.wstring());
	}
	else {
		return;
	}
	nest.programPath = s;
	ansiPath = ansiPath + "\\config\\DroneNestConfig.json";

	// 打开配置文件
	std::ifstream configFile(ansiPath.c_str());
	if (!configFile.is_open()) {
		std::cout << "Could not open config file!" << std::endl;
		return;
	}

	// 解析 JSON
	Json::Value config;
	Json::CharReaderBuilder readerBuilder;
	JSONCPP_STRING errs;
	if (!Json::parseFromStream(readerBuilder, configFile, &config, &errs)) {
		std::cout << "Failed to parse configuration file: " << errs << std::endl;
		return;
	}

	// 从 JSON 中提取参数
	std::string buildFile = s + config["buildFile"].asString();
	std::string buildFile_all = s + config["buildFile_all"].asString();
	std::string pointFile = s + config["pointFile"].asString();
	std::string lineFile = s + config["lineFile"].asString();
	std::string dsmFile = s + config["dsmFile"].asString();
	std::string slopeFile = s + config["slopeFile"].asString();
	std::string projshare = s + config["projshare"].asString();
	std::string json_notfly = s + config["json_notfly"].asString();
	std::string roadFile = s + config["roadFile"].asString();
	std::string sport_groundFile = s + config["sport_groundFile"].asString();
	outputJson_build = s + config["outputJson_build"].asString();
	outputShp_build = s + config["outputShp_build"].asString();
	outputJson_ground = s + config["outputJson_ground"].asString();
	outputShp_ground = s + config["outputShp_ground"].asString();
	outputPdf = s + config["outputPdf"].asString();

	//double precision = config["precision"].asDouble();
	//double squareSize = config["squareSize"].asDouble() / precision;

	//double radius_ground = squareSize / 2;

	//double threshold = config["threshold"].asDouble();
	//double threshold_ground = config["threshold_ground"].asDouble();
	//double circleRadius = config["circleRadius"].asDouble() / precision;
	//double dis_build = config["dis_build"].asDouble();
	//double dis_build_g = config["dis_build_g"].asDouble();
	//double circleRadius_t = config["circleRadius_t"].asDouble() / precision;
	//double circleRadius_s = config["circleRadius_s"].asDouble() / precision;
	//double circleRadius_g = config["circleRadius_g"].asDouble() / precision;
	//double circleRadius_b = config["circleRadius_b"].asDouble() / precision;
	//double circleRadius_j = config["circleRadius_j"].asDouble() / precision;
	//double circleRadius_r = config["circleRadius_r"].asDouble() / precision;
	nest.projPath = projshare;

	//std::cout << "PROJ_LIB:" << getenv("PROJ_LIB") << std::endl;// 查看刚才设置的PROJ_LIB环境变量值
	//std::cout << "PROJ_LIB:" << getenv("Path") << std::endl;// 查看Path环境变量的内容

	nest.openShpFile(buildFile);
	nest.openShpFile_allBuild(buildFile_all);
	nest.openShpFile_point(pointFile);
	nest.openShpFile_road(lineFile);
	nest.readDsm(dsmFile, projshare);
	nest.openRoadShpFile(roadFile);
	nest.openSportShpFile(sport_groundFile);
	nest.readJsonFile(json_notfly);
	nest.buildPreprocessing();
	nest.groundPreprocessing(slopeFile);
	//nest.openBoundaryFile("D:\\shenzhen\\0926\\边界0926.shp");
}

//// 添加 CORS 头的函数
//void addCORSHeaders(crow::response& res) {
//	res.add_header("Access-Control-Allow-Origin", "*"); // 允许所有来源
//	res.add_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS"); // 允许的 HTTP 方法
//	res.add_header("Access-Control-Allow-Headers", "Content-Type, Authorization"); // 允许的请求头
//}

int main()
{
	std::string outputJson_build;
	std::string outputShp_build;
	std::string outputJson_ground;
	std::string outputShp_ground;
	std::string outputPdf;

	DroneNest nest;
	preprocessing(nest, outputJson_build, outputShp_build, outputJson_ground, outputShp_ground, outputPdf);

	double precision1;
	double squareSize_build1;
	double squareSize_ground1;
	double threshold1;
	double threshold_ground1;
	double circleRadius1;
	double dis_build1;
	double circleRadius_t1;
	double dis_build_g1;
	double circleRadius_s1;
	double circleRadius_g1;
	double circleRadius_b1;
	double circleRadius_j1;
	double circleRadius_r1;
	double heightStandard1;
	double nestRadius1;
	bool isbuild;

	crow::SimpleApp app;
	// 定义路由，处理带参数的 GET 请求
	CROW_ROUTE(app, "/parameters")
		([&](const crow::request& req) {
		//crow::response res;
		//// 添加 CORS 头
		//addCORSHeaders(res);

		double precision = std::stod(req.url_params.get("precision"));
		double squareSize_build = std::stod(req.url_params.get("squareSize_build"));
		double squareSize_ground = std::stod(req.url_params.get("squareSize_ground"));
		double threshold = std::stod(req.url_params.get("threshold"));
		double threshold_ground = std::stod(req.url_params.get("threshold_ground"));
		double circleRadius = std::stod(req.url_params.get("circleRadius"));
		double dis_build = std::stod(req.url_params.get("dis_build"));
		double circleRadius_t = std::stod(req.url_params.get("circleRadius_t"));
		double dis_build_g = std::stod(req.url_params.get("dis_build_g"));
		double circleRadius_s = std::stod(req.url_params.get("circleRadius_s"));
		double circleRadius_g = std::stod(req.url_params.get("circleRadius_g"));
		double circleRadius_b = std::stod(req.url_params.get("circleRadius_b"));
		double circleRadius_j = std::stod(req.url_params.get("circleRadius_j"));
		double circleRadius_r = std::stod(req.url_params.get("circleRadius_r"));
		double heightStandard = std::stod(req.url_params.get("heightStandard"));
		double nestRadius = std::stod(req.url_params.get("nestRadius"));

		precision1 = precision;
		squareSize_build1 = squareSize_build;
		squareSize_ground1 = squareSize_ground;
		threshold1 = threshold;
		threshold_ground1 = threshold_ground;
		circleRadius1 = circleRadius;
		dis_build1 = dis_build;
		circleRadius_t1 = circleRadius_t;
		dis_build_g1 = dis_build_g;
		circleRadius_s1 = circleRadius_s;
		circleRadius_g1 = circleRadius_g;
		circleRadius_b1 = circleRadius_b;
		circleRadius_j1 = circleRadius_j;
		circleRadius_r1 = circleRadius_r;
		heightStandard1 = heightStandard;
		nestRadius1 = nestRadius;

		std::string position_str;

		handleRequest(precision, squareSize_build, squareSize_ground, threshold, threshold_ground, circleRadius, dis_build, circleRadius_t,
			dis_build_g, circleRadius_s, circleRadius_g, circleRadius_b, circleRadius_j, circleRadius_r, heightStandard,
			nestRadius, nest, outputJson_build, outputShp_build, outputJson_ground, outputShp_ground, outputPdf, position_str, isbuild);

		// 返回 JSON 响应
		//res = JsonRequest();
	
        // 创建一个新线程来写入 PDF
		std::thread([&nest, outputPdf]() {
			//std::cout << "Writing PDF..." << std::endl;
			nest.writeSquaresToPDF(nest.groundsq, outputPdf);
			//std::cout << "PDF written." << std::endl;
			}).detach();  // 分离线程，允许其独立执行

		return JsonRequest(outputJson_build, outputJson_ground);

		//return "Hello World!";
		//return handleRequest();
			});

	CROW_ROUTE(app, "/parameters/alone")
		([&](const crow::request& req) {

		//crow::response res;
		//addCORSHeaders(res);

		std::string position_str = req.url_params.get("position");
		const char* isbuildParam = req.url_params.get("isbuild");
		if (isbuildParam) {
			std::string isbuildStr(isbuildParam);
			isbuild = (isbuildStr == "true");
		}
		auto start3 = std::chrono::high_resolution_clock::now();

		handleRequest(precision1, squareSize_build1, squareSize_ground1, threshold1, threshold_ground1, circleRadius1, dis_build1, circleRadius_t1,
			dis_build_g1, circleRadius_s1, circleRadius_g1, circleRadius_b1, circleRadius_j1, circleRadius_r1, heightStandard1, 
			nestRadius1, nest, outputJson_build, outputShp_build, outputJson_ground, outputShp_ground, outputPdf, position_str, isbuild);
		auto end3 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed3 = end3 - start3;
		std::cout << "1Time taken: " << elapsed3.count() << " seconds" << std::endl;
		position_str.clear();
		//res = JsonRequest_s(isbuild);
		return JsonRequest_s(isbuild, outputJson_build, outputJson_ground);

		//return "Hello World!";
		//return handleRequest();
			});
	// 启动 HTTP 服务器
	app.port(18080).run();

	return 0;
}

//int main() {
//	CDEM test;
//	test.Open("D:\\深圳无人机\\DOM-1M\\DOM-1M_DSM.tif","D:\\5kmapping\\SatBlockBA@20240130\\proj-share");
//    //test.Read();
//	std::string inputFilePath = "D:\\深圳无人机\\point.txt";
//	std::string outputFilePath = "D:\\深圳无人机\\pointout.txt";
//	std::ifstream inputFile(inputFilePath);
//	std::ofstream outputFile(outputFilePath);
//
//    std::string line;
//    while (std::getline(inputFile, line)) {
//        std::istringstream lineStream(line);
//        std::string point;
//        std::vector<std::pair<int, int>> pixels;
//
//        while (lineStream >> point) {
//            size_t commaPos = point.find(',');
//            if (commaPos == std::string::npos) continue;
//
//            int x = std::stoi(point.substr(0, commaPos));
//            int y = std::stoi(point.substr(commaPos + 1));
//
//            pixels.emplace_back(x, y);
//        }
//
//        if (pixels.size() != 4) {
//            //std::cerr << "Unexpected number of points on line: " << line << std::endl;
//            continue;
//        }
//        outputFile << "[";
//        bool first = true; // 标记是否是第一个坐标对
//        double x, y, z;
//        for (const auto& [pixelX, pixelY] : pixels) {
//            test.PixelCoorToWGS84LBH(pixelX, pixelY, x, y, z);
//            if (!first) {
//                outputFile << ","; // 在第一个坐标对之前不加逗号
//            }
//            outputFile << std::fixed << std::setprecision(6) << "[" << x << "," << y << "]";
//            first = false; // 标记后续坐标对之间需要加逗号
//        }
//        outputFile << "]" << std::endl;
//        //outputFile << std::endl;
//    }
//    inputFile.close();
//    outputFile.close();
//}