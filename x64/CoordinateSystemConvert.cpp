
#define _CRT_SECURE_NO_WARNINGS

#include "CoordinateSystemConvert.h"
#include "gdal_priv.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Eigen {
	typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;
}

bool IsNaN(const float x) { return x != x; }
bool IsNaN(const double x) { return x != x; }

Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& vector) {
	Eigen::Matrix3d matrix;
	matrix << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1),
		vector(0), 0;
	return matrix;
}

void RotationMatrixToEulerAngles(const Eigen::Matrix3d& R, double* rx,
	double* ry, double* rz) {
	*rx = std::atan2(-R(1, 2), R(2, 2));
	*ry = std::asin(R(0, 2));
	*rz = std::atan2(-R(0, 1), R(0, 0));

	*rx = IsNaN(*rx) ? 0 : *rx;
	*ry = IsNaN(*ry) ? 0 : *ry;
	*rz = IsNaN(*rz) ? 0 : *rz;
}

Eigen::Matrix3d EulerAnglesToRotationMatrix(const double rx, const double ry,
	const double rz) {
	const Eigen::Matrix3d Rx =
		Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
	const Eigen::Matrix3d Ry =
		Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();
	const Eigen::Matrix3d Rz =
		Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
	//return Rz * Ry * Rx;  //this is wrong
	return Rx * Ry * Rz;    //this is right
}


Eigen::Matrix3d EulerAnglesToRotationMatrix_new(const double rx, const double ry,
	const double rz) {
	const Eigen::Matrix3d Rx =
		Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
	const Eigen::Matrix3d Ry =
		Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();
	const Eigen::Matrix3d Rz =
		Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
	//return Rz * Ry * Rx;  //this is wrong
	return Rx * Ry * Rz;    //this is right
}

//wai xuan
Eigen::Matrix3d RotateX(double angle) {
	double c = cos(angle);
	double s = sin(angle);

	Eigen::Matrix3d R;
	R(0, 0) = 1.0;
	R(0, 1) = 0.0;
	R(0, 2) = 0.0;
	R(1, 0) = 0.0;
	R(1, 1) = c;
	R(1, 2) = -s;
	R(2, 0) = 0.0;
	R(2, 1) = s;
	R(2, 2) = c;

	return R;
}

Eigen::Matrix3d RotateY(double angle) {
	double c = cos(angle);
	double s = sin(angle);

	Eigen::Matrix3d R;
	R(0, 0) = c;
	R(0, 1) = 0.0;
	R(0, 2) = s;
	R(1, 0) = 0.0;
	R(1, 1) = 1.0;
	R(1, 2) = 0.0;
	R(2, 0) = -s;
	R(2, 1) = 0.0;
	R(2, 2) = c;

	return R;
}

Eigen::Matrix3d RotateZ(double angle) {
	double c = cos(angle);
	double s = sin(angle);

	Eigen::Matrix3d R;
	R(0, 0) = c;
	R(0, 1) = -s;
	R(0, 2) = 0.0;
	R(1, 0) = s;
	R(1, 1) = c;
	R(1, 2) = 0.0;
	R(2, 0) = 0.0;
	R(2, 1) = 0.0;
	R(2, 2) = 1.0;

	return R;
}

Eigen::Matrix3d EulerAnglesToRotationMatrix_my(const double rx, const double ry,
	const double rz) {

	const Eigen::Matrix3d Rx = RotateX(rx);
	const Eigen::Matrix3d Ry = RotateY(ry);
	const Eigen::Matrix3d Rz = RotateZ(rz);

	//return Rz * Ry * Rx;  //this is wrong
	return Rx * Ry * Rz;    //this is right
}

Eigen::Matrix3d EulerAnglesToRotationMatrix_my2(const double rx, const double ry,
	const double rz) {

	double cx = cos(rx);
	double sx = sin(rx);
	double cy = cos(ry);
	double sy = sin(ry);
	double cz = cos(rz);
	double sz = sin(rz);

	Eigen::Matrix3d R;
	R(0, 0) = cy * cz;
	R(0, 1) = -sz * cy;
	R(0, 2) = sy;
	R(1, 0) = sx * sy * cz + cx * sz;
	R(1, 1) = -sz * sx * sy + cx * cz;
	R(1, 2) = -sx * cy;
	R(2, 0) = -sy * cx * cz + sx * sz;
	R(2, 1) = sy * cx * sz + sx * cz;
	R(2, 2) = cx * cy;

	return R;
}

Eigen::Vector4d NormalizeQuaternion(const Eigen::Vector4d& qvec) {
	const double norm = qvec.norm();
	if (norm == 0) {
		// We do not just use (1, 0, 0, 0) because that is a constant and when used
		// for automatic differentiation that would lead to a zero derivative.
		return Eigen::Vector4d(1.0, qvec(1), qvec(2), qvec(3));
	}
	else {
		return qvec / norm;
	}
}

Eigen::Vector4d InvertQuaternion(const Eigen::Vector4d& qvec) {
	return Eigen::Vector4d(qvec(0), -qvec(1), -qvec(2), -qvec(3));
}

Eigen::Vector4d RotationMatrixToQuaternion(const Eigen::Matrix3d& rot_mat) {
	const Eigen::Quaterniond quat(rot_mat);
	return Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
}

Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Vector4d& qvec) {
	const Eigen::Vector4d normalized_qvec = NormalizeQuaternion(qvec);
	const Eigen::Quaterniond quat(normalized_qvec(0), normalized_qvec(1),
		normalized_qvec(2), normalized_qvec(3));
	return quat.toRotationMatrix();
}

Eigen::Vector4d EulerAnglesToQuaternion(const Eigen::Vector3d& Angles) {
	Eigen::Matrix3d R;
	R = EulerAnglesToRotationMatrix(Angles.x(), Angles.y(), Angles.z());
	const Eigen::Quaterniond quat(R);
	//return Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
	return NormalizeQuaternion(
		Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z()));
}

Eigen::Vector3d QuaternionRotatePoint(const Eigen::Vector4d& qvec,
	const Eigen::Vector3d& point) {
	const Eigen::Vector4d normalized_qvec = NormalizeQuaternion(qvec);
	const Eigen::Quaterniond quat(normalized_qvec(0), normalized_qvec(1),
		normalized_qvec(2), normalized_qvec(3));
	return quat * point;
}

Eigen::Vector4d ConcatenateQuaternions(const Eigen::Vector4d& qvec1,
	const Eigen::Vector4d& qvec2) {
	const Eigen::Vector4d normalized_qvec1 = NormalizeQuaternion(qvec1);
	const Eigen::Vector4d normalized_qvec2 = NormalizeQuaternion(qvec2);
	const Eigen::Quaterniond quat1(normalized_qvec1(0), normalized_qvec1(1),
		normalized_qvec1(2), normalized_qvec1(3));
	const Eigen::Quaterniond quat2(normalized_qvec2(0), normalized_qvec2(1),
		normalized_qvec2(2), normalized_qvec2(3));
	const Eigen::Quaterniond cat_quat = quat2 * quat1;
	return NormalizeQuaternion(
		Eigen::Vector4d(cat_quat.w(), cat_quat.x(), cat_quat.y(), cat_quat.z()));
}

Eigen::Matrix3x4d ComposeProjectionMatrix(const Eigen::Vector4d& qvec,
	const Eigen::Vector3d& tvec) {
	Eigen::Matrix3x4d proj_matrix;
	proj_matrix.leftCols<3>() = QuaternionToRotationMatrix(qvec);
	proj_matrix.rightCols<1>() = tvec;
	return proj_matrix;
}

int TransformPOS(Eigen::Affine3d& TransMatrix, Eigen::Vector4d& qvecIn, Eigen::Vector3d& tvecIn,
	Eigen::Vector4d& qvecOut, Eigen::Vector3d& tvecOut)
{
	Eigen::Matrix4d src_matrix = Eigen::MatrixXd::Identity(4, 4);
	src_matrix.topLeftCorner<3, 4>() = ComposeProjectionMatrix(qvecIn, tvecIn);
	Eigen::Matrix4d dst_matrix = src_matrix.matrix() * TransMatrix.inverse().matrix();
	double scale = TransMatrix.matrix().block<1, 3>(0, 0).norm();
	dst_matrix *= scale;

	qvecOut = RotationMatrixToQuaternion(dst_matrix.block<3, 3>(0, 0));
	tvecOut = dst_matrix.block<3, 1>(0, 3);

	return 1;
}

///////////////////////////////////////////////////////////////////////////////////////

CCoordinateSystemConvert::CCoordinateSystemConvert(const char* proj_share)
{
	m_pInputCS = (void*)new OGRSpatialReference;
	m_pOutputCS = (void*)new OGRSpatialReference;

	const char* proj_path[] = { proj_share, nullptr };
	OSRSetPROJSearchPaths(proj_path);

	//CPLSetConfigOption("GDAL_DATA", "D:/program/OpenSource@2020/gdal/gdal-3.6.3/home/share/gdal");
}

CCoordinateSystemConvert::~CCoordinateSystemConvert()
{
	if (m_pInputCS)
	{
		delete m_pInputCS;
		m_pInputCS = nullptr;
	}

	if (m_pOutputCS)
	{
		delete m_pOutputCS;
		m_pOutputCS = nullptr;
	}
}

// 뵽ת
bool CCoordinateSystemConvert::Transform(int count, double* east, double* north, double* height)
{
	OGRSpatialReference* in = (OGRSpatialReference*)m_pInputCS;
	OGRSpatialReference* out = (OGRSpatialReference*)m_pOutputCS;

	if (in->IsSame(out)) {
		return true;
	}

	OGRCoordinateTransformation* trans = OGRCreateCoordinateTransformation(in, out);
	if (!trans)
	{
		return false;
	}

	if (!trans->Transform(count, east, north, height))
	{
		return false;
	}

	OGRCoordinateTransformation::DestroyCT(trans);
	return true;
}

bool CCoordinateSystemConvert::InverseTransform(int count, double* east, double* north, double* height)
{
	OGRSpatialReference* in = (OGRSpatialReference*)m_pInputCS;
	OGRSpatialReference* out = (OGRSpatialReference*)m_pOutputCS;

	if (in->IsSame(out)) {
		return true;
	}

	OGRCoordinateTransformation* trans = OGRCreateCoordinateTransformation(out, in);
	if (!trans)
	{
		return false;
	}

	if (!trans->Transform(count, east, north, height))
	{
		return false;
	}

	OGRCoordinateTransformation::DestroyCT(trans);
	return true;
}

std::unordered_map<std::string, double> CCoordinateSystemConvert::getParam(int ESPGValue)
{
	OGRSpatialReference spatialReference;
	spatialReference.importFromEPSG(ESPGValue);
	char* pszWKT = nullptr;
	spatialReference.exportToPrettyWkt(&pszWKT);

	std::stringstream ss(pszWKT);
	std::string line_string;
	std::unordered_map<std::string, double> map;
	std::string str1 = "latitude_of_origin";
	std::string str2 = "central_meridian";
	std::string str3 = "scale_factor";
	std::string str4 = "false_easting";
	std::string str5 = "false_northing";
	while (getline(ss, line_string))
	{
		std::size_t found1 = line_string.find(str1);
		if (found1 != std::string::npos) {
			std::string latitude_of_origin_value = line_string.substr(found1 + str1.length() + 2, std::string::npos);
			double value1 = std::stod(latitude_of_origin_value);
			map[str1] = value1;
		}
		std::size_t found2 = line_string.find(str2);
		if (found2 != std::string::npos) {
			std::string central_meridian_value = line_string.substr(found2 + str2.length() + 2, std::string::npos);
			double value2 = std::stod(central_meridian_value);
			map[str2] = value2;
		}
		std::size_t found3 = line_string.find(str3);
		if (found3 != std::string::npos) {
			std::string scale_factor_value = line_string.substr(found3 + str3.length() + 2, std::string::npos);
			double value3 = std::stod(scale_factor_value);
			map[str3] = value3;
		}
		std::size_t found4 = line_string.find(str4);
		if (found4 != std::string::npos) {
			std::string false_easting_value = line_string.substr(found4 + str4.length() + 2, std::string::npos);
			double value4 = std::stod(false_easting_value);
			map[str4] = value4;
		}
		std::size_t found5 = line_string.find(str5);
		if (found5 != std::string::npos) {
			std::string false_northing_value = line_string.substr(found5 + str5.length() + 2, std::string::npos);
			double value5 = std::stod(false_northing_value);
			map[str5] = value5;
		}
	}

	return map;
}

bool CCoordinateSystemConvert::SetInputCoordByEPSG(int epsg)
{
	OGRSpatialReference* in = (OGRSpatialReference*)m_pInputCS;
	if (OGRERR_NONE == in->importFromEPSG(epsg)) {
		return true;
	}
	else {
		return false;
	}
}

bool CCoordinateSystemConvert::SetOutputCoordByEPSG(int epsg)
{
	OGRSpatialReference* out = (OGRSpatialReference*)m_pOutputCS;
	if (OGRERR_NONE == out->importFromEPSG(epsg)) {
		return true;
	}
	else {
		return false;
	}
}

bool CCoordinateSystemConvert::GetSrsInfo(std::string srs, int& epsg)
{
	std::size_t pos_epsg1 = srs.find("(");
	std::size_t pos_epsg2 = srs.find(")");
	std::string srs_epsg = srs.substr(pos_epsg1 + 6, pos_epsg2 - pos_epsg1 - 6);
	epsg = stoi(srs_epsg);
	return true;
}

bool CCoordinateSystemConvert::SetInputCoordBySrs(std::string srs)
{
	int epsg = 0;
	GetSrsInfo(srs, epsg);

	return SetInputCoordByEPSG(epsg);
}

bool CCoordinateSystemConvert::SetOutputCoordBySrs(std::string srs)
{
	int epsg = 0;
	GetSrsInfo(srs, epsg);

	return SetOutputCoordByEPSG(epsg);
}

int CCoordinateSystemConvert::File2WKTString(char* pwktFile, std::string& wktString)
{
	FILE* pF = fopen(pwktFile, "r");
	if (pF == nullptr) {
		return 0;
	}

	while (!feof(pF)) {
		char pTemp[512] = { 0 };
		fgets(pTemp, 512, pF);
		if (strlen(pTemp) > 0) {
			wktString.append(pTemp);
		}
	}

	return 1;
}

bool CCoordinateSystemConvert::SetInputCoordByWKT(std::string wktString)
{
	char* pTemp = (char*)wktString.c_str();

	OGRSpatialReference* in = (OGRSpatialReference*)m_pInputCS;
	if (OGRERR_NONE == in->importFromWkt(&pTemp)) {
		return true;
	}
	else {
		return false;
	}
}

bool CCoordinateSystemConvert::SetOutputCoordByWKT(std::string wktString)
{
	char* pTemp = (char*)wktString.c_str();

	OGRSpatialReference* out = (OGRSpatialReference*)m_pOutputCS;
	if (OGRERR_NONE == out->importFromWkt(&pTemp)) {
		return true;
	}
	else {
		return false;
	}
}

bool CCoordinateSystemConvert::SetInputCoordByProj4(std::string Proj4String)
{
	char* pTemp = (char*)Proj4String.c_str();

	OGRSpatialReference* in = (OGRSpatialReference*)m_pInputCS;
	if (OGRERR_NONE == in->importFromProj4(pTemp)) {
		return true;
	}
	else {
		return false;
	}
}

bool CCoordinateSystemConvert::SetOutputCoordByProj4(std::string Proj4String)
{
	char* pTemp = (char*)Proj4String.c_str();

	OGRSpatialReference* out = (OGRSpatialReference*)m_pOutputCS;
	if (OGRERR_NONE == out->importFromProj4(pTemp)) {
		return true;
	}
	else {
		return false;
	}
}

///////////////////////////////////////////////////////////////

CLocalCSConvert::CLocalCSConvert(const char* proj_share)
{
	const char* geoccs = "+proj=geocent +datum=WGS84 +units=m +no_defs +type=crs";
	m_pCoorCvtWGS84ToCXYZ = new CCoordinateSystemConvert(proj_share);
	m_pCoorCvtWGS84ToCXYZ->SetInputCoordByEPSG(4326);
	m_pCoorCvtWGS84ToCXYZ->SetOutputCoordByProj4(geoccs);
}

CLocalCSConvert::~CLocalCSConvert()
{
	if (m_pCoorCvtWGS84ToCXYZ) {
		delete m_pCoorCvtWGS84ToCXYZ; m_pCoorCvtWGS84ToCXYZ = nullptr;
	}
}

int CLocalCSConvert::DefineENUByLBH(double Origin_Lon, double Origin_Lat, double Origin_Hgt)
{
	if (m_pCoorCvtWGS84ToCXYZ == nullptr) {
		printf("CoorCvtWGS84ToCXYZ error.\n");
		return false;
	}

	m_OriginLBH[0] = Origin_Lon;
	m_OriginLBH[1] = Origin_Lat;
	m_OriginLBH[2] = Origin_Hgt;

	m_OriginECEF[0] = m_OriginLBH[0];
	m_OriginECEF[1] = m_OriginLBH[1];
	m_OriginECEF[2] = m_OriginLBH[2];

	m_pCoorCvtWGS84ToCXYZ->Transform(1, &(m_OriginECEF[0]), &(m_OriginECEF[1]), &(m_OriginECEF[2]));

	return 1;
}

int CLocalCSConvert::DefineENUByCXYZ(double Origin_CX, double Origin_CY, double Origin_CZ)
{
	if (m_pCoorCvtWGS84ToCXYZ == nullptr) {
		printf("CoorCvtWGS84ToCXYZ error.\n");
		return false;
	}

	m_OriginECEF[0] = Origin_CX;
	m_OriginECEF[1] = Origin_CY;
	m_OriginECEF[2] = Origin_CZ;

	m_OriginLBH[0] = m_OriginECEF[0];
	m_OriginLBH[1] = m_OriginECEF[1];
	m_OriginLBH[2] = m_OriginECEF[2];

	m_pCoorCvtWGS84ToCXYZ->InverseTransform(1, &(m_OriginLBH[0]), &(m_OriginLBH[1]), &(m_OriginLBH[2]));

	return 1;
}

int CLocalCSConvert::ENUToECEFMatrix(double TransformMatrix[12])
{
	const double epsilon = 0.000000000000001;
	const double pi = 3.14159265358979323846;
	const double d2r = pi / 180;
	const double r2d = 180 / pi;

	double rzAngle = (m_OriginLBH[0] * d2r + pi / 2);
	Eigen::AngleAxisd rzAngleAxis(rzAngle, Eigen::Vector3d(0, 0, 1));
	Eigen::Matrix3d rZ = rzAngleAxis.matrix();

	double rxAngle = (pi / 2 - m_OriginLBH[1] * d2r);
	Eigen::AngleAxisd rxAngleAxis(rxAngle, Eigen::Vector3d(1, 0, 0));
	Eigen::Matrix3d rX = rxAngleAxis.matrix();

	Eigen::Matrix4d rotation;
	rotation.setIdentity();
	rotation.block<3, 3>(0, 0) = (rZ * rX);

	double tx = m_OriginECEF[0];
	double ty = m_OriginECEF[1];
	double tz = m_OriginECEF[2];

	Eigen::Matrix4d translation;
	translation.setIdentity();
	translation(0, 3) = tx;
	translation(1, 3) = ty;
	translation(2, 3) = tz;

	Eigen::Matrix4d Matrix = translation * rotation;

	TransformMatrix[0] = Matrix(0, 0); TransformMatrix[1] = Matrix(0, 1); TransformMatrix[2] = Matrix(0, 2); TransformMatrix[3] = Matrix(0, 3);
	TransformMatrix[4] = Matrix(1, 0); TransformMatrix[5] = Matrix(1, 1); TransformMatrix[6] = Matrix(1, 2); TransformMatrix[7] = Matrix(1, 3);
	TransformMatrix[8] = Matrix(2, 0); TransformMatrix[9] = Matrix(2, 1); TransformMatrix[10] = Matrix(2, 2); TransformMatrix[11] = Matrix(2, 3);

	return 1;
}

int CLocalCSConvert::ECEFToENUMatrix(double TransformMatrix[12])
{
	const double epsilon = 0.000000000000001;
	const double pi = 3.14159265358979323846;
	const double d2r = pi / 180;
	const double r2d = 180 / pi;

	double rzAngle = -(m_OriginLBH[0] * d2r + pi / 2);
	Eigen::AngleAxisd rzAngleAxis(rzAngle, Eigen::Vector3d(0, 0, 1));
	Eigen::Matrix3d rZ = rzAngleAxis.matrix();

	double rxAngle = -(pi / 2 - m_OriginLBH[1] * d2r);
	Eigen::AngleAxisd rxAngleAxis(rxAngle, Eigen::Vector3d(1, 0, 0));
	Eigen::Matrix3d rX = rxAngleAxis.matrix();

	Eigen::Matrix4d rotation;
	rotation.setIdentity();
	rotation.block<3, 3>(0, 0) = (rX * rZ);

	double tx = m_OriginECEF[0];
	double ty = m_OriginECEF[1];
	double tz = m_OriginECEF[2];

	Eigen::Matrix4d translation;
	translation.setIdentity();
	translation(0, 3) = -tx;
	translation(1, 3) = -ty;
	translation(2, 3) = -tz;

	Eigen::Matrix4d Matrix = rotation * translation;

	TransformMatrix[0] = Matrix(0, 0); TransformMatrix[1] = Matrix(0, 1); TransformMatrix[2] = Matrix(0, 2); TransformMatrix[3] = Matrix(0, 3);
	TransformMatrix[4] = Matrix(1, 0); TransformMatrix[5] = Matrix(1, 1); TransformMatrix[6] = Matrix(1, 2); TransformMatrix[7] = Matrix(1, 3);
	TransformMatrix[8] = Matrix(2, 0); TransformMatrix[9] = Matrix(2, 1); TransformMatrix[10] = Matrix(2, 2); TransformMatrix[11] = Matrix(2, 3);

	return 1;
}

int CLocalCSConvert::PointFromENUToECEF(std::vector<double>& Points)
{
	double TransformMatrix[12] = { 0 };
	ENUToECEFMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < Points.size(); i += 3) {
		Eigen::Vector3d pt1 = Eigen::Vector3d(Points[i + 0], Points[i + 1], Points[i + 2]);
		Eigen::Vector3d pt2 = Matrix * pt1;

		Points[i + 0] = pt2.x();
		Points[i + 1] = pt2.y();
		Points[i + 2] = pt2.z();
	}

	return 1;
}

int CLocalCSConvert::PointFromECEFToENU(std::vector<double>& Points)
{
	double TransformMatrix[12] = { 0 };
	ECEFToENUMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < Points.size(); i += 3) {
		Eigen::Vector3d pt1 = Eigen::Vector3d(Points[i + 0], Points[i + 1], Points[i + 2]);
		Eigen::Vector3d pt2 = Matrix * pt1;

		Points[i + 0] = pt2.x();
		Points[i + 1] = pt2.y();
		Points[i + 2] = pt2.z();
	}

	return 1;
}

int CLocalCSConvert::PointFromENUToECEF(int count, double* X, double* Y, double* Z)
{
	double TransformMatrix[12] = { 0 };
	ENUToECEFMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector3d pt1 = Eigen::Vector3d(X[i], Y[i], Z[i]);
		Eigen::Vector3d pt2 = Matrix * pt1;

		X[i] = pt2.x();
		Y[i] = pt2.y();
		Z[i] = pt2.z();
	}

	return 1;
}

int CLocalCSConvert::PointFromECEFToENU(int count, double* X, double* Y, double* Z)
{
	double TransformMatrix[12] = { 0 };
	ECEFToENUMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector3d pt1 = Eigen::Vector3d(X[i], Y[i], Z[i]);
		Eigen::Vector3d pt2 = Matrix * pt1;

		X[i] = pt2.x();
		Y[i] = pt2.y();
		Z[i] = pt2.z();
	}

	return 1;
}

int CLocalCSConvert::VectorFromENUToECEF(std::vector<double>& Vectors)
{
	double TransformMatrix[12] = { 0 };
	ENUToECEFMatrix(TransformMatrix);

	Eigen::Matrix3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10];

	for (int i = 0; i < Vectors.size(); i += 3) {
		Eigen::Vector3d Vt1 = Eigen::Vector3d(Vectors[i + 0], Vectors[i + 1], Vectors[i + 2]);
		Eigen::Vector3d Vt2 = Matrix * Vt1;

		Vectors[i + 0] = Vt2.x();
		Vectors[i + 1] = Vt2.y();
		Vectors[i + 2] = Vt2.z();
	}

	return 1;
}

int CLocalCSConvert::VectorFromECEFToENU(std::vector<double>& Vectors)
{
	double TransformMatrix[12] = { 0 };
	ECEFToENUMatrix(TransformMatrix);

	Eigen::Matrix3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10];

	for (int i = 0; i < Vectors.size(); i += 3) {
		Eigen::Vector3d Vt1 = Eigen::Vector3d(Vectors[i + 0], Vectors[i + 1], Vectors[i + 2]);
		Eigen::Vector3d Vt2 = Matrix * Vt1;

		Vectors[i + 0] = Vt2.x();
		Vectors[i + 1] = Vt2.y();
		Vectors[i + 2] = Vt2.z();
	}

	return 1;
}

int CLocalCSConvert::VectorFromENUToECEF(int count, double* Vx, double* Vy, double* Vz)
{
	double TransformMatrix[12] = { 0 };
	ENUToECEFMatrix(TransformMatrix);

	Eigen::Matrix3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10];

	for (int i = 0; i < count; i++) {
		Eigen::Vector3d Vt1 = Eigen::Vector3d(Vx[i], Vy[i], Vz[i]);
		Eigen::Vector3d Vt2 = Matrix * Vt1;

		Vx[i] = Vt2.x();
		Vy[i] = Vt2.y();
		Vz[i] = Vt2.z();
	}

	return 1;
}

int CLocalCSConvert::VectorFromECEFToENU(int count, double* Vx, double* Vy, double* Vz)
{
	double TransformMatrix[12] = { 0 };
	ECEFToENUMatrix(TransformMatrix);

	Eigen::Matrix3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10];

	for (int i = 0; i < count; i++) {
		Eigen::Vector3d Vt1 = Eigen::Vector3d(Vx[i], Vy[i], Vz[i]);
		Eigen::Vector3d Vt2 = Matrix * Vt1;

		Vx[i] = Vt2.x();
		Vy[i] = Vt2.y();
		Vz[i] = Vt2.z();
	}

	return 1;
}

int CLocalCSConvert::POSQTFromENUToECEF(int count, double* Q, double* T)
{
	double TransformMatrix[12] = { 0 };
	ENUToECEFMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector4d qvec = Eigen::Vector4d(Q[i * 4 + 0], Q[i * 4 + 1], Q[i * 4 + 2], Q[i * 4 + 3]);
		Eigen::Vector3d tvec = Eigen::Vector3d(T[i * 3 + 0], T[i * 3 + 1], T[i * 3 + 2]);
		Eigen::Vector4d qvecNew = qvec;
		Eigen::Vector3d tvecNew = tvec;

		TransformPOS(Matrix, qvec, tvec, qvecNew, tvecNew);

		Q[i * 4 + 0] = qvecNew(0);
		Q[i * 4 + 1] = qvecNew(1);
		Q[i * 4 + 2] = qvecNew(2);
		Q[i * 4 + 3] = qvecNew(3);

		T[i * 3 + 0] = tvecNew(0);
		T[i * 3 + 1] = tvecNew(1);
		T[i * 3 + 2] = tvecNew(2);
	}

	return 1;
}

int CLocalCSConvert::POSQTFromECEFToENU(int count, double* Q, double* T)
{
	double TransformMatrix[12] = { 0 };
	ECEFToENUMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector4d qvec = Eigen::Vector4d(Q[i * 4 + 0], Q[i * 4 + 1], Q[i * 4 + 2], Q[i * 4 + 3]);
		Eigen::Vector3d tvec = Eigen::Vector3d(T[i * 3 + 0], T[i * 3 + 1], T[i * 3 + 2]);
		Eigen::Vector4d qvecNew = qvec;
		Eigen::Vector3d tvecNew = tvec;

		TransformPOS(Matrix, qvec, tvec, qvecNew, tvecNew);

		Q[i * 4 + 0] = qvecNew(0);
		Q[i * 4 + 1] = qvecNew(1);
		Q[i * 4 + 2] = qvecNew(2);
		Q[i * 4 + 3] = qvecNew(3);

		T[i * 3 + 0] = tvecNew(0);
		T[i * 3 + 1] = tvecNew(1);
		T[i * 3 + 2] = tvecNew(2);
	}

	return 1;
}

int CLocalCSConvert::POSQCFromENUToECEF(int count, double* Q, double* C)
{
	double TransformMatrix[12] = { 0 };
	ENUToECEFMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector4d qvec = Eigen::Vector4d(Q[i * 4 + 0], Q[i * 4 + 1], Q[i * 4 + 2], Q[i * 4 + 3]);
		Eigen::Vector3d Cvec = Eigen::Vector3d(C[i * 3 + 0], C[i * 3 + 1], C[i * 3 + 2]);
		Eigen::Vector3d tvec = -QuaternionRotatePoint(qvec, Cvec);

		Eigen::Vector4d qvecNew = qvec;
		Eigen::Vector3d tvecNew = tvec;

		TransformPOS(Matrix, qvec, tvec, qvecNew, tvecNew);

		Eigen::Vector4d inv_qvecNew = InvertQuaternion(qvecNew);
		Eigen::Vector3d CvecNew = -QuaternionRotatePoint(inv_qvecNew, tvecNew);

		Q[i * 4 + 0] = qvecNew(0);
		Q[i * 4 + 1] = qvecNew(1);
		Q[i * 4 + 2] = qvecNew(2);
		Q[i * 4 + 3] = qvecNew(3);

		C[i * 3 + 0] = CvecNew(0);
		C[i * 3 + 1] = CvecNew(1);
		C[i * 3 + 2] = CvecNew(2);
	}

	return 1;
}

int CLocalCSConvert::POSQCFromECEFToENU(int count, double* Q, double* C)
{
	double TransformMatrix[12] = { 0 };
	ECEFToENUMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector4d qvec = Eigen::Vector4d(Q[i * 4 + 0], Q[i * 4 + 1], Q[i * 4 + 2], Q[i * 4 + 3]);
		Eigen::Vector3d Cvec = Eigen::Vector3d(C[i * 3 + 0], C[i * 3 + 1], C[i * 3 + 2]);
		Eigen::Vector3d tvec = -QuaternionRotatePoint(qvec, Cvec);

		Eigen::Vector4d qvecNew = qvec;
		Eigen::Vector3d tvecNew = tvec;

		TransformPOS(Matrix, qvec, tvec, qvecNew, tvecNew);

		Eigen::Vector4d inv_qvecNew = InvertQuaternion(qvecNew);
		Eigen::Vector3d CvecNew = -QuaternionRotatePoint(inv_qvecNew, tvecNew);

		Q[i * 4 + 0] = qvecNew(0);
		Q[i * 4 + 1] = qvecNew(1);
		Q[i * 4 + 2] = qvecNew(2);
		Q[i * 4 + 3] = qvecNew(3);

		C[i * 3 + 0] = CvecNew(0);
		C[i * 3 + 1] = CvecNew(1);
		C[i * 3 + 2] = CvecNew(2);
	}

	return 1;
}

int CLocalCSConvert::DefineNEDByLBH(double Origin_Lon, double Origin_Lat, double Origin_Hgt)
{
	if (m_pCoorCvtWGS84ToCXYZ == nullptr) {
		printf("CoorCvtWGS84ToCXYZ error.\n");
		return false;
	}

	m_OriginLBH[0] = Origin_Lon;
	m_OriginLBH[1] = Origin_Lat;
	m_OriginLBH[2] = Origin_Hgt;

	m_OriginECEF[0] = m_OriginLBH[0];
	m_OriginECEF[1] = m_OriginLBH[1];
	m_OriginECEF[2] = m_OriginLBH[2];

	m_pCoorCvtWGS84ToCXYZ->Transform(1, &(m_OriginECEF[0]), &(m_OriginECEF[1]), &(m_OriginECEF[2]));

	return 1;
}

int CLocalCSConvert::DefineNEDByCXYZ(double Origin_CX, double Origin_CY, double Origin_CZ)
{
	if (m_pCoorCvtWGS84ToCXYZ == nullptr) {
		printf("CoorCvtWGS84ToCXYZ error.\n");
		return false;
	}

	m_OriginECEF[0] = Origin_CX;
	m_OriginECEF[1] = Origin_CY;
	m_OriginECEF[2] = Origin_CZ;

	m_OriginLBH[0] = m_OriginECEF[0];
	m_OriginLBH[1] = m_OriginECEF[1];
	m_OriginLBH[2] = m_OriginECEF[2];

	m_pCoorCvtWGS84ToCXYZ->InverseTransform(1, &(m_OriginLBH[0]), &(m_OriginLBH[1]), &(m_OriginLBH[2]));

	return 1;
}

int CLocalCSConvert::NEDToECEFMatrix(double TransformMatrix[12])
{
	const double epsilon = 0.000000000000001;
	const double pi = 3.14159265358979323846;
	const double d2r = pi / 180;
	const double r2d = 180 / pi;

	double rzAngle = (m_OriginLBH[0] * d2r + pi / 2);
	Eigen::AngleAxisd rzAngleAxis(rzAngle, Eigen::Vector3d(0, 0, 1));
	Eigen::Matrix3d rZ = rzAngleAxis.matrix();

	double rxAngle = (pi / 2 - m_OriginLBH[1] * d2r);
	Eigen::AngleAxisd rxAngleAxis(rxAngle, Eigen::Vector3d(1, 0, 0));
	Eigen::Matrix3d rX = rxAngleAxis.matrix();

	Eigen::Matrix4d rotation;
	rotation.setIdentity();
	Eigen::Matrix3d R_ENU_2_ECEF = (rZ * rX);

	Eigen::Matrix3d NED_2_ENU;
	NED_2_ENU(0, 0) = 0; NED_2_ENU(0, 1) = 1; NED_2_ENU(0, 2) = 0;
	NED_2_ENU(1, 0) = 1; NED_2_ENU(1, 1) = 0; NED_2_ENU(1, 2) = 0;
	NED_2_ENU(2, 0) = 0; NED_2_ENU(2, 1) = 0; NED_2_ENU(2, 2) = -1;

	rotation.block<3, 3>(0, 0) = R_ENU_2_ECEF * NED_2_ENU; // R_NED_2_ECEF

	double tx = m_OriginECEF[0];
	double ty = m_OriginECEF[1];
	double tz = m_OriginECEF[2];

	Eigen::Matrix4d translation;
	translation.setIdentity();
	translation(0, 3) = tx;
	translation(1, 3) = ty;
	translation(2, 3) = tz;

	Eigen::Matrix4d Matrix = translation * rotation;

	TransformMatrix[0] = Matrix(0, 0); TransformMatrix[1] = Matrix(0, 1); TransformMatrix[2] = Matrix(0, 2); TransformMatrix[3] = Matrix(0, 3);
	TransformMatrix[4] = Matrix(1, 0); TransformMatrix[5] = Matrix(1, 1); TransformMatrix[6] = Matrix(1, 2); TransformMatrix[7] = Matrix(1, 3);
	TransformMatrix[8] = Matrix(2, 0); TransformMatrix[9] = Matrix(2, 1); TransformMatrix[10] = Matrix(2, 2); TransformMatrix[11] = Matrix(2, 3);

	return 1;
}

int CLocalCSConvert::ECEFToNEDMatrix(double TransformMatrix[12])
{
	const double epsilon = 0.000000000000001;
	const double pi = 3.14159265358979323846;
	const double d2r = pi / 180;
	const double r2d = 180 / pi;

	double rzAngle = -(m_OriginLBH[0] * d2r + pi / 2);
	Eigen::AngleAxisd rzAngleAxis(rzAngle, Eigen::Vector3d(0, 0, 1));
	Eigen::Matrix3d rZ = rzAngleAxis.matrix();

	double rxAngle = -(pi / 2 - m_OriginLBH[1] * d2r);
	Eigen::AngleAxisd rxAngleAxis(rxAngle, Eigen::Vector3d(1, 0, 0));
	Eigen::Matrix3d rX = rxAngleAxis.matrix();

	Eigen::Matrix4d rotation;
	rotation.setIdentity();
	Eigen::Matrix3d R_ECEF_2_ENU = (rX * rZ);

	Eigen::Matrix3d NED_2_ENU;
	NED_2_ENU(0, 0) = 0; NED_2_ENU(0, 1) = 1; NED_2_ENU(0, 2) = 0;
	NED_2_ENU(1, 0) = 1; NED_2_ENU(1, 1) = 0; NED_2_ENU(1, 2) = 0;
	NED_2_ENU(2, 0) = 0; NED_2_ENU(2, 1) = 0; NED_2_ENU(2, 2) = -1;

	rotation.block<3, 3>(0, 0) = NED_2_ENU.transpose() * R_ECEF_2_ENU; // R_ECEF_2_NED

	double tx = m_OriginECEF[0];
	double ty = m_OriginECEF[1];
	double tz = m_OriginECEF[2];

	Eigen::Matrix4d translation;
	translation.setIdentity();
	translation(0, 3) = -tx;
	translation(1, 3) = -ty;
	translation(2, 3) = -tz;

	Eigen::Matrix4d Matrix = rotation * translation;

	TransformMatrix[0] = Matrix(0, 0); TransformMatrix[1] = Matrix(0, 1); TransformMatrix[2] = Matrix(0, 2); TransformMatrix[3] = Matrix(0, 3);
	TransformMatrix[4] = Matrix(1, 0); TransformMatrix[5] = Matrix(1, 1); TransformMatrix[6] = Matrix(1, 2); TransformMatrix[7] = Matrix(1, 3);
	TransformMatrix[8] = Matrix(2, 0); TransformMatrix[9] = Matrix(2, 1); TransformMatrix[10] = Matrix(2, 2); TransformMatrix[11] = Matrix(2, 3);

	return 1;
}

int CLocalCSConvert::PointFromNEDToECEF(std::vector<double>& Points)
{
	double TransformMatrix[12] = { 0 };
	NEDToECEFMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < Points.size(); i += 3) {
		Eigen::Vector3d pt1 = Eigen::Vector3d(Points[i + 0], Points[i + 1], Points[i + 2]);
		Eigen::Vector3d pt2 = Matrix * pt1;

		Points[i + 0] = pt2.x();
		Points[i + 1] = pt2.y();
		Points[i + 2] = pt2.z();
	}

	return 1;
}

int CLocalCSConvert::PointFromECEFToNED(std::vector<double>& Points)
{
	double TransformMatrix[12] = { 0 };
	ECEFToNEDMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < Points.size(); i += 3) {
		Eigen::Vector3d pt1 = Eigen::Vector3d(Points[i + 0], Points[i + 1], Points[i + 2]);
		Eigen::Vector3d pt2 = Matrix * pt1;

		Points[i + 0] = pt2.x();
		Points[i + 1] = pt2.y();
		Points[i + 2] = pt2.z();
	}

	return 1;
}

int CLocalCSConvert::PointFromNEDToECEF(int count, double* X, double* Y, double* Z)
{
	double TransformMatrix[12] = { 0 };
	NEDToECEFMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector3d pt1 = Eigen::Vector3d(X[i], Y[i], Z[i]);
		Eigen::Vector3d pt2 = Matrix * pt1;

		X[i] = pt2.x();
		Y[i] = pt2.y();
		Z[i] = pt2.z();
	}

	return 1;
}

int CLocalCSConvert::PointFromECEFToNED(int count, double* X, double* Y, double* Z)
{
	double TransformMatrix[12] = { 0 };
	ECEFToNEDMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector3d pt1 = Eigen::Vector3d(X[i], Y[i], Z[i]);
		Eigen::Vector3d pt2 = Matrix * pt1;

		X[i] = pt2.x();
		Y[i] = pt2.y();
		Z[i] = pt2.z();
	}

	return 1;
}

int CLocalCSConvert::VectorFromNEDToECEF(std::vector<double>& Vectors)
{
	double TransformMatrix[12] = { 0 };
	NEDToECEFMatrix(TransformMatrix);

	Eigen::Matrix3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10];

	for (int i = 0; i < Vectors.size(); i += 3) {
		Eigen::Vector3d Vt1 = Eigen::Vector3d(Vectors[i + 0], Vectors[i + 1], Vectors[i + 2]);
		Eigen::Vector3d Vt2 = Matrix * Vt1;

		Vectors[i + 0] = Vt2.x();
		Vectors[i + 1] = Vt2.y();
		Vectors[i + 2] = Vt2.z();
	}

	return 1;
}

int CLocalCSConvert::VectorFromECEFToNED(std::vector<double>& Vectors)
{
	double TransformMatrix[12] = { 0 };
	ECEFToNEDMatrix(TransformMatrix);

	Eigen::Matrix3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10];

	for (int i = 0; i < Vectors.size(); i += 3) {
		Eigen::Vector3d Vt1 = Eigen::Vector3d(Vectors[i + 0], Vectors[i + 1], Vectors[i + 2]);
		Eigen::Vector3d Vt2 = Matrix * Vt1;

		Vectors[i + 0] = Vt2.x();
		Vectors[i + 1] = Vt2.y();
		Vectors[i + 2] = Vt2.z();
	}

	return 1;
}

int CLocalCSConvert::VectorFromNEDToECEF(int count, double* Vx, double* Vy, double* Vz)
{
	double TransformMatrix[12] = { 0 };
	NEDToECEFMatrix(TransformMatrix);

	Eigen::Matrix3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10];

	for (int i = 0; i < count; i++) {
		Eigen::Vector3d Vt1 = Eigen::Vector3d(Vx[i], Vy[i], Vz[i]);
		Eigen::Vector3d Vt2 = Matrix * Vt1;

		Vx[i] = Vt2.x();
		Vy[i] = Vt2.y();
		Vz[i] = Vt2.z();
	}

	return 1;
}

int CLocalCSConvert::VectorFromECEFToNED(int count, double* Vx, double* Vy, double* Vz)
{
	double TransformMatrix[12] = { 0 };
	ECEFToNEDMatrix(TransformMatrix);

	Eigen::Matrix3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10];

	for (int i = 0; i < count; i++) {
		Eigen::Vector3d Vt1 = Eigen::Vector3d(Vx[i], Vy[i], Vz[i]);
		Eigen::Vector3d Vt2 = Matrix * Vt1;

		Vx[i] = Vt2.x();
		Vy[i] = Vt2.y();
		Vz[i] = Vt2.z();
	}

	return 1;
}

int CLocalCSConvert::POSQTFromNEDToECEF(int count, double* Q, double* T)
{
	double TransformMatrix[12] = { 0 };
	NEDToECEFMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector4d qvec = Eigen::Vector4d(Q[i * 4 + 0], Q[i * 4 + 1], Q[i * 4 + 2], Q[i * 4 + 3]);
		Eigen::Vector3d tvec = Eigen::Vector3d(T[i * 3 + 0], T[i * 3 + 1], T[i * 3 + 2]);
		Eigen::Vector4d qvecNew = qvec;
		Eigen::Vector3d tvecNew = tvec;

		TransformPOS(Matrix, qvec, tvec, qvecNew, tvecNew);

		Q[i * 4 + 0] = qvecNew(0);
		Q[i * 4 + 1] = qvecNew(1);
		Q[i * 4 + 2] = qvecNew(2);
		Q[i * 4 + 3] = qvecNew(3);

		T[i * 3 + 0] = tvecNew(0);
		T[i * 3 + 1] = tvecNew(1);
		T[i * 3 + 2] = tvecNew(2);
	}

	return 1;
}

int CLocalCSConvert::POSQTFromECEFToNED(int count, double* Q, double* T)
{
	double TransformMatrix[12] = { 0 };
	ECEFToNEDMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector4d qvec = Eigen::Vector4d(Q[i * 4 + 0], Q[i * 4 + 1], Q[i * 4 + 2], Q[i * 4 + 3]);
		Eigen::Vector3d tvec = Eigen::Vector3d(T[i * 3 + 0], T[i * 3 + 1], T[i * 3 + 2]);
		Eigen::Vector4d qvecNew = qvec;
		Eigen::Vector3d tvecNew = tvec;

		TransformPOS(Matrix, qvec, tvec, qvecNew, tvecNew);

		Q[i * 4 + 0] = qvecNew(0);
		Q[i * 4 + 1] = qvecNew(1);
		Q[i * 4 + 2] = qvecNew(2);
		Q[i * 4 + 3] = qvecNew(3);

		T[i * 3 + 0] = tvecNew(0);
		T[i * 3 + 1] = tvecNew(1);
		T[i * 3 + 2] = tvecNew(2);
	}

	return 1;
}

int CLocalCSConvert::POSQCFromNEDToECEF(int count, double* Q, double* C)
{
	double TransformMatrix[12] = { 0 };
	NEDToECEFMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector4d qvec = Eigen::Vector4d(Q[i * 4 + 0], Q[i * 4 + 1], Q[i * 4 + 2], Q[i * 4 + 3]);
		Eigen::Vector3d Cvec = Eigen::Vector3d(C[i * 3 + 0], C[i * 3 + 1], C[i * 3 + 2]);
		Eigen::Vector3d tvec = -QuaternionRotatePoint(qvec, Cvec);

		Eigen::Vector4d qvecNew = qvec;
		Eigen::Vector3d tvecNew = tvec;

		TransformPOS(Matrix, qvec, tvec, qvecNew, tvecNew);

		Eigen::Vector4d inv_qvecNew = InvertQuaternion(qvecNew);
		Eigen::Vector3d CvecNew = -QuaternionRotatePoint(inv_qvecNew, tvecNew);

		Q[i * 4 + 0] = qvecNew(0);
		Q[i * 4 + 1] = qvecNew(1);
		Q[i * 4 + 2] = qvecNew(2);
		Q[i * 4 + 3] = qvecNew(3);

		C[i * 3 + 0] = CvecNew(0);
		C[i * 3 + 1] = CvecNew(1);
		C[i * 3 + 2] = CvecNew(2);
	}

	return 1;
}

int CLocalCSConvert::POSQCFromECEFToNED(int count, double* Q, double* C)
{
	double TransformMatrix[12] = { 0 };
	ECEFToNEDMatrix(TransformMatrix);

	Eigen::Affine3d Matrix;
	Matrix(0, 0) = TransformMatrix[0]; Matrix(0, 1) = TransformMatrix[1]; Matrix(0, 2) = TransformMatrix[2]; Matrix(0, 3) = TransformMatrix[3];
	Matrix(1, 0) = TransformMatrix[4]; Matrix(1, 1) = TransformMatrix[5]; Matrix(1, 2) = TransformMatrix[6]; Matrix(1, 3) = TransformMatrix[7];
	Matrix(2, 0) = TransformMatrix[8]; Matrix(2, 1) = TransformMatrix[9]; Matrix(2, 2) = TransformMatrix[10]; Matrix(2, 3) = TransformMatrix[11];

	for (int i = 0; i < count; i++) {
		Eigen::Vector4d qvec = Eigen::Vector4d(Q[i * 4 + 0], Q[i * 4 + 1], Q[i * 4 + 2], Q[i * 4 + 3]);
		Eigen::Vector3d Cvec = Eigen::Vector3d(C[i * 3 + 0], C[i * 3 + 1], C[i * 3 + 2]);
		Eigen::Vector3d tvec = -QuaternionRotatePoint(qvec, Cvec);

		Eigen::Vector4d qvecNew = qvec;
		Eigen::Vector3d tvecNew = tvec;

		TransformPOS(Matrix, qvec, tvec, qvecNew, tvecNew);

		Eigen::Vector4d inv_qvecNew = InvertQuaternion(qvecNew);
		Eigen::Vector3d CvecNew = -QuaternionRotatePoint(inv_qvecNew, tvecNew);

		Q[i * 4 + 0] = qvecNew(0);
		Q[i * 4 + 1] = qvecNew(1);
		Q[i * 4 + 2] = qvecNew(2);
		Q[i * 4 + 3] = qvecNew(3);

		C[i * 3 + 0] = CvecNew(0);
		C[i * 3 + 1] = CvecNew(1);
		C[i * 3 + 2] = CvecNew(2);
	}

	return 1;
}

int CLocalCSConvert::Test(int argc, char* argv[])
{
	double D2R = 3.1415926535897932384626433832795 / 180.0;
	double R2D = 180.0 / 3.1415926535897932384626433832795;

	{
		double Lon = 114.629769972222;
		double Lat = 30.9348514166667;
		double Alt = 119.767;

		double GimbalRollDegree = 0;
		double GimbalPitchDegree = -60.00;
		double GimbalYawDegree = 145.90;

		double XYZ_ecef_Given[3] = { Lon, Lat, Alt };
		m_pCoorCvtWGS84ToCXYZ->Transform(1, &XYZ_ecef_Given[0], &XYZ_ecef_Given[1], &XYZ_ecef_Given[2]);

		double XYZ_NED[3] = { XYZ_ecef_Given[0], XYZ_ecef_Given[1], XYZ_ecef_Given[2] };
		PointFromECEFToNED(1, &XYZ_NED[0], &XYZ_NED[1], &XYZ_NED[2]);

		double GimbalRollRadian = GimbalRollDegree * D2R;
		double GimbalPitchRadian = GimbalPitchDegree * D2R;
		double GimbalYawRadian = GimbalYawDegree * D2R;

		const Eigen::Matrix3d Rx = RotateX(GimbalRollRadian);
		const Eigen::Matrix3d Ry = RotateY(GimbalPitchRadian);
		const Eigen::Matrix3d Rz = RotateZ(GimbalYawRadian);
		Eigen::Matrix3d R_Gimbal_2_NED = Rz * Ry * Rx;
		Eigen::Matrix3d R_NED_2_Gimbal = R_Gimbal_2_NED.transpose();

		Eigen::Matrix3d R_Gimbal_2_Cam;
		R_Gimbal_2_Cam(0, 0) = 0; R_Gimbal_2_Cam(0, 1) = 1; R_Gimbal_2_Cam(0, 2) = 0;
		R_Gimbal_2_Cam(1, 0) = 0; R_Gimbal_2_Cam(1, 1) = 0; R_Gimbal_2_Cam(1, 2) = 1;
		R_Gimbal_2_Cam(2, 0) = 1; R_Gimbal_2_Cam(2, 1) = 0; R_Gimbal_2_Cam(2, 2) = 0;

		Eigen::Matrix3d R_NED_2_CAM = R_Gimbal_2_Cam * R_NED_2_Gimbal;
		Eigen::Vector4d qvec_ned = RotationMatrixToQuaternion(R_NED_2_CAM);

		Eigen::Vector4d qvec_ecef = qvec_ned;
		double XYZ_ecef[3] = { XYZ_NED[0], XYZ_NED[1], XYZ_NED[2] };
		POSQCFromNEDToECEF(1, qvec_ecef.data(), XYZ_ecef);

		Eigen::Matrix3d R_ECEF_2_CAM = QuaternionToRotationMatrix(qvec_ecef);
		double Temp_ECEF_2_CAM[9] = {
			R_ECEF_2_CAM(0,0), R_ECEF_2_CAM(0,1), R_ECEF_2_CAM(0,2),
			R_ECEF_2_CAM(1,0), R_ECEF_2_CAM(1,1), R_ECEF_2_CAM(1,2),
			R_ECEF_2_CAM(2,0), R_ECEF_2_CAM(2,1), R_ECEF_2_CAM(2,2)
		};

		return 1;
	}

	{
		Eigen::Matrix3d R_ECEF_2_Cam;

		//100_0199_0030.JPG
		R_ECEF_2_Cam(0, 0) = 0.632619;  R_ECEF_2_Cam(0, 1) = 0.607068;  R_ECEF_2_Cam(0, 2) = -0.480897;
		R_ECEF_2_Cam(1, 0) = 0.773719; R_ECEF_2_Cam(1, 1) = -0.522630;  R_ECEF_2_Cam(1, 2) = 0.358074;
		R_ECEF_2_Cam(2, 0) = -0.033956;  R_ECEF_2_Cam(2, 1) = -0.598603;  R_ECEF_2_Cam(2, 2) = -0.800325;

		double Lon = 114.629769972222;
		double Lat = 30.9348514166667;
		double Alt = 119.767;

		double GimbalRollDegree = 0;
		double GimbalPitchDegree = -60.00;
		double GimbalYawDegree = 145.90;

		//100_0200_0001.JPG
		//R_ECEF_2_Cam(0, 0) = 0.103487;  R_ECEF_2_Cam(0, 1) = -0.513975;  R_ECEF_2_Cam(0, 2) = 0.851540;
		//R_ECEF_2_Cam(1, 0) = -0.928175; R_ECEF_2_Cam(1, 1) = -0.357594;  R_ECEF_2_Cam(1, 2) = -0.103038;
		//R_ECEF_2_Cam(2, 0) = 0.357465;  R_ECEF_2_Cam(2, 1) = -0.779715;  R_ECEF_2_Cam(2, 2) = -0.514065;

		//double Lon = 114.630640;
		//double Lat = 30.934852;
		//double Alt = 163.88;

		//double GimbalRollDegree = 0;
		//double GimbalPitchDegree = -90.00;
		//double GimbalYawDegree = -83.10;

		//100_0200_0057.JPG
		//R_ECEF_2_Cam(0, 0) = -0.326134;  R_ECEF_2_Cam(0, 1) = -0.619988;  R_ECEF_2_Cam(0, 2) = 0.713619;
		//R_ECEF_2_Cam(1, 0) = -0.364534; R_ECEF_2_Cam(1, 1) = -0.614027;  R_ECEF_2_Cam(1, 2) = -0.700061;
		//R_ECEF_2_Cam(2, 0) = 0.872211;  R_ECEF_2_Cam(2, 1) = -0.488452;  R_ECEF_2_Cam(2, 2) = -0.025751;

		//double Lon = 114.629798611111;
		//double Lat = 30.93439075;
		//double Alt = 163.831;

		//double GimbalRollDegree = 0;
		//double GimbalPitchDegree = -44.9;
		//double GimbalYawDegree = -56.3;

		Eigen::Vector4d qvec_ecef = RotationMatrixToQuaternion(R_ECEF_2_Cam);
		double XYZ_ecef[3] = { Lon, Lat, Alt };
		m_pCoorCvtWGS84ToCXYZ->Transform(1, &XYZ_ecef[0], &XYZ_ecef[1], &XYZ_ecef[2]);

		Eigen::Vector4d qvec_enu = qvec_ecef;
		double XYZ_enu[3] = { XYZ_ecef[0], XYZ_ecef[1], XYZ_ecef[2] };
		POSQCFromECEFToENU(1, qvec_enu.data(), XYZ_enu);

		Eigen::Matrix3d R_ENU_2_Cam_Truth = QuaternionToRotationMatrix(qvec_enu);

		double Temp_ENU_2_Cam_Truth[9] = {
			R_ENU_2_Cam_Truth(0,0), R_ENU_2_Cam_Truth(0,1), R_ENU_2_Cam_Truth(0,2),
			R_ENU_2_Cam_Truth(1,0), R_ENU_2_Cam_Truth(1,1), R_ENU_2_Cam_Truth(1,2),
			R_ENU_2_Cam_Truth(2,0), R_ENU_2_Cam_Truth(2,1), R_ENU_2_Cam_Truth(2,2)
		};

		Eigen::Matrix3d NED_2_ENU;
		NED_2_ENU(0, 0) = 0; NED_2_ENU(0, 1) = 1; NED_2_ENU(0, 2) = 0;
		NED_2_ENU(1, 0) = 1; NED_2_ENU(1, 1) = 0; NED_2_ENU(1, 2) = 0;
		NED_2_ENU(2, 0) = 0; NED_2_ENU(2, 1) = 0; NED_2_ENU(2, 2) = -1;

		Eigen::Matrix3d R_NED_2_CAM_Truth = R_ENU_2_Cam_Truth * NED_2_ENU;

		double Temp_NED_2_CAM_Truth[9] = {
			R_NED_2_CAM_Truth(0,0), R_NED_2_CAM_Truth(0,1), R_NED_2_CAM_Truth(0,2),
			R_NED_2_CAM_Truth(1,0), R_NED_2_CAM_Truth(1,1), R_NED_2_CAM_Truth(1,2),
			R_NED_2_CAM_Truth(2,0), R_NED_2_CAM_Truth(2,1), R_NED_2_CAM_Truth(2,2)
		};

		//////////////////////////////////////////////////

		double GimbalRollRadian = GimbalRollDegree * D2R;
		double GimbalPitchRadian = GimbalPitchDegree * D2R;
		double GimbalYawRadian = GimbalYawDegree * D2R;

		const Eigen::Matrix3d Rx = RotateX(GimbalRollRadian);
		const Eigen::Matrix3d Ry = RotateY(GimbalPitchRadian);
		const Eigen::Matrix3d Rz = RotateZ(GimbalYawRadian);
		Eigen::Matrix3d R_Gimbal_2_NED = Rz * Ry * Rx;
		Eigen::Matrix3d R_NED_2_Gimbal = R_Gimbal_2_NED.transpose();

		Eigen::Matrix3d R_Gimbal_2_Cam;
		R_Gimbal_2_Cam(0, 0) = 0; R_Gimbal_2_Cam(0, 1) = 1; R_Gimbal_2_Cam(0, 2) = 0;
		R_Gimbal_2_Cam(1, 0) = 0; R_Gimbal_2_Cam(1, 1) = 0; R_Gimbal_2_Cam(1, 2) = 1;
		R_Gimbal_2_Cam(2, 0) = 1; R_Gimbal_2_Cam(2, 1) = 0; R_Gimbal_2_Cam(2, 2) = 0;

		Eigen::Matrix3d R_NED_2_CAM = R_Gimbal_2_Cam * R_NED_2_Gimbal;

		double Temp_Gimbal_2_NED[9] = {
			R_Gimbal_2_NED(0,0), R_Gimbal_2_NED(0,1), R_Gimbal_2_NED(0,2),
			R_Gimbal_2_NED(1,0), R_Gimbal_2_NED(1,1), R_Gimbal_2_NED(1,2),
			R_Gimbal_2_NED(2,0), R_Gimbal_2_NED(2,1), R_Gimbal_2_NED(2,2)
		};

		double Temp_NED_2_Gimbal[9] = {
			R_NED_2_Gimbal(0,0), R_NED_2_Gimbal(0,1), R_NED_2_Gimbal(0,2),
			R_NED_2_Gimbal(1,0), R_NED_2_Gimbal(1,1), R_NED_2_Gimbal(1,2),
			R_NED_2_Gimbal(2,0), R_NED_2_Gimbal(2,1), R_NED_2_Gimbal(2,2)
		};

		double Temp_NED_2_CAM[9] = {
			R_NED_2_CAM(0,0), R_NED_2_CAM(0,1), R_NED_2_CAM(0,2),
			R_NED_2_CAM(1,0), R_NED_2_CAM(1,1), R_NED_2_CAM(1,2),
			R_NED_2_CAM(2,0), R_NED_2_CAM(2,1), R_NED_2_CAM(2,2)
		};

		return 1;
	}
	return 1;
}


