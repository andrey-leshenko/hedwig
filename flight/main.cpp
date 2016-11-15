#include <opencv2/viz.hpp>
#include <unistd.h>

#include "config.hpp"
#include "camera_tracking.hpp"
#include "pid.hpp"
#include "arduino/arduino_serial.hpp"

using cv::FileStorage;

std::string type2str(int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
		case CV_8U: r = "8U"; break;
		case CV_8S: r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default: r = "User"; break;
	}

	r += "C";
	r += (chans+'0');

	return r;
}

bool pointInImage(Point2d p, int xFloor = 0, int xCeiling = 650, int yFloor = 0, int yCeiling = 500)
{
	return (xFloor <= p.x) && (p.x <= xCeiling) && (yFloor <= p.y) && (p.y <= yCeiling);
}

Point3f centroid(const vector<Point3f>& points)
{
	Vec3f sum;
	for (Point3f p : points) {
		sum += Vec3f{p};
	}
	int count = points.size();
	return sum / count;
}

Mat createPointMatrix(const vector<Point3f>& points)
{
	int pointCount = static_cast<int>(points.size());
	Mat m{3, pointCount, CV_32F};

	for (int i = 0; i < pointCount; i++) {
		m.at<float>(0,i) = points[i].x;
		m.at<float>(1,i) = points[i].y;
		m.at<float>(2,i) = points[i].z;
	}
	return m;
}

vector<Point3f> pointsTranslate(const vector<Point3f>& points, Point3f v)
{
	vector<Point3f> result(points.size());

	for (int i = 0; i < points.size(); i++) {
		result[i].x = points[i].x + v.x;
		result[i].y = points[i].y + v.y;
		result[i].z = points[i].z + v.z;
	}

	return result;
}

void addCamerasToViz(CameraData &cameras, cv::viz::Viz3d &window)
{
	const char *names[] = {"__c0", "__c1", "__c2", "__c3"};

	CV_Assert(cameras.cameraCount <= (sizeof names / sizeof *names));

	for (int i = 0; i < cameras.cameraCount; i++) {
		cv::viz::WCameraPosition camWidget{cv::Matx33d{cameras.cameraMatrixes[i]}, 10};
		window.showWidget(names[i], camWidget);
		window.setWidgetPose(names[i], cameras.transforms[i]);
	}
}

Mat svdRotation(Mat &initialPointsCenteredMat, Mat &currPointsCenteredMat)
{
	Mat covarianceMatrix{initialPointsCenteredMat * currPointsCenteredMat.t()};
	Mat u, s, vt;

	cv::SVD::compute(covarianceMatrix, s, u, vt); // cov = u * s * vt
	Mat rot = vt.t() * u.t();

	if (cv::determinant(rot) < 0)
	{
		// Nick Lambert was wrong
		Mat d = Mat::eye(3, 3, rot.type());
		d.col(2) *= -1;
		rot = vt.t() * d * u.t();
	}

	return rot;
}

Vec4f calculateControlErrors(Vec3f currPos, Mat currRotation, Vec3f targetPos)
{
	// The world axes are (from above):
	//
	// *-----> X
	// |
	// |
	// V
	// Z
	//
	// atan2 expects:
	//
	// Y
	// ^
	// |
	// |
	// *-----> X
	//
	// Also, atan2 calculates rotation angle relative to X
	// while we want it relative to Z (-Z is forward).
	// So Z becomes the new X, and X becomes the new Y.

	float zx = currRotation.at<float>(0, 2);
	float zz = currRotation.at<float>(2, 2);
	float atanY = zx;
	float atanX = zz;

	float yawCCW = std::atan2(atanY, atanX);

	Mat yawRotation = (cv::Mat_<float>(3,3) <<
			cos(yawCCW),  0, sin(yawCCW),
			0,            1, 0,
			-sin(yawCCW), 0, cos(yawCCW));

	auto rotatePoint3f = [](Point3f p, const Mat& m) {
		Mat mp{p, false};
		mp = m * mp;
		return p;
	};

	Point3f droneWorldForward{0, 0, -1};
	Point3f droneWorldRight{1, 0, 0};
	Point3f droneWorldUp{0, 1, 0};

	droneWorldForward = rotatePoint3f(droneWorldForward, yawRotation);
	droneWorldRight = rotatePoint3f(droneWorldRight, yawRotation);
	droneWorldUp = rotatePoint3f(droneWorldUp, yawRotation);

	Point3f target = targetPos - currPos;

	Vec4f errors {
		target.dot(droneWorldRight),
		target.dot(droneWorldForward),
		target.dot(droneWorldUp),
		-yawCCW
	};

	return errors;
}

int lerp(float t, int a, int b)
{
	return (int)((1 - t) * a + t * b);
}

int transformIntoChannel(float normalizedValue, ChannelBounds bounds)
{
	float t = normalizedValue;
	if (t >= 0) {
		return lerp(t, bounds.zero, bounds.max);
	}
	else {
		return lerp(-t, bounds.zero, bounds.min);
	}
}

int sendToDrone(int serialfd, int channels[4])
{
	for (int i = 0; i < 4; i++) {
//		std::cout << channels[i] << " ";
	}
//	std::cout << std::endl;

	if (serialfd < 0) {
		return -1;
	}

	unsigned char cmd[5] = {0xff, 0, 0, 0, 0};

	cmd[1] = channels[0];
	cmd[2] = channels[1];
	cmd[3] = channels[2];
	cmd[4] = channels[3];

	if (write(serialfd, cmd, 5) != 5)
	{
		return -1;
	}
	return 0;
}

struct KeyboardState
{
	unsigned char keyPressed[256];
	unsigned char keyDown[256];
	int modifiers;
};

void keyboardClearDownKeys(KeyboardState *keyboard)
{
	memset(keyboard->keyDown, 0, 256);
}

void vizUpdateKeyboardStateCallback(const cv::viz::KeyboardEvent &e, void* keyboardState)
{
	KeyboardState *state = (KeyboardState*)keyboardState;

	state->keyPressed[e.code] = e.action;
	state->keyDown[e.code] = e.action;
	state->modifiers = e.modifiers;
}

static int connectToSerial(const vector<String> &possibleDevices)
{
	int serialfd = -1;

	for (const String &s : possibleDevices) {
		serialfd = arduino_serial_open(s.c_str(), 115200);
		if (serialfd >= 0)
			break;
	}

	return serialfd;
}

static int connectToSerialVerbose(const vector<String> &possibleDevices)
{
	int serialfd = connectToSerial(possibleDevices);

	if (serialfd < 0)
	{
		printf("ERROR: couldn't connect to any of the serial devices specified.\n"
				"Commands WILL NOT be sent.\n"
				"\n"
				"Make sure your device is connected.\n"
				"\n"
				"Devices checked:\n");
		for (const String &s : possibleDevices) {
			printf("		%s\n", s.c_str());
		}
		printf("\n"
				"To add devices to this list, update the `possible_serial_devices` variable and recompile.\n"
				"\n"
				"press ENTER to continue");
		getchar();
	}

	return serialfd;
}

void displayCamerasRange(CameraData &cameras, vector<Point3d> &displayCluster) {

	float edge[3] = {220, 180, 220};
	float shift[3] = {-140, 0, -110};
	int density[3] = {20, 20, 20};

	Mat homoSamplingCluster{4, density[0] * density[1] * density[2], CV_64F};
	int col = 0;
	for(int i = 0; i < density[0]; i++)
		for(int j = 0; j < density[1]; j++)
			for(int k = 0; k < density[2]; k++)
			{
				homoSamplingCluster.at<double>(0, col) = (edge[0] / density[0]) * i + shift[0];
				homoSamplingCluster.at<double>(1, col) = (edge[1] / density[1]) * j + shift[1];
				homoSamplingCluster.at<double>(2, col) = (edge[2] / density[2]) * k + shift[2];
				homoSamplingCluster.at<double>(3, col) = 1;
				col++;
			}

//	std::cout << homoSamplingCluster << std::endl;

	vector<Mat> homoImageCluster;

	for(vector<Mat>::iterator projIter = cameras.projectionMatrixes.begin(); projIter != cameras.projectionMatrixes.end(); ++projIter)
	{
		homoImageCluster.push_back((*projIter) * homoSamplingCluster);
	}

//	for(vector<Mat>::iterator it = homoImageCluster.begin(); it != homoImageCluster.end(); ++it)
//		std::cout << (*it) << std::endl;

	vector<Mat> imageCluster;

	for(vector<Mat>::iterator homoIter = homoImageCluster.begin(); homoIter != homoImageCluster.end(); ++homoIter)
	{
		Mat temp{};
		convertPointsFromHomogeneous((*homoIter).t(), temp);
		imageCluster.push_back(temp);
	}

//	for(vector<Mat>::iterator it = imageCluster.begin(); it != imageCluster.end(); ++it)
//		std::cout << (*it) << std::endl;

	vector<bool> inImage(density[0] * density[1] * density[2], true);

//	for(vector<bool>::iterator it = inImage.begin(); it != inImage.end(); ++it)
//		std::cout << (*it) << std::endl;

	for(vector<Mat>::iterator it = imageCluster.begin(); it != imageCluster.end(); ++it)
	{
		vector<bool>::iterator boolIt = inImage.begin();
		for(int i = 0; i < density[0] * density[1] * density[2]; i++, ++boolIt)
		{
			(*boolIt) = (*boolIt) & pointInImage(Point2d{(*it).at<double>(i, 0), (*it).at<double>(i, 1)});
		}
	}

//	for(vector<bool>::iterator it = inImage.begin(); it != inImage.end(); ++it)
//		std::cout << (*it) << std::endl;


	int iter = 0;
	for(vector<bool>::iterator boolIt = inImage.begin(); boolIt != inImage.end(); ++boolIt, iter++)
		if(*boolIt)
		{
			displayCluster.push_back(Point3d{
			homoSamplingCluster.at<double>(0, iter),
			homoSamplingCluster.at<double>(1, iter),
			homoSamplingCluster.at<double>(2, iter)});
		}

//	for(vector<Point3d>::iterator it = displayCluster.begin(); it != displayCluster.end(); ++it)
//		std::cout << (*it) << std::endl;


}

int main(int argc, char *argv[])
{
	Config cfg;

	{
		auto file = "../config.yaml";

		bool success = loadConfigFromFile(cfg, file);
		if (!success) {
			std::cerr << "ERROR: Couldn't read config file `" << file << "`." << std::endl;
			return 1;
		}

		std::cout << "Fetched the following config:" << std::endl;
		printConfig(cfg);
		std::cout << std::endl;
	}

	int serialfd = connectToSerialVerbose(cfg.dronePossibleSerialDevices);

	CameraData cameras;
	setupCameras(cameras, cfg);

	if (argc > 1)
	{
		doExternalCalibrationInteractive(cameras, cfg.calibrationChessboardSize, cfg.calibrationChessboardSquareSize);
		saveExternalCalibrationToFile(cameras, "external_calibration.yaml");
	}
	if (loadExternalCalibrationFromFile(cameras, "external_calibration.yaml")) {
		std::cout << "Successfully loaded external calibration." << std::endl;
	}

	vector<Point3f> initialPoints;

	while (!triangulateChessboardPoints(initialPoints, cameras, cfg.droneChessboardSize)) {
		std::cout << "Couldn't find drone, retrying..." << std::endl;
	}

	Point3f initialPosition = centroid(initialPoints);
	vector<Point3f> initialPointsCentered = pointsTranslate(initialPoints, -initialPosition);
	Mat initialPointsCenteredMat = createPointMatrix(initialPointsCentered);

	vector<Point3f> currPoints;
	Point3f currPosition;
	vector<Point3f> currPointsCentered;
	Mat currPointsCenteredMat;

	cv::viz::Viz3d window{"Flight Control"};

	KeyboardState keyboard = {0};
	window.registerKeyboardCallback(&vizUpdateKeyboardStateCallback, &keyboard);


	addCamerasToViz(cameras, window);
	window.showWidget("floor", cv::viz::WPlane{Point3d{0, 0, 0}, Vec3d{0, 1, 0}, Vec3d{1, 0, 0}, cv::Size2d{20,20}, cv::viz::Color::silver()});
	window.showWidget("axes", cv::viz::WCoordinateSystem{20});
	window.showWidget("drone", cv::viz::WCube{Point3f{-10, -2, -10}, Point3f{10, 2, 10}, true});
	window.showWidget("drone_direction", cv::viz::WArrow{Point3f{0, 0, 0}, Point3f{0, 0, -20}});
	vector<Point3d> displayCluster;
	displayCamerasRange(cameras, displayCluster);
<<<<<<< Updated upstream
	//window.showWidget("range", cv::viz::WCloud{displayCluster, cv::viz::Color::yellow()});
=======
	window.showWidget("range", cv::viz::WCloud{displayCluster, cv::viz::Color::yellow()});

	Vec3f target = {20, 20, 20};
	window.showWidget("target_point", cv::viz::WSphere{Point3f{target}, 3, 10, cv::viz::Color::violet()});
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
	ChannelBounds channels[4];
	Pid pids[4];
	s64 lastFrameTickCount = cv::getTickCount();

	for (int i = 0; i < 4; i++) {
		PIDParameters param = cfg.dronePIDParameters[i];
		pids[i] = Pid{param.kp, param.ki, param.kd};
		pids[i].setLimits(-1, 1);

		channels[i] = cfg.droneChannels[i];
	}

	while (!window.wasStopped()) {
		bool found = triangulateChessboardPoints(currPoints, cameras, cfg.droneChessboardSize);

		if (found)
		{
			currPosition = centroid(currPoints);
			currPointsCentered = pointsTranslate(currPoints, -currPosition);
			currPointsCenteredMat = createPointMatrix(currPointsCentered);

			Vec3f pos = Vec3f{currPosition} - Vec3f{initialPosition};
			Mat rot = svdRotation(initialPointsCenteredMat, currPointsCenteredMat);

			Affine3f currTransform {rot, pos};

			Vec4f controlErrors = calculateControlErrors(pos, rot, target);

			Vec4f pidedErrors;
			float deltaTime = (float)(cv::getTickCount() - lastFrameTickCount) / cv::getTickFrequency();
			lastFrameTickCount = cv::getTickCount();

			for (int i = 0; i < 4; i++) {
				pidedErrors[i] = pids[i].calculate(controlErrors[i], deltaTime);
			}

			int sentValues[4];

			for (int i = 0; i < 4; i++) {
				sentValues[i] = transformIntoChannel(pidedErrors[i], channels[i]);
			}

			sendToDrone(serialfd, sentValues);

			Point3f pidViz;
			pidViz.x = lerp(pidedErrors[0], 0, 40) + target[0];
			pidViz.y = lerp(pidedErrors[1], 0, 40) + target[1];
			pidViz.z = lerp(pidedErrors[2], 0, 40) + target[2];

			window.showWidget("pid_arrow", cv::viz::WArrow{Point3f{target}, pidViz, 0.03, cv::viz::Color::cherry()});
			window.showWidget("chessboard", cv::viz::WCloud{currPoints});
			window.setRenderingProperty("chessboard", cv::viz::POINT_SIZE, 4);

			window.setWidgetPose("drone", currTransform);
			window.setWidgetPose("drone_direction", currTransform);
		}

		if (found) {
			// NOTE(Andrey): Default Viz background
			window.setBackgroundColor(cv::viz::Color{2, 1, 1}, cv::viz::Color{240, 120, 120});
		}
		else {
			window.setBackgroundColor(cv::viz::Color{35, 35, 255}, cv::viz::Color{0, 0, 255});
		}

		if (keyboard.keyDown['c']) {
			doExternalCalibrationInteractive(cameras, cfg.calibrationChessboardSize, cfg.calibrationChessboardSquareSize);
			addCamerasToViz(cameras, window);
			saveExternalCalibrationToFile(cameras, "external_calibration.yaml");
		}

		keyboardClearDownKeys(&keyboard);
		window.spinOnce();
	}

	if (serialfd >= 0) {
		arduino_serial_close(serialfd);
	}

	return 0;
}
