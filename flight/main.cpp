#include <opencv2/viz.hpp>
#include <unistd.h>

#include "config.hpp"
#include "camera_tracking.hpp"
#include "pid.hpp"
#include "arduino/arduino_serial.hpp"

using cv::FileStorage;

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

void saveExternalCalibrationToFile(CameraData &cameras, const char *file)
{
	FileStorage fs(file, FileStorage::WRITE);

	fs << "projection_matrixes" << "[";
	for (Mat &m : cameras.projectionMatrixes) {
		fs << m;
	}
	fs << "]";

	fs << "transforms" << "[";
	for (auto &t : cameras.transforms) {
		fs << Mat{t.matrix};
	}
	fs << "]";
}

bool loadExternalCalibrationFromFile(CameraData &cameras, const char *file)
{
	FileStorage fs(file, FileStorage::READ);

	if (!fs.isOpened())
	{
		return false;
	}

	fs["projection_matrixes"] >> cameras.projectionMatrixes;

	vector<Mat> affineTransforms;
	fs["transforms"] >> affineTransforms;

	cameras.transforms.resize(cameras.cameraCount);
	for (int i = 0; i < cameras.cameraCount; i++) {
		cameras.transforms[i] = Affine3d{Affine3d::Mat4{affineTransforms[i]}};
	}

	return true;
}

Vec4f calculateControlErrors(Vec3f currPos, Mat currRotation, Vec3f targetPos)
{
    float   yawError; // in radians
    Vec2f   currPosCenteredProj,
            targetPosCenteredProj;
    Vec4f   posError;

	// TODO: Find the yaw from the forward vector

    float r31 = currRotation.at<float>(2, 0);
    float r32 = currRotation.at<float>(2, 1);
    float r33 = currRotation.at<float>(2, 2);

    yawError = atan2(-r31, sqrt(r32 * r32 + r33 * r33));

    Mat yRotation = (cv::Mat_<float>(3,3) <<
                        cos(yawError) , 0       , sin(yawError),
                        0             , 1       , 0            ,
                        -sin(yawError), 0       , cos(yawError));

    yRotation = yRotation.inv();
    posError[1] = targetPos[1] - currPos[1];
    posError[3] = yawError;

    Mat targetPosRotated = yRotation * Mat(targetPos);
    Mat currPosRotated = yRotation * Mat(currPos);

    currPosCenteredProj =   {  currPosRotated.at<float>(0,0) ,
                                    currPosRotated.at<float>(2,0) };
    targetPosCenteredProj = {  targetPosRotated.at<float>(0,0) ,
                                    targetPosRotated.at<float>(2,0) };

    targetPosCenteredProj -= currPosCenteredProj;
    currPosCenteredProj -= currPosCenteredProj;

    posError[0] = targetPosCenteredProj[0] - currPosCenteredProj[0];
    posError[2] = targetPosCenteredProj[1] - currPosCenteredProj[1];

	Vec4f finalError{posError[0], -posError[2], posError[1], yawError};

    return finalError;
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
		std::cout << channels[i] << " ";
	}
	std::cout << std::endl;

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
	}

	int serialfd = -1;

	{
		for (String &s : cfg.dronePossibleSerialDevices) {
			serialfd = arduino_serial_open(s.c_str(), 115200);
			if (serialfd >= 0)
				break;
		}

		if (serialfd < 0)
		{
			printf("ERROR: couldn't connect to any of the serial devices specified.\n"
					"Commands WILL NOT be sent.\n"
					"\n"
					"Make sure your device is connected.\n"
					"\n"
					"Devices checked:\n");
			for (String &s : cfg.dronePossibleSerialDevices) {
				printf("    %s\n", s.c_str());
			}
			printf("\n"
					"To add devices to this list, update the `possible_serial_devices` variable and recompile.\n"
					"\n"
					"press ENTER to continue");
			getchar();
		}
	}

	CameraData cameras;

	setupCameras(cameras, cfg);

	bool loadedExternalCalibration = false;

	if (argc - 1 != 0) {
		if (argv[1][0] == 'l' && argv[1][1] == 0) {
			loadedExternalCalibration = loadExternalCalibrationFromFile(cameras, "external_calibration.yaml");
		}
		else {
			std::cerr << "ERROR: Unkown arguments." << std::endl;
			return -1;
		}
	}

	if (!loadedExternalCalibration) {
		bool ret = doExternalCalibrationInteractive(cameras, cfg.calibrationChessboardSize, cfg.calibrationChessboardSquareSize);
		if (!ret) {
			std::cerr << "Error: Couldn't do external calibration" << std::endl;
			return -1;
		}
		// XXX: Save only when requested
		saveExternalCalibrationToFile(cameras, "external_calibration.yaml");
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
	window.showWidget("axes", cv::viz::WCoordinateSystem{20});
	window.showWidget("drone", cv::viz::WCube{Point3f{-10, -2, -10}, Point3f{10, 2, 10}, true});
	window.showWidget("drone_direction", cv::viz::WArrow{Point3f{0, 0, 0}, Point3f{0, 0, -20}});

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

			Vec3f target = {0, 50, 0};

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
		}

		keyboardClearDownKeys(&keyboard);
		window.spinOnce();
	}

	if (serialfd >= 0) {
		arduino_serial_close(serialfd);
	}

	return 0;
}
