#include "camera_tracking.hpp"

#include "chessboard.hpp"

static void captureCameraFrames(vector<VideoCapture>& cameras, vector<Mat>& frames)
{
	for (int i = 0; i < frames.size(); i++) {
		cameras[i].grab();
	}
	for (int i = 0; i < frames.size(); i++) {
		cameras[i].retrieve(frames[i]);
	}
}

static bool findChessboards(vector<Mat> images, Size chessboardSize, vector<vector<Point2f>>& pointsOut)
{
	for (int i = 0; i < images.size(); i++) {
		// Our brand new chessboard detection algorithm
		bool found = findChessboardSquares(
				images[i],
				chessboardSize,
				pointsOut[i],
				0);
				//CV_CALIB_CB_ADAPTIVE_THRESH);

		if (!found) {
			return false;
		}
	}
	return true;
}

static vector<Point3f> constructXZChessboard(Size chessboardSize, float squareSize)
{
	vector<Point3f> chessPoints;

	for (int z = 0; z < chessboardSize.height; z++)	{
		for(int x = 0; x < chessboardSize.width; x++) {
			chessPoints.push_back(Point3f{x * squareSize, 0, z * squareSize});
		}
	}

	return chessPoints;
}

bool doExternalCalibration(CameraData &cameras, Size chessboardSize, float squareSize)
{
	cameras.projectionMatrixes.resize(cameras.cameraCount);
	cameras.transforms.resize(cameras.cameraCount);

	captureCameraFrames(cameras.captures, cameras.frames);

	if (!findChessboards(cameras.frames, chessboardSize, cameras.imagePoints)) {
		return false;
	}

	vector<Mat> rvecs(cameras.cameraCount);
	vector<Mat> tvecs(cameras.cameraCount);

	vector<Point3f> chessboardPoints = constructXZChessboard(chessboardSize, squareSize);


	for (int i = 0; i < cameras.frames.size(); i++) {
		bool found = cv::solvePnP(chessboardPoints,
				cameras.imagePoints[i],
				cameras.cameraMatrixes[i],
				Mat{},
				rvecs[i],
				tvecs[i]);

		if (!found) {
			return false;
		}
	}
	
	for (int i = 0; i < cameras.frames.size(); i++) {
		Affine3d transform{rvecs[i], tvecs[i]};
		Mat rtMatrix = Mat{transform.matrix}.rowRange(0, 3);
		cameras.projectionMatrixes[i] = cameras.cameraMatrixes[i] * rtMatrix;
		// We found how to transform points so a camera placed
		// at the origin will see them. Assuming that points were at
		// the origin in the first place, the location of the camera
		// is the inverse of that transformation.
		cameras.transforms[i] = transform.inv();
	}


	return true;
}

bool doExternalCalibrationInteractive(CameraData &cameras, Size chessboardSize, float squareSize)
{
	int currCamera = 0;
	int cameraCount = cameras.cameraCount;

	bool inspecting = true;
	bool success = false;

	while (inspecting) {
		captureCameraFrames(cameras.captures, cameras.frames);

		Mat display;

		if (cameraCount == 2) {
			cv::hconcat(cameras.frames[0], cameras.frames[1], display);
		}
		else {
			display = cameras.frames[currCamera];
		}

		cv::imshow("External Calibration", display);

		int pressedKey = cv::waitKey(1);

		switch (pressedKey)
		{
			case 'n':
			case ' ':
			case '\r':
				success = doExternalCalibration(cameras, chessboardSize, squareSize);

				if (success) {
					inspecting = false;
				}
				else {
					std::cerr << "ERROR: External calibration failed." << std::endl;

					display.setTo(Scalar{0, 0, 255});
					cv::imshow("External Calibration", display);
					cv::waitKey(100);
				}
				break;
			case 'q':
				success = false;
				inspecting = false;
				break;
			case 'j':
				currCamera = (currCamera + 1) % cameraCount;
				break;
			case 'k':
				currCamera = (currCamera - 1 + cameraCount) % cameraCount;
				break;
		}
	}

	cv::destroyWindow("External Calibration");

	return success;
}

bool triangulateChessboardPoints(vector<Point3f> &points, CameraData &cameras, Size chessboardSize)
{
	CV_Assert(cameras.cameraCount == 2);

	captureCameraFrames(cameras.captures, cameras.frames);

	if (!findChessboards(cameras.frames, chessboardSize, cameras.imagePoints)) {
		return false;
	}

	// XXX: Shouldn't be allocated on each call
	Mat homogeneous;

	cv::triangulatePoints(
			cameras.projectionMatrixes[0],
			cameras.projectionMatrixes[1],
			cameras.imagePoints[0],
			cameras.imagePoints[1],
			homogeneous);

	// cv::convertPointsFromHomogeneous won't accept a vector of points
	// or the matrix before transposition.
	cv::convertPointsFromHomogeneous(homogeneous.t(), points);

	return true;
}

bool setupCameras(CameraData &cameras, Config &cfg)
{
	cameras.cameraCount = cfg.cameraCount;
	cameras.indexes = cfg.cameraIndexes;
	cameras.captures = vector<VideoCapture>{};

	for (int i : cameras.indexes) {
		cameras.captures.push_back(VideoCapture{i});
		if (!cameras.captures.back().isOpened()) {
			std::cerr << "Couldn't open camera at index " << i << std::endl;
			return false;
		}
		cameras.captures.back().set(cv::CAP_PROP_FPS, cfg.cameraTargetFPS);
	}

	cameras.cameraMatrixes.resize(cameras.cameraCount);

	for (int i = 0; i < cameras.cameraCount; i++) {
		cameras.cameraMatrixes[i] = cfg.cameraMatrix;
	}

	cameras.projectionMatrixes.resize(cameras.cameraCount);
	cameras.transforms.resize(cameras.cameraCount);

	for (int i = 0; i < cameras.cameraCount; i++) {
		cameras.projectionMatrixes[i] = Mat::eye(3, 4, CV_64F);
		cameras.transforms[i] = Affine3f{};
	}

	cameras.frames.resize(cameras.cameraCount);
	cameras.imagePoints.resize(cameras.cameraCount);

	return true;
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
