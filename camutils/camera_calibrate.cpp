#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

using std::vector;

using cv::VideoCapture;
using cv::Mat;
using cv::Point2f;
using cv::Point3f;
using cv::Size;
using cv::FileStorage;

const cv::String windowName{"Calibrate camera"};

void usage()
{
	std::cout << "Usage: camera_calibrate CAMERA_INDEX GRID_SIZE [SQUARE_SIZE] [OUTPUT_FILE]\n"
		"\n"
		"Find the camera parameters of camera number CAMERA_INDEX\n"
		"using chessboard calibration.\n"
		"\n"
		"First, take photos of the chessboard by pressing the space key.\n"
		"After you take enough photos press 'n', and you will be able to view\n"
		"the captured images by pressing any key. When you reach the last image,\n"
		"the calibration data will be printed to the screen and saved to a file.\n"
		"You can then view the images again after the undistort operation.\n"
		"Press 'q' to abort at any time.\n";
}

int main(int argc, char* argv[])
{
	int cameraIndex;
	Size chessboardSize;
	float chessSquareSize = 1;
	const char* outputFile = "default_cam_calib.yaml";

	//////// Parse arguments ////////

	if (argc - 1 < 2 || argc - 1 > 4) {
		usage();
		return 1;
	}

	cameraIndex = std::atoi(argv[1]);

	if (sscanf(argv[2], "%dx%d", &chessboardSize.width, &chessboardSize.height) != 2) {
		usage();
		return 1;
	}

	if (3 < argc) {
		chessSquareSize = std::atof(argv[3]);
	}
	if (4 < argc) {
		outputFile = argv[4];
	}

	//////// Capture images ////////

	VideoCapture cap{cameraIndex};

	if (!cap.isOpened()) {
		std::cerr << "error: couldn't capture camera number " << cameraIndex << std::endl;
		return 1;
	}

	vector<Mat> capturedFrames;

	int pressedKey = 0;
	Mat currFrame, currFrameFlipped;

	while (pressedKey != 'n') {
		cap >> currFrame;

		cv::flip(currFrame, currFrameFlipped, 1);
		cv::imshow(windowName, currFrameFlipped);
		pressedKey = cv::waitKey(1);

		if (pressedKey == ' ') {
			capturedFrames.push_back(currFrame.clone());

			cv::setWindowTitle(windowName, windowName + " (" + std::to_string(capturedFrames.size()) + ")");

			cv::threshold(currFrameFlipped, currFrameFlipped, 70, 255, CV_THRESH_BINARY_INV);
			cv::imshow(windowName, currFrameFlipped);
			cv::waitKey(60);
		}
		else if (pressedKey == 'q') {
			return 0;
		}
	}

	cv::setWindowTitle(windowName, windowName);

	//////// Detect chessboards ////////

	vector<vector<Point3f>> objectPoints;
	vector<vector<Point2f>> imagePoints;

	vector<Point3f> chessboardPoints;

	for (int y = 0; y < chessboardSize.height; y++)	{
		for(int x = 0; x < chessboardSize.width; x++) {
			chessboardPoints.push_back(Point3f{x * chessSquareSize, y * chessSquareSize, 0});
		}
	}

	for (Mat frame : capturedFrames) {
		vector<Point2f> corners;
		bool found = cv::findChessboardCorners(
				frame,
				chessboardSize,
				corners,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);

		if (found) {
			objectPoints.push_back(chessboardPoints);
			imagePoints.push_back(corners);
		}

		cv::drawChessboardCorners(frame, chessboardSize, corners, found);
		cv::imshow(windowName, frame);
		if (cv::waitKey(0) == 'q')
			return 0;

	}

	if (imagePoints.size() == 0) {
		return 0;
	}

	//////// Calibrate camera ////////

	Mat firstFrame = capturedFrames.front();
	Mat cameraMatrix, distCoeffs;
	vector<Mat> rvecs, tvecs;

	double reprojectionError = calibrateCamera(
		objectPoints,
		imagePoints,
		Size{firstFrame.cols, firstFrame.rows},
		cameraMatrix,
		distCoeffs,
		rvecs,
		tvecs,
		0);

	//////// Write output file ////////
	
	{
		FileStorage fs{outputFile, FileStorage::WRITE};
		fs << "cameraMatrix" << cameraMatrix;
		fs << "distCoeffs" << distCoeffs;
	}

	//////// Display results ////////

	std::cout
		<< "reprojection error: " << reprojectionError << std::endl
		<< "camera matrix:\n" << cameraMatrix << std::endl
		<< "distortion coefficients:\n" << distCoeffs << std::endl;

	for (Mat frame : capturedFrames) {
		Mat m;
		cv::undistort(frame, m, cameraMatrix, distCoeffs);
		cv::imshow(windowName, m);
		if (cv::waitKey(0) == 'q')
			return 0;
	}

	return 0;
}
