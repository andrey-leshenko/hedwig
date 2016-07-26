#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using std::cerr;
using std::cout;
using std::endl;
using std::vector;

using cv::VideoCapture;
using cv::Mat;
using cv::Point2f;
using cv::Point3f;
using cv::Size;

const Size chessboardSize{4, 3};

void usage()
{
	std::cout << "Usage: callibrate-camera [CAMERA_INDEX]\n"
		<< '\n'
		<< "Find the camera parameters of camera number CAMERA_INDEX.\n"
		<< "If no camera index is specified, CAMERA_INDEX=0 is assumed.\n"
		<< '\n'
		<< "When the callibration window openes and you see the output from the camera,\n"
		<< "take pictures of a chessboard by pressing the space key.\n"
		<< "Press 'n' to finish and write the camera parameters to disc as [NAME HERE].\n"
		<< "Press 'q' to abort.\n";
}

int main(int argc, char* argv[])
{
	int cameraIndex = 0;

	if (argc > 2) {
		usage();
		return 1;
	}

	if (argc == 2) {
		char* arg = argv[1];
		char* end;
		cameraIndex = std::strtol(arg, &end, 0);

		if (end - arg != std::strlen(arg)) {
			usage();
			return 1;
		}
	}

	VideoCapture cap{cameraIndex};

	if (!cap.isOpened()) {
		cerr << "error: couldn't capture camera number " << cameraIndex << '\n';
		return 1;
	}

	vector<Mat> capturedFrames;

	int pressedKey = 0;

	while (pressedKey != 'n') {
		Mat currFrame;
		cap >> currFrame;
		cv::imshow("Callibrate camera", currFrame);
		pressedKey = cv::waitKey(1);

		if (pressedKey == ' ') {
			capturedFrames.push_back(currFrame.clone());

			cv::threshold(currFrame, currFrame, 70, 255, CV_THRESH_BINARY_INV);
			cv::imshow("Callibrate camera", currFrame);
			cv::waitKey(60);
		}
		else if (pressedKey == 'q') {
			return 0;
		}
	}

	vector<Point3f> chessboardPoints;

	for (int y = 0; y < chessboardSize.height; y++)	{
		for(int x = 0; x < chessboardSize.width; x++) {
			chessboardPoints.push_back(Point3f{(float) x, (float) y, 0});
		}
	}

	vector<vector<Point3f>> objectPoints;
	vector<vector<Point2f>> imagePoints;

	for (auto frame : capturedFrames) {
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
		cv::imshow("Callibrate camera", frame);
		if (cv::waitKey(0) == 'q')
			return 0;

	}

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
		CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	cout << reprojectionError << endl;
	cout << cameraMatrix << endl;
	cout << distCoeffs << endl;

	for (auto frame : capturedFrames) {
		Mat m;
		cv::undistort(frame, m, cameraMatrix, distCoeffs);
		cv::imshow("Callibrate camera", m);
		if (cv::waitKey(0) == 'q')
			return 0;
	}

	return 0;
}
