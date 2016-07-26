#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using std::cerr;
using std::cout;
using std::vector;

using cv::VideoCapture;
using cv::Mat;

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

	Mat currFrame;
	vector<Mat> capturedFrames;

	int pressedKey = 0;

	while (pressedKey != 'n') {
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

	for (auto m : capturedFrames) {
		cv::imshow("Callibrate camera", m);
		if (cv::waitKey(0) == 'q')
			return 0;
	}

	return 0;
}
