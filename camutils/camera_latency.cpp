//
// Written by Andrey Leshenko, Eli Tarnarutsky and Shir Amir.
// Published under the MIT license.
//

#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

using std::cerr;
using std::cout;

using cv::VideoCapture;
using cv::Mat;
using cv::Scalar;

void usage()
{
	std::cout << "Usage: camera-latency [CAMERA_INDEX]\n"
		<< '\n'
		<< "Estimate the latency of video fro camera number CAMERA_INDEX.\n"
		<< "If no camera index is specified, CAMERA_INDEX=0 is assumed.\n";
}

Mat channel(const Mat& m, int channel)
{
	Mat channels[3];
	cv::split(m, channels);
	return channels[channel];
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
	// TODO(Andrey): Pass as arguments
	cap.set(cv::CAP_PROP_FPS, 60);

	if (!cap.isOpened()) {
		cerr << "error: couldn't capture camera number " << cameraIndex << '\n';
		return 1;
	}

	Mat currFrame;
	Mat imageBlack{512, 512, CV_8UC3, Scalar{0, 0, 255}};
	Mat imageWhite{512, 512, CV_8UC3, Scalar{255, 255, 255}};

	bool measuring = false;
	auto beginTime = std::chrono::high_resolution_clock::now();
	int frames = 0;

	cv::imshow("Display", imageBlack);

	int pressedKey = 0;

	while (pressedKey != 'n') {
		cap >> currFrame;

		currFrame = channel(currFrame, 0);
		cv::threshold(currFrame, currFrame, 85, 255, CV_THRESH_BINARY);

		frames++;
		int white = cv::countNonZero(currFrame);

		if (measuring && white > currFrame.rows * currFrame.cols * 0.3) {
			auto endTime = std::chrono::high_resolution_clock::now();
			measuring = false;
			cv::imshow("Display", imageBlack);

			std::cout <<
				std::chrono::duration_cast<std::chrono::milliseconds>(endTime-beginTime).count()
				<< "ms "
				<< frames << " frames" << std::endl;
		}

		cv::imshow("Video feed", currFrame);
		pressedKey = cv::waitKey(1);

		if (!measuring && pressedKey == ' ') {
			cv::imshow("Display", imageWhite);
			beginTime = std::chrono::high_resolution_clock::now();
			measuring = true;
			frames = 0;
		}
		else if (pressedKey == 'q') {
			return 0;
		}
	}

	return 0;
}
