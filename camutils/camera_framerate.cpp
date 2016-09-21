//
// Written by Andrey Leshenko, Eli Tarnarutsky and Shir Amir.
// Published under the MIT license.
//

#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

using cv::VideoCapture;
using cv::Mat;

void usage()
{
	std::cout << "Usage: camera-framerate [CAMERA_INDEX]\n"
		<< '\n'
		<< "Measure the framerate of camera number CAMERA_INDEX.\n"
		<< "If no camera index is specified, CAMERA_INDEX=0 is assumed.\n";
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
	// TODO(Andrey): pass these settings as arguments
	//cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
	//cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
	cap.set(cv::CAP_PROP_FPS, 120);

	if (!cap.isOpened()) {
		std::cerr << "error: couldn't capture camera number " << cameraIndex << '\n';
		return 1;
	}

	Mat currFrame;
	auto beginTime = high_resolution_clock::now();
	int frameCount = 0;

	int pressedKey = 0;

	while (pressedKey != 'q') {
		cap >> currFrame;
		frameCount++;

		auto now = high_resolution_clock::now();
		int elapsed = duration_cast<milliseconds>(now - beginTime).count();

		if (elapsed > 1000) {
			std::cout << frameCount << std::endl;
			frameCount = 0;
			beginTime = high_resolution_clock::now();
		}

		cv::imshow("Video feed", currFrame);
		pressedKey = cv::waitKey(1);
	}

	return 0;
}
