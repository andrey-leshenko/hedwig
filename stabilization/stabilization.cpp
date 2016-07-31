#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using std::vector;

using cv::VideoCapture;
using cv::Mat;

const vector<int> cameraIndexes{1, 2};

int main()
{
	vector<VideoCapture> cameras;

	for (int i : cameraIndexes) {
		cameras.push_back(VideoCapture{i});
		if (!cameras.back().isOpened()) {
			std::cerr << "Couldn't open camera at index " << i << std::endl;
			return 1;
		}
	}

	vector<Mat> frames(cameras.size());

	{
		int pressedKey = 0;
		int currCamera = 0;

		while (pressedKey != 'n') {
			for (int i = 0; i < frames.size(); i++) {
				cameras[i].grab();
			}

			for (int i = 0; i < frames.size(); i++) {
				cameras[i].retrieve(frames[i]);
			}

			cv::imshow("w", frames[currCamera]);
			pressedKey = cv::waitKey(1);

			if (pressedKey == 'j') {
				currCamera = (currCamera + 1) % cameras.size();
			}
			else if (pressedKey == 'k') {
				currCamera = (currCamera - 1 + cameras.size()) % cameras.size();
			}
		}
	}

	return 0;
}
