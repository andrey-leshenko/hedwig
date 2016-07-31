#include <iostream>

#include <opencv2/opencv.hpp>

using cv::VideoCapture;
using cv::Mat;

int main()
{
	VideoCapture c1{1};
	VideoCapture c2{2};

	Mat frame1;
	Mat frame2;

	int pressed_key = 0;

	while (pressed_key != 'q') {
		c1 >> frame1;
		c2 >> frame2;

		cv::imshow("window1", frame1);
		cv::imshow("window2", frame2);

		pressed_key = cv::waitKey(1);
	}

	return 0;
}
