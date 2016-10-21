#include "util.hpp"

void showFrameRateInTitle(const char* window)
{
	static int64 freq = static_cast<int>(cv::getTickFrequency());
	static int64 captureLength = freq / 10;

	static int64 start = cv::getTickCount();
	static int frames = 0;

	frames++;

	int64 curr = cv::getTickCount();

	if ((curr - start) >= captureLength) {
		int fps = frames * (freq / (curr - start));
		cv::setWindowTitle(window, std::to_string(fps));
		start = curr;
		frames = 0;
	}
}

void printTimeSinceLastCall(const char* message)
{
	static s64 freq = static_cast<int>(cv::getTickFrequency());
	static s64 last = cv::getTickCount();

	s64 curr = cv::getTickCount();
	s64 delta = curr - last;
	double deltaMs = (double)delta / freq * 1000;
	printf("%s: %.2f\n", message, deltaMs);

	last = curr;
}

void showImageAsGrayscale(InputArray img)
{
	Mat uc;

	Mat m = img.getMat();
	img.getMat().convertTo(uc, CV_8U);
	cv::imshow("w", uc);
}
