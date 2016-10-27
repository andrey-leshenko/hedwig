#include "threshold.hpp"

void leshenkoThreshold(InputArray _src, OutputArray _dst, int minDiff)
{
	Mat src = _src.getMat();

	CV_Assert(src.type() == CV_8U);

	Size blurKernel{3, 3};

	int minSize = std::min(src.rows, src.cols);
	int downscalingLevels = static_cast<int>(std::log2(minSize));

	Mat foundColors = Mat::zeros(src.rows, src.cols, CV_8U);

	Mat mipmap = src.clone();
	int level = 0;

	for (int i = 0; i < 3; i++) {
		cv::pyrDown(mipmap, mipmap);
		cv::blur(mipmap, mipmap, blurKernel);
		level++;
	}

	while (level <= downscalingLevels) {
		for (int r = 0; r < src.rows; r++) {
			for (int c = 0; c < src.cols; c++) {
				u8 &curr = foundColors.at<u8>(r, c);
				if (curr == 0) {
					u8 original = src.at<u8>(r, c);
					u8 thisMip = mipmap.at<u8>(r >> level, c >> level);
					if (original >= thisMip + minDiff) {
						curr = 255;
					}
					else if (original <= thisMip - minDiff) {
						curr = 127;
					}
				}
			}
		}

		cv::pyrDown(mipmap, mipmap);
		cv::blur(mipmap, mipmap, blurKernel);
		level++;
	}

	cv::threshold(foundColors, _dst, 128, 255, cv::THRESH_BINARY);
}
