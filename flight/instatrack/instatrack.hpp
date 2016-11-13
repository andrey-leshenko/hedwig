#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

bool instatrackFindChessboard(cv::InputArray src, cv::Size gridSize, std::vector<cv::Point2f> &foundPoints, int flags);
void leshenkoThreshold(cv::InputArray _src, cv::OutputArray _dst, int minDiff = 10);
int instatrackDemo(int cameraIndex, cv::Size chessSize);
