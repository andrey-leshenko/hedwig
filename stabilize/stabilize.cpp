#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

using std::vector;

using cv::VideoCapture;
using cv::Mat;
using cv::Size;
using cv::Point2f;
using cv::Point3f;
using cv::FileStorage;

const vector<int> cameraIndexes{1, 2};
const vector<const char*> cameraCalib{"cam3011.yaml", "cam7192.yaml"};
const Size chessboardSize{9, 6}; 
const float chessSquareSize = 2.13;

int main()
{
	const int cameraCount = cameraIndexes.size();
	vector<VideoCapture> cameras;

	for (int i : cameraIndexes) {
		cameras.push_back(VideoCapture{i});
		if (!cameras.back().isOpened()) {
			std::cerr << "Couldn't open camera at index " << i << std::endl;
			return 1;
		}
	}

	vector<Mat> frames(cameraCount);

	//////// Inspect the camera feeds ////////
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
				currCamera = (currCamera + 1) % cameraCount;
			}
			else if (pressedKey == 'k') {
				currCamera = (currCamera - 1 + cameraCount) % cameraCount;
			}
		}
	}
	
	///////////////////////////////////
	
	vector<Mat> cameraMatrixes(cameraCount);
	
	{
		for (int i = 0; i < cameraCount; i++) {
			FileStorage fs(cameraCalib[i], FileStorage::READ);
			fs["cameraMatrix"] >> cameraMatrixes[i];
		}
	}

	vector<Point3f> objectPoints;

	for (int y = 0; y < chessboardSize.height; y++)	{
		for(int x = 0; x < chessboardSize.width; x++) {
			objectPoints.push_back(Point3f{x * chessSquareSize, y * chessSquareSize, 0});
		}
	}
	
	vector<vector<Point2f>> imagePoints(cameraCount);
	vector<Mat> rvecs(cameraCount);
	vector<Mat> tvecs(cameraCount);
	
	//////////////////////////////////
	{
		for (int i = 0; i < frames.size(); i++) {
			cameras[i].grab();
		}

		for (int i = 0; i < frames.size(); i++) {
			cameras[i].retrieve(frames[i]);
		}
		
		for (int i = 0; i < frames.size(); i++) {
			bool found = cv::findChessboardCorners(
				frames[i],
				chessboardSize, 
				imagePoints[i], 
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
				
			if(!found) {
				std::cerr << "Chessboard corners were not found." << std::endl;
				return 1;
			}			
		}
		
		for (int i = 0; i < frames.size(); i++) {
			bool found = cv::solvePnP(objectPoints, 
			imagePoints[i], 
			cameraMatrixes[i], 
			Mat{}, 
			rvecs[i], 
			tvecs[i]);
			
			if (!found) {
				std::cerr << "Couldn't callibrate camera." << std::endl;
			}	
		}

		for (int i = 0; i < frames.size(); i++) {
			std::cout << cameraCalib[i] << ":" << std::endl << rvecs[i] << std::endl << tvecs[i] << std::endl;
		}
	}

	return 0;
}
