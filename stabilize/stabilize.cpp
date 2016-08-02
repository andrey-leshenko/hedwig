#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

using std::vector;

using cv::VideoCapture;
using cv::Mat;
using cv::Size;
using cv::Point2f;
using cv::Point3f;
using cv::FileStorage;
using cv::Affine3d;

const vector<int> cameraIndexes{1, 2};
const vector<const char*> cameraCalib{"cam3011.yaml", "cam7192.yaml"};
const Size chessboardSize{8, 5}; 
const float chessSquareSize = 2.9;

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
	
	vector<Mat> cameraMatrixes(cameraCount);
	
	{
		for (int i = 0; i < cameraCount; i++) {
			FileStorage fs(cameraCalib[i], FileStorage::READ);
			if (!fs.isOpened()) {
				std::cerr << "Couldn't open " << cameraCalib[i] << std::endl;
				return 1;
			}
			fs["cameraMatrix"] >> cameraMatrixes[i];
		}
	}

	vector<Point3f> objectPoints;

	for (int z = 0; z < chessboardSize.height; z++)	{
		for(int x = 0; x < chessboardSize.width; x++) {
			objectPoints.push_back(Point3f{x * chessSquareSize, 0, z * chessSquareSize});
		}
	}
	
	vector<vector<Point2f>> imagePoints(cameraCount);
	vector<Mat> rvecs(cameraCount);
	vector<Mat> tvecs(cameraCount);
	vector<Affine3d> cameraTransforms(cameraCount);
	vector<Mat> projectionMatrixes(cameraCount);

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
				std::cerr << "Couldn't calibrate camera." << std::endl;
				return 1;
			}	
		}

		for (int i = 0; i < frames.size(); i++) {
			cameraTransforms[i] = Affine3d(rvecs[i], tvecs[i]);
			Mat rtMatrix = Mat{cameraTransforms[i].matrix}.rowRange(0, 3);
			projectionMatrixes[i] = cameraMatrixes[i] * rtMatrix;

			cameraTransforms[i] = cameraTransforms[i].inv();
		}
	}

	vector<Point3f> triangulated;

	{
		Mat homogeneous;
		cv::triangulatePoints(projectionMatrixes[0], projectionMatrixes[1], imagePoints[0], imagePoints[1], homogeneous);
		cv::convertPointsFromHomogeneous(homogeneous.t(), triangulated);
	}

	cv::destroyAllWindows();

	cv::viz::Viz3d window{"window"};

	for (int i = 0; i < cameraCount; i++) {
		std::string name = "camera";
		name += std::to_string(i);
		cv::viz::WCameraPosition camWidget{cv::Matx33d{cameraMatrixes[i]}, 10};
		window.showWidget(name, camWidget);
		cv::Affine3d transform{rvecs[i], tvecs[i]};
		window.setWidgetPose(name, cameraTransforms[i]);
	}

	window.showWidget("axes", cv::viz::WCoordinateSystem{20});
	window.showWidget("chessboard", cv::viz::WCloud{objectPoints});
	window.showWidget("chessboardTracked", cv::viz::WCloud{triangulated, cv::viz::Color::red()});

	window.spin();

	return 0;
}
