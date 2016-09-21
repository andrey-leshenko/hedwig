//
// Written by Andrey Leshenko, Eli Tarnarutsky and Shir Amir.
// Published under the MIT license.
//

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
using cv::Affine3f;
using cv::Affine3d;
using cv::SVD;
using cv::Vec3f;

const vector<int> cameraIndexes{1, 2};
const vector<const char*> cameraCalib{"../../calib_data/ps_eye.yaml", "../../calib_data/ps_eye.yaml"};
const Size chessboardSize{8, 5};
const float chessSquareSize = 3.025;

Point3f centroid(const vector<Point3f>& points)
{
	Vec3f sum;
	for (Point3f p : points) {
		sum += Vec3f{p};
	}
	int count = points.size();
	return sum / count;
}

Mat createPointMatrix(const vector<Point3f>& points)
{
	int pointCount = static_cast<int>(points.size());
	Mat m{3, pointCount, CV_32F};

	for (int i = 0; i < pointCount; i++) {
		m.at<float>(0,i) = points[i].x;
		m.at<float>(1,i) = points[i].y;
		m.at<float>(2,i) = points[i].z;
	}
	return m;
}

vector<Point3f> pointsTranslate(const vector<Point3f>& points, Point3f v)
{
	vector<Point3f> result(points.size());

	for (int i = 0; i < points.size(); i++) {
		result[i].x = points[i].x + v.x;
		result[i].y = points[i].y + v.y;
		result[i].z = points[i].z + v.z;
	}

	return result;
}

void captureCameraFrames(vector<VideoCapture>& cameras, vector<Mat>& frames)
{
	for (int i = 0; i < frames.size(); i++) {
		cameras[i].grab();
	}
	for (int i = 0; i < frames.size(); i++) {
		cameras[i].retrieve(frames[i]);
	}
}

bool findChessboards(const vector<Mat> images, Size chessboardSize, vector<vector<Point2f>>& pointsOut)
{
	for (int i = 0; i < images.size(); i++) {
		bool found = cv::findChessboardCorners(
				images[i],
				chessboardSize,
				pointsOut[i],
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);

		if (!found) {
			return false;
		}
	}
	return true;
}

int main()
{
	int cameraCount = cameraIndexes.size();
	vector<VideoCapture> cameras;
	vector<Mat> frames(cameraCount);

	for (int i : cameraIndexes) {
		cameras.push_back(VideoCapture{i});
		if (!cameras.back().isOpened()) {
			std::cerr << "Couldn't open camera at index " << i << std::endl;
			return 1;
		}
	}

	//
	// Inspect the camera feeds
	//

	{
		int currCamera = 0;
		bool inspecting = true;

		while (inspecting) {
			captureCameraFrames(cameras, frames);

			cv::imshow("w", frames[currCamera]);
			int pressedKey = cv::waitKey(1);

			switch (pressedKey)
			{
				case 'n':
				case ' ':
					inspecting = false;
					break;
				case 'q':
					exit(0);
				case 'j':
					currCamera = (currCamera + 1) % cameraCount;
					break;
				case 'k':
					currCamera = (currCamera - 1 + cameraCount) % cameraCount;
					break;

			}
		}
	}

	//
	// Read camera matrices from config file
	//

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

	//
	// Calculate the chessboard's initial position
	//

	vector<Point3f> initialPoints;

	for (int z = 0; z < chessboardSize.height; z++)	{
		for(int x = 0; x < chessboardSize.width; x++) {
			initialPoints.push_back(Point3f{x * chessSquareSize, 0, z * chessSquareSize});
		}
	}

	Point3f initialPosition = centroid(initialPoints);
	vector<Point3f> initialPointsCentered = pointsTranslate(initialPoints, -initialPosition);
	Mat initialPointsCenteredMat = createPointMatrix(initialPointsCentered);

	//
	// Calcualte projection matrices for each camera
	//

	vector<vector<Point2f>> imagePoints(cameraCount);
	vector<Affine3d> cameraTransforms(cameraCount);
	vector<Mat> projectionMatrixes(cameraCount);

	{
		captureCameraFrames(cameras, frames);

		if (!findChessboards(frames, chessboardSize, imagePoints)) {
			std::cerr << "Chessboard corners were not found." << std::endl;
			return 1;
		}

		vector<Mat> rvecs(cameraCount);
		vector<Mat> tvecs(cameraCount);

		for (int i = 0; i < frames.size(); i++) {
			bool found = cv::solvePnP(initialPoints,
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
			// We found how to transform points so a camera placed
			// at the origin will see them. Assuming that points were at
			// the origin in the first place, the location of the camera
			// is the inverse of that transformation.
			cameraTransforms[i] = cameraTransforms[i].inv();
		}
	}

	//
	// Triangulate and visualize the outputted data
	//

	cv::destroyAllWindows();
	cv::viz::Viz3d window{"window"};

	for (int i = 0; i < cameraCount; i++) {
		std::string name = "camera";
		name += std::to_string(i);
		cv::viz::WCameraPosition camWidget{cv::Matx33d{cameraMatrixes[i]}, 10};
		window.showWidget(name, camWidget);
		window.setWidgetPose(name, cameraTransforms[i]);
	}

	window.showWidget("axes", cv::viz::WCoordinateSystem{20});
	window.showWidget("drone", cv::viz::WCube{Point3f{-10, -2, -10}, Point3f{10, 2, 10}, true});

	do {
		window.spinOnce(1, true);

		captureCameraFrames(cameras, frames);

		if (!findChessboards(frames, chessboardSize, imagePoints)) {
			continue;
		}

		Mat homogeneous;
		cv::triangulatePoints(projectionMatrixes[0], projectionMatrixes[1], imagePoints[0], imagePoints[1], homogeneous);

		vector<Point3f> currPoints;
		// Using a vector of points or the matrix before transposition doesn't work
		// because of some strange OpenCV problems.
		cv::convertPointsFromHomogeneous(homogeneous.t(), currPoints);

		Point3f currPosition = centroid(currPoints);
		vector<Point3f> currPointsCentered = pointsTranslate(currPoints, -currPosition);
		Mat currPointsCenteredMat = createPointMatrix(currPointsCentered);

		Affine3f currTransform;

		{
			Mat covarianceMatrix{initialPointsCenteredMat * currPointsCenteredMat.t()};
			Mat u, s, vt;

			SVD::compute(covarianceMatrix, s, u, vt); // cov = u * s * vt
			Mat rot = vt.t() * u.t();

			if (cv::determinant(rot) < 0)
			{
				rot.col(2) *= -1;
			}

			currTransform.rotation(rot);

			//currTransform.translation(Vec3f{currPosition} - Vec3f{initialPosition});
			currTransform.translation(Vec3f{currPosition});
		}

		window.showWidget("chessboard", cv::viz::WCloud{currPoints});
		window.setRenderingProperty("chessboard", cv::viz::POINT_SIZE, 4);

		window.setWidgetPose("drone", currTransform);
	} while (!window.wasStopped());

	return 0;
}
