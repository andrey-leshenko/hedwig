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
using cv::Scalar;
using cv::Vec3f;

const vector<int> cameraIndexes{1, 2};
const vector<const char*> cameraCalib{"cam3011.yaml", "cam7192.yaml"};
const Size chessboardSize{5, 4};
const float chessSquareSize = 3.025;

Point3f pointsMean(const vector<Point3f>& points)
{
	Point3f sum;
	for (Point3f p : points) {
		sum.x += p.x;
		sum.y += p.y;
		sum.z += p.z;
	}
	int count = points.size();
	return Point3f{sum.x / count, sum.y / count, sum.z / count};
}

Mat createPointMatrix(const vector<Point3f>& points)
{
	Mat m{static_cast<int>(points.size()), 3, CV_32F};

	for (int i = 0; i < points.size(); i++) {
		m.at<float>(i,1) = points[i].x;
		m.at<float>(i,2) = points[i].y;
		m.at<float>(i,3) = points[i].z;
	}
	return m;
}

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

	//////// Read camera matrices from xml/yaml file ////////

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

	//////// Calculate the chessboard's initial position ////////

	vector<Point3f> objectPoints;

	for (int z = 0; z < chessboardSize.height; z++)	{
		for(int x = 0; x < chessboardSize.width; x++) {
			objectPoints.push_back(Point3f{x * chessSquareSize, 0, z * chessSquareSize});
		}
	}

	Point3f initialPosition = pointsMean(objectPoints);

	vector<Point3f> movedPoints(objectPoints.size());
	for (int i = 0; i < objectPoints.size(); i++) {
		movedPoints[i].x = objectPoints[i].x - initialPosition.x;
		movedPoints[i].y = objectPoints[i].y - initialPosition.y;
		movedPoints[i].z = objectPoints[i].z - initialPosition.z;
	}

	Mat initialPoints = createPointMatrix(movedPoints);

	//////// Calculate Projection matrices for each camera ////////

	vector<vector<Point2f>> imagePoints(cameraCount);
	vector<Mat> rvecs(cameraCount);
	vector<Mat> tvecs(cameraCount);
	vector<Affine3d> cameraTransforms(cameraCount);
	vector<Mat> projectionMatrixes(cameraCount);

	int pre = cv::getTickCount();

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

	//////// Triangulate and visualize the outputted data ////////

	window.showWidget("axes", cv::viz::WCoordinateSystem{20});
	window.showWidget("drone", cv::viz::WCube{Point3f{-20, -5, -20}, Point3f{20, 5, 20}, false});
	window.showWidget("calibrationChessboard", cv::viz::WCloud{objectPoints, cv::viz::Color::red()});

	do {
		for (int i = 0; i < frames.size(); i++) {
			cameras[i].grab();
		}

		for (int i = 0; i < frames.size(); i++) {
			cameras[i].retrieve(frames[i]);
		}

		bool found;

		for (int i = 0; i < frames.size(); i++) {
			found = cv::findChessboardCorners(
					frames[i],
					chessboardSize,
					imagePoints[i],
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
			if (!found) {
				break;
			}
		}

		if (found) {
			Mat homogeneous;
			cv::triangulatePoints(projectionMatrixes[0], projectionMatrixes[1], imagePoints[0], imagePoints[1], homogeneous);

			vector<Point3f> triangulated;
			cv::convertPointsFromHomogeneous(homogeneous.t(), triangulated);

			Point3f currentPosition = pointsMean(triangulated);

			// REWRITE!!!!!!
			vector<Point3f> movedPoints2(objectPoints.size());
			for (int i = 0; i < triangulated.size(); i++) {
				movedPoints2[i].x = triangulated[i].x - currentPosition.x;
				movedPoints2[i].y = triangulated[i].y - currentPosition.y;
				movedPoints2[i].z = triangulated[i].z - currentPosition.z;
			}

			Mat currPoints = createPointMatrix(movedPoints2);
			Affine3f currTransform;

			{
				Mat covarianceMatrix{initialPoints.t() * currPoints};
				Mat lsf, rsft, sv;

				SVD::compute(covarianceMatrix, sv, lsf, rsft);
				double det = cv::determinant(rsft.t() * lsf.t());
				int d = 1;
				if(det < 0)
					d = -1;
				//if(det == 0)
					//d = 0;
				Mat diagonal = Mat::eye(3, 3, CV_32F);
				diagonal.at<float>(2,2) = d;
				Mat u = rsft.t() * diagonal * lsf.t();

				currTransform.rotation(u);
				currTransform.translation(Vec3f{currentPosition} - Vec3f{initialPosition});
			}

			window.showWidget("chessboard", cv::viz::WCloud{triangulated});
			window.setRenderingProperty("chessboard", cv::viz::POINT_SIZE, 4);
			window.setWidgetPose("drone", currTransform);

			int post = cv::getTickCount();
			//std::cout << (post - pre)/cv::getTickFrequency() << std::endl;
		}

		window.spinOnce(1, true);
	} while (!window.wasStopped());

	return 0;
}
