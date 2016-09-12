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
const Size chessboardSize{8, 5};
const float chessSquareSize = 3.025;

Point3f centroid(const vector<Point3f>& points)
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
	Mat m{3, static_cast<int>(points.size()), CV_32F};

	for (int i = 0; i < points.size(); i++) {
		m.at<float>(0,i) = points[i].x;
		m.at<float>(1,i) = points[i].y;
		m.at<float>(2,i) = points[i].z;
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

	vector<Point3f> initialPoints;

	//for (int z = 0; z < chessboardSize.height; z++)	{
	//	for(int x = 0; x < chessboardSize.width; x++) {
	//		initialPoints.push_back(Point3f{x * chessSquareSize, 0, z * chessSquareSize});
	//	}
	//}

	for (int z = 0; z < chessboardSize.height; z++)	{
		for(int x = 0; x < chessboardSize.width; x++) {
			initialPoints.push_back(Point3f{x * chessSquareSize, (chessboardSize.height - z) * chessSquareSize, 0});
		}
	}

	Point3f initialPosition = centroid(initialPoints);

	vector<Point3f> initialPointsCentered(initialPoints.size());
	for (int i = 0; i < initialPoints.size(); i++) {
		initialPointsCentered[i].x = initialPoints[i].x - initialPosition.x;
		initialPointsCentered[i].y = initialPoints[i].y - initialPosition.y;
		initialPointsCentered[i].z = initialPoints[i].z - initialPosition.z;
	}

	Mat initialPointsCenteredMat = createPointMatrix(initialPointsCentered);

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
	//window.showWidget("drone", cv::viz::WCube{Point3f{-20, -5, -20}, Point3f{20, 5, 20}, true});
	window.showWidget("drone", cv::viz::WCube{Point3f{-10, -10, -2}, Point3f{10, 10, 2}, true});
	//window.showWidget("calibrationChessboard", cv::viz::WCloud{initialPointsCentered, cv::viz::Color::red()});
	//window.setRenderingProperty("calibrationChessboard", cv::viz::POINT_SIZE, 4);

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

			vector<Point3f> currPoints;
			cv::convertPointsFromHomogeneous(homogeneous.t(), currPoints);

			Point3f currPosition = centroid(currPoints);

			vector<Point3f> currPointsCentered(initialPoints.size());
			for (int i = 0; i < currPoints.size(); i++) {
				currPointsCentered[i].x = currPoints[i].x - currPosition.x;
				currPointsCentered[i].y = currPoints[i].y - currPosition.y;
				currPointsCentered[i].z = currPoints[i].z - currPosition.z;
			}

			Mat currPointsCenteredMat = createPointMatrix(currPointsCentered);

			Affine3f currTransform;

			{
				Mat covarianceMatrix{initialPointsCenteredMat * currPointsCenteredMat.t()};
				Mat u, s, vt;

				SVD::compute(covarianceMatrix, s, u, vt); // cov = u * s * vt
				Mat rot = vt.t() * u.t();

				if (cv::determinant(rot) < 0)
				{
					Mat thirdCol = rot.col(2);
					thirdCol *= -1;
				}

				//Mat diagonal = Mat::eye(3, 3, CV_32F);
				//diagonal.at<float>(2,2) = d;
				//Mat u = rsft.t() * diagonal * lsf.t();

				currTransform.rotation(rot);

				//currTransform.translation(Vec3f{currPosition} - Vec3f{initialPosition});
				// For Testing:
				currTransform.translation(Vec3f{currPosition});
			}

			window.showWidget("chessboard", cv::viz::WCloud{currPoints});
			window.setRenderingProperty("chessboard", cv::viz::POINT_SIZE, 4);

			window.setWidgetPose("drone", currTransform);

			int post = cv::getTickCount();
			//std::cout << (post - pre)/cv::getTickFrequency() << std::endl;
		}

		window.spinOnce(1, true);
	} while (!window.wasStopped());

	return 0;
}
