#pragma once

#include "base.hpp"
#include "config.hpp"

struct CameraData
{
	int cameraCount;
	vector<int> indexes;
	vector<VideoCapture> captures;
	vector<Mat> cameraMatrixes;
	vector<Mat> projectionMatrixes;
	vector<Affine3f> transforms;

	vector<Mat> frames;
	vector<vector<Point2f>> imagePoints;
};

bool doExternalCalibration(CameraData &cameras, Size chessboardSize, float squareSize);
bool doExternalCalibrationInteractive(CameraData &cameras, Size chessboardSize, float squareSize);
bool setupCameras(CameraData &cameras, Config &cfg);
bool triangulateChessboardPoints(vector<Point3f> &points, CameraData &cameras, Size chessboardSize);
void saveExternalCalibrationToFile(CameraData &cameras, const char *file);
bool loadExternalCalibrationFromFile(CameraData &cameras, const char *file);
