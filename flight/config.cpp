//
// Written by Andrey Leshenko, Eli Tarnarutsky and Shir Amir.
// Published under the MIT license.
//

#include "config.hpp"

using cv::FileStorage;
using cv::String;
using cv::FileNode;
using cv::Affine3f;
using cv::Affine3d;
using cv::Vec4f;
using cv::SVD;

void printConfig(Config &cfg)
{
	std::cout << "camera_count: " << cfg.cameraCount << std::endl;

	std::cout << "camera_indexes:";
	for (int i : cfg.cameraIndexes) {
		std::cout << " " << i;
	}
	std::cout << std::endl;

	std::cout << "camera_matrix: " << cfg.cameraMatrix << std::endl;

	std::cout << "calibration_chessboard_size: " << cfg.calibrationChessboardSize << std::endl;
	std::cout << "calibration_chessboard_square_size: " << cfg.calibrationChessboardSquareSize << std::endl;
	std::cout << "drone_chessboard_size: " << cfg.droneChessboardSize << std::endl;
	std::cout << "drone_possible_serial_devices:";
	for (auto s : cfg.dronePossibleSerialDevices) {
		std::cout << " " << s;
	}
	std::cout << std::endl;

	std::cout << "drone_channel_bounds: " << std::endl;
	for (int i = 0; i < 4; i++) {
		ChannelBounds b = cfg.droneChannels[i];
		std::cout << "    [" << b.min << ", " << b.zero << ", " << b.max << "]" << std::endl;
	}

	std::cout << "drone_pid_parameters: " << std::endl;
	for (int i = 0; i < 4; i++) {
		PIDParameters p = cfg.dronePIDParameters[i];
		std::cout << "    [" << p.kp << ", " << p.ki << ", " << p.kd << "]" << std::endl;
	}
}

static bool readChannelBoundsFromFile(ChannelBounds &bounds, FileNode &node)
{
	if (node.size() != 3) {
		return false;
	}

	vector<int> params;
	node >> params;

	bounds.min = params[0];
	bounds.zero = params[1];
	bounds.max = params[2];

	return true;
}

static bool readPIDPatametersFromFile(PIDParameters &pid, FileNode &node)
{
	node["kp"] >> pid.kp;
	node["ki"] >> pid.ki;
	node["kd"] >> pid.kd;

	return true;
}

bool loadConfigFromFile(Config &cfg, const char *file)
{
	FileStorage fs(file, FileStorage::READ);

	if (!fs.isOpened())
	{
		return false;
	}

	fs["camera_count"] >> cfg.cameraCount;
	fs["camera_indexes"] >> cfg.cameraIndexes;
	fs["intrinsic_calibration_data"]["camera_matrix"] >> cfg.cameraMatrix;

	fs["calibration_chessboard_size"] >> cfg.calibrationChessboardSize;
	fs["calibration_chessboard_square_size"] >> cfg.calibrationChessboardSquareSize;

	fs["drone_chessboard_size"] >> cfg.droneChessboardSize;
	fs["drone_possible_serial_devices"] >> cfg.dronePossibleSerialDevices;

	const char *axes[4] = {"right", "forward", "up", "clockwise"};

	for (int i = 0; i < 4; i++) {
		const char *axis = axes[i];
		FileNode fnChannel = fs["drone_channel_bounds"][axis];
		FileNode fnPID = fs["drone_pid_parameters"][axis];

		readChannelBoundsFromFile(cfg.droneChannels[i], fnChannel);
		readPIDPatametersFromFile(cfg.dronePIDParameters[i], fnPID);
	}

	// TODO(Andrey): Verify the config file is correct

	return true;
}
