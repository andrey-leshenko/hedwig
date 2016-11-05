//
// Written by Andrey Leshenko, Eli Tarnarutsky and Shir Amir.
// Published under the MIT license.
//

#pragma once

#include "base.hpp"

struct ChannelBounds
{
    int min;
    int zero;
    int max;
};

struct PIDParameters
{
    float kp;
    float ki;
    float kd;
};

struct Config
{
	int				cameraCount;
    vector<int>		cameraIndexes;
	Mat				cameraMatrix;
	int				cameraTargetFPS;

    Size			calibrationChessboardSize;
    float			calibrationChessboardSquareSize;

    Size			droneChessboardSize;
    vector<String>	dronePossibleSerialDevices;
    ChannelBounds	droneChannels[4];
    PIDParameters	dronePIDParameters[4];
};

void printConfig(Config &cfg);
bool loadConfigFromFile(Config &cfg, const char *file);
