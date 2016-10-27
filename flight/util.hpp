#pragma once

#include "base.hpp"

#define QQQ fprintf(stderr, "---QQQ %s(%d)\n", __FUNCTION__, __LINE__)

void showFrameRateInTitle(const char* window);

void printTimeSinceLastCall(const char* message);

void showImageAsGrayscale(InputArray img);
