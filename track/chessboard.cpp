#include "chessboard.hpp"

#include "base.hpp"
#include "util.hpp"
#include "threshold.hpp"

struct Blob
{
	int neighbors[4];
	bool invalid;
};

static void markAsNeighbor(Blob &a, int bIndex)
{
	for (int i = 0; i < 4; i++) {
		if (a.neighbors[i] == bIndex) {
			return;
		}
		if (a.neighbors[i] == 0) {
			a.neighbors[i] = bIndex;
			return;
		}
	}

	a.invalid = true;
}

static bool closeCircle(const vector<Blob> &blobs, int a, int b, int &ax, int &bx, int notAx = -1)
{
	const Blob &blobA = blobs[a];
	const Blob &blobB = blobs[b];

	for (int i = 0; i < 4; i++) {
		int maybeAx = blobA.neighbors[i];

		if (maybeAx == 0) {
			break;
		}
		if (maybeAx == b || maybeAx == notAx) {
			continue;
		}

		const Blob &blobMaybeAx = blobs[maybeAx];

		if (blobMaybeAx.invalid) {
			continue;
		}

		for (int j = 0; j < 4; j++) {
			int maybeBx = blobMaybeAx.neighbors[j];

			if (maybeBx == 0) {
				break;
			}
			if (blobs[maybeBx].invalid) {
				continue;
			}

			for (int k = 0; k < 4; k++) {
				if (blobB.neighbors[k] == 0) {
					break;
				}
				if (maybeBx == blobB.neighbors[k] &&
					maybeBx != a) {
					ax = maybeAx;
					bx = maybeBx;
					return true;
				}
			}
		}
	}

	return false;
}

static bool areClockwiseOnImage(Vec2d a, Vec2d b, Vec2d c, Vec2d d)
{
	Vec2d ac = c - a;
	Vec2d bd = d - b;

	Vec2d acNormal{ac(1), -ac(0)};
	double dot = acNormal.dot(bd);

	bool clockwise = dot >= 0;
	bool clockwiseWhenYIsDown = !clockwise;

	return clockwiseWhenYIsDown;
}

bool findChessboardSquares(InputArray src, Size gridSize, vector<Point2f> &foundPoints, int flags)
{
	const bool newThreshold = flags & CV_CALIB_CB_ADAPTIVE_THRESH;
	const bool newTernerization = true;

	int gridSizeMin = std::min(gridSize.width, gridSize.height);
	int gridSizeMax = std::max(gridSize.width, gridSize.height);

	Mat frame = src.getMat();
	Mat grayscale;
	Mat binary;
	Mat ternary;
	Mat result;

	Mat whiteMask;
	Mat blackMask;
	Mat whiteMaskInv;

	Mat whiteComps;
	Mat blackComps;
	Mat allComps;

	Mat whiteStats;
	Mat blackStats;

	Mat whiteCenters;
	Mat blackCenters;

	vector<Blob> blobs;

	Mat chess{gridSizeMax, gridSizeMax, CV_32S};

	//
	// Find ternary version
	//

	cv::cvtColor(frame, grayscale, CV_BGR2GRAY);

	if (!newThreshold) {
		cv::threshold(grayscale, whiteMask, 80, 255, CV_THRESH_BINARY);
	}
	else {
		leshenkoThreshold(grayscale, whiteMask);
	}

	binary = whiteMask.clone();
	cv::bitwise_not(whiteMask, blackMask);

	if (newTernerization)
	{
		//cv::erode(whiteMask, whiteMask, Mat{}, Point{-1, -1}, 2);
		cv::dilate(whiteMask, whiteMask, Mat{}, Point{-1, -1}, 2);
		//cv::erode(blackMask, blackMask, Mat{}, Point{-1, -1}, 2);
		cv::dilate(blackMask, blackMask, Mat{}, Point{-1, -1}, 2);

		Mat notInBoth;
		cv::bitwise_xor(whiteMask, blackMask, notInBoth);
		cv::bitwise_and(whiteMask, notInBoth, whiteMask);
		cv::bitwise_and(blackMask, notInBoth, blackMask);

		cv::dilate(whiteMask, whiteMask, Mat{}, Point{-1, -1}, 1);
		cv::dilate(blackMask, blackMask, Mat{}, Point{-1, -1}, 1);
	}
	else
	{
		cv::dilate(whiteMask, whiteMask, Mat{}, Point{-1, -1}, 2);
		cv::erode(whiteMask, whiteMask, Mat{}, Point{-1, -1}, 6);
		cv::dilate(whiteMask, whiteMask, Mat{}, Point{-1, -1}, 1);

		cv::dilate(blackMask, blackMask, Mat{}, Point{-1, -1}, 2);
		cv::erode(blackMask, blackMask, Mat{}, Point{-1, -1}, 6);
		cv::dilate(blackMask, blackMask, Mat{}, Point{-1, -1}, 1);
	}

	cv::bitwise_not(whiteMask, whiteMaskInv);
	cv::bitwise_and(blackMask, whiteMaskInv, blackMask);

	ternary.create(frame.rows, frame.cols, CV_8U);
	ternary.setTo(Scalar{127});
	cv::subtract(ternary, blackMask, ternary);
	cv::bitwise_or(ternary, whiteMask, ternary);

	//
	// Find connected components
	//

	int whiteN = cv::connectedComponentsWithStats(whiteMask, whiteComps, whiteStats, whiteCenters, 4, CV_32S);
	int blackN = cv::connectedComponentsWithStats(blackMask, blackComps, blackStats, blackCenters, 4, CV_32S);

	allComps = blackComps.clone();
	int foregroundBlackN = blackN - 1;
	cv::add(allComps, foregroundBlackN, allComps, whiteMask);
	allComps += whiteComps;

	int totalCompCount = whiteN + foregroundBlackN;

	//
	// Find neighbors
	//

	if (totalCompCount > blobs.size()) {
		blobs.resize(totalCompCount);
	}

	{
		Blob empty = {0};
		std::fill(blobs.begin(), blobs.end(), empty);
	}


	auto findNeighborsHorizontal = [](vector<Blob> &blobs, Mat ternary, Mat allComps)
	{
		const int maxDistance = 10;

		int imRows = ternary.rows;
		int imCols = ternary.cols;

		for (int r = 0; r < imRows; r++) {
			u8 lastColor = 127;
			s32 lastComp = 0;
			int distance = 0;

			for (int c = 0; c < imCols; c++) {
				u8 currColor = ternary.at<u8>(r, c);
				s32 currComp = allComps.at<s32>(r, c);

				if (currComp == 0) {
					distance++;
				}
				else if (currComp == lastComp) {
					distance = 1;
				}
				else {
					if (lastComp != 0 && lastColor != currColor && distance <= maxDistance) {
						markAsNeighbor(blobs[lastComp], currComp);
						markAsNeighbor(blobs[currComp], lastComp);
					}

					lastColor = currColor;
					lastComp = currComp;
					distance = 1;
				}
			}
		}
	};

	{
		findNeighborsHorizontal(blobs, ternary, allComps);
		Mat ternaryT = ternary.t();
		Mat allCompsT = allComps.t();
		findNeighborsHorizontal(blobs, ternaryT, allCompsT);
	}

	//
	// Visualize neighbors graph
	//

	if (false) {
		for (int i = 1; i < whiteN; i++) {
			Point2d center{whiteCenters.at<double>(i, 0), whiteCenters.at<double>(i, 1)};
			cv::circle(ternary, center, 5, Scalar{180, 10, 10}, -1);
		}

		for (int i = 1; i < blackN; i++) {
			Point2d center{blackCenters.at<double>(i, 0), blackCenters.at<double>(i, 1)};
			cv::circle(ternary, center, 5, Scalar{180, 10, 10}, -1);
		}

		for (int i = 1; i < whiteN; i++) {
			Blob b = blobs[i + foregroundBlackN];
			Point2d p1{whiteCenters.at<double>(i, 0), whiteCenters.at<double>(i, 1)};

			for (int k = 0; k < 4; k++) {
				int n = b.neighbors[k];

				if (n == 0) {
					break;
				}

				Point2d p2{blackCenters.at<double>(n, 0), blackCenters.at<double>(n, 1)};

				cv::line(ternary, p1, p2, Scalar{180, 10, 10});
			}
		}

		for (int i = 1; i < blackN; i++) {
			Blob b = blobs[i];
			Point2d p1{blackCenters.at<double>(i, 0), blackCenters.at<double>(i, 1)};

			for (int k = 0; k < 4; k++) {
				int n = b.neighbors[k];

				if (n == 0) {
					break;
				}

				n -= foregroundBlackN;

				Point2d p2{whiteCenters.at<double>(n, 0), whiteCenters.at<double>(n, 1)};

				cv::line(ternary, p1, p2, Scalar{180, 10, 10});
			}
		}
	}

	//
	// Find the grid neighborhood
	//

	CV_Assert(gridSize.height > 1 && gridSize.width > 1);

	result = frame.clone();

	for (int i = 1; i < blackN; i++) {
		Blob &b = blobs[i];
		if (b.neighbors[3] || !b.neighbors[2]) {
			continue;
		}

		chess.setTo(0);
		chess.at<s32>(0, 0) = i;
		chess.at<s32>(0, 1) = blobs[b.neighbors[0]].invalid ?
			b.neighbors[1] : b.neighbors[0];

		for (int r = 0; r + 1 < gridSizeMax; r++) {
			bool found = closeCircle(blobs,
					chess.at<s32>(r, 0), chess.at<s32>(r, 1),
					chess.at<s32>(r + 1, 0), chess.at<s32>(r + 1, 1),
					r > 0 ? chess.at<s32>(r - 1, 0) : -1);
			if (!found) {
				break;
			}

			for (int c = 0; c + 1 < gridSizeMax; c++) {
				bool found = closeCircle(blobs,
						chess.at<s32>(r, c), chess.at<s32>(r + 1, c),
						chess.at<s32>(r, c + 1), chess.at<s32>(r + 1, c + 1),
						c > 0 ? chess.at<s32>(r, c - 1) : -1);
				if (!found) {
					break;
				}
			}
		}


		if (cv::countNonZero(chess(Range{0, 2}, Range{0, 2})) != 4) {
			continue;
		}


		{
			int i1 = chess.at<s32>(0, 0);
			int i2 = chess.at<s32>(0, 1) - foregroundBlackN;
			int i3 = chess.at<s32>(1, 1);
			int i4 = chess.at<s32>(1, 0) - foregroundBlackN;

			Vec2d p1{blackCenters.at<double>(i1, 0), blackCenters.at<double>(i1, 1)};
			Vec2d p2{whiteCenters.at<double>(i2, 0), whiteCenters.at<double>(i2, 1)};
			Vec2d p3{blackCenters.at<double>(i3, 0), blackCenters.at<double>(i3, 1)};
			Vec2d p4{whiteCenters.at<double>(i4, 0), whiteCenters.at<double>(i4, 1)};

			if (!areClockwiseOnImage(p1, p2, p3, p4)) {
				chess = chess.t();
			}
		}


		Mat chessActive = chess(Range{0, gridSize.height}, Range{0, gridSize.width});


		if (cv::countNonZero(chessActive) != gridSize.width * gridSize.height) {
			if (gridSize.height == gridSize.width) {
				continue;
			}

			if (gridSize.height % 2 == 0 && gridSize.width % 2 == 0) {
				continue;
			}

			chessActive = chess(Range{0, gridSize.width}, Range{0, gridSize.height});

			if (cv::countNonZero(chessActive) != gridSize.width * gridSize.height) {
				continue;
			}

			int currentHeight = gridSize.width;
			int currentWidth = gridSize.height;

			if (currentHeight & 1) {
				// rotate CW
				chessActive = chessActive.t();
				int flipAroundY = 1;
				cv::flip(chessActive, chessActive, flipAroundY);
			}
			else if (currentWidth & 1) {
				// rotate CCW
				chessActive = chessActive.t();
				int flipAroundX = 0;
				cv::flip(chessActive, chessActive, flipAroundX);
			}
			else {
				CV_Assert(0);
			}

			if (cv::countNonZero(chessActive) != gridSize.width * gridSize.height) {
				continue;
			}
		}

		// TODO: Check that the chessboard is EXACTLY in the correct format

		foundPoints.resize(0);

		for (int r = 0; r < gridSize.height; r++) {
			for (int c = 0; c < gridSize.width; c++) {
				int p = chessActive.at<s32>(r, c);
				Point2f pt;
				if (p < blackN) {
					pt = Point2f{(float)blackCenters.at<double>(p, 0), (float)blackCenters.at<double>(p, 1)};
				}
				else {
					p -= foregroundBlackN;
					pt = Point2f{(float)whiteCenters.at<double>(p, 0), (float)whiteCenters.at<double>(p, 1)};
				}

				foundPoints.push_back(pt);
			}
		}

		return true;
	}

	return false;
}
