#pragma once
#include <Eigen/Core>
#include "DisplayWindow.h"
#include <vector>

#define MIN_LIMIT_INDEX     0
#define MAX_LIMIT_INDEX     1

struct segment
{
    float length;
    float rot;
    float limits[2];
};

class Chain
{
private:
    std::vector<segment> m_segments;

    Eigen::Vector2f m_origin;

public:
    Chain(int origin_x, int origin_y);

    bool addSegment(float length, float init_rotation, float limit1 = -M_PI, float limit2 = M_PI);

    void rotateSegment(int segmentIdx, float amount);

    void drawChain(DisplayWindow& window, COLOUR& c);

    Eigen::Vector2f calculateEffectorPosition();
    Eigen::Vector2f calculateSegmentEndPosition(int jointIndex);
    Eigen::Vector2f calculateSegmentStartPosition(int jointIndex);

    void solveForTargetIKWithCCD(float targetX, float targetY, unsigned int numIterations, float tolerance = 1.0f);
};
