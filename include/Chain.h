#pragma once
#include <Eigen/Core>
#include "DisplayWindow.h"
#include <vector>

struct segment
{
    float length;
    float rot;
};

class Chain
{
private:
    std::vector<segment> m_segments;

    Eigen::Vector2f m_origin;

public:
    Chain(int origin_x, int origin_y);

    void addSegment(float length, float rotation);

    void rotateSegment(int segmentIdx, float amount);

    void drawChain(DisplayWindow& window, COLOUR& c);

    Eigen::Vector2f calculateEffectorPosition();
    Eigen::Vector2f calculateSegmentEndPosition(int jointIndex);
    Eigen::Vector2f calculateSegmentStartPosition(int jointIndex);

    void solveForTargetIKWithCCD(float targetX, float targetY, unsigned int numIterations, float tolerance = 1.0f);
};
