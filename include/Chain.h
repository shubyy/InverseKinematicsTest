#pragma once

#include <vector>
#include "DisplayWindow.h"

#include <Eigen/Core>

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
    Eigen::Vector2f calculateSegmentPosition(int jointIndex);

    void solveForTargetIKWithCCD(Eigen::Vector2f targetPos, unsigned int numIterations, bool alternate = false);
};
