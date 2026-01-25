#include "Chain.h"
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Dense>


Chain::Chain(int origin_x, int origin_y)
{
    m_origin = Eigen::Vector2f(origin_x, origin_y);
}

bool Chain::addSegment(float length, float init_rotation, float limit1, float limit2)
{
    bool success = false;

    // Verify limit values
    // offset due to floating point imprecision
    if( limit1 > (-M_PI - 0.00001)  && limit2 < (M_PI + 0.00001)) {
        if(limit1 < limit2) {
            float start_rot = fmax(fmin(init_rotation, limit2), limit1);
            m_segments.push_back( segment(length, start_rot, {limit1, limit2} ) );
            success = true;
        }
    }

    return success;
}

void Chain::rotateSegment(int segmentIdx, float amount)
{
    if(m_segments.size() ) {
        segment& seg = m_segments[segmentIdx];
        seg.rot += amount;

        seg.rot = fmax(fmin(seg.rot, seg.limits[MAX_LIMIT_INDEX]), seg.limits[MIN_LIMIT_INDEX]);
    }
}

static Eigen::Matrix3f createTranslationMatrix(Eigen::Vector2f translation)
{
    return Eigen::Affine2f(Eigen::Translation2f(translation)).matrix();
}

static Eigen::Matrix3f createRotationMatrix(float rotation)
{
    return Eigen::Matrix3f(Eigen::AngleAxisf(rotation, Eigen::Vector3f(0, 0, 1)).toRotationMatrix());
}

void Chain::drawChain(DisplayWindow &window, COLOUR &c)
{
    float start[2], end[2];
    Eigen::Vector2f current = m_origin;

    for(int i = 0; i < m_segments.size(); i++) {
        Eigen::Vector2f segmentEndPos = calculateSegmentEndPosition(i);

        end[0] = segmentEndPos(0);
        end[1] = segmentEndPos(1);

        start[0] = current(0);
        start[1] = current(1);

        WindowLine x(start, end);
        window.drawLine(x, c);

        current = segmentEndPos;
    }
}

Eigen::Vector2f Chain::calculateEffectorPosition()
{
    return calculateSegmentEndPosition(m_segments.size() - 1);
}

static Eigen::Vector2f rotatePointAroundPoint(Eigen::Vector2f position, Eigen::Vector2f joint, float rotation)
{
    Eigen::Matrix3f trMat = createTranslationMatrix(joint) * createRotationMatrix(rotation) * createTranslationMatrix(-joint);
    return (trMat * Eigen::Vector3f(position(0), position(1), 1)).head<2>();
}

Eigen::Vector2f Chain::calculateSegmentStartPosition(int segmentIndex)
{
    if(segmentIndex >= m_segments.size() || segmentIndex < 0)
        return Eigen::Vector2f(0);

    Eigen::Vector2f current = m_origin;
    float totalRotf = 0.0f;

    int currentindex = 0;

    while(currentindex < segmentIndex) {
        segment& seg = m_segments[currentindex++];
        totalRotf += seg.rot;

        current = rotatePointAroundPoint(current + Eigen::Vector2f(seg.length, 0), current, totalRotf);
    }

    return current;
}

Eigen::Vector2f Chain::calculateSegmentEndPosition(int segmentIndex)
{
    if(segmentIndex >= m_segments.size() || segmentIndex < 0)
        return Eigen::Vector2f(0);

    Eigen::Vector2f current = m_origin;
    float totalRotf = 0.0f;

    int currentindex = 0;

    do {
        segment& seg = m_segments[currentindex];
        totalRotf += seg.rot;

        current = rotatePointAroundPoint(current + Eigen::Vector2f(seg.length, 0), current, totalRotf);
    }
    while(currentindex++ < segmentIndex);

    return current;
}

void Chain::solveForTargetIKWithCCD(float targetX, float targetY, unsigned int numIterations, float tolerance)
{
    bool solved = false;
    Eigen::Vector2f targetPos(targetX, targetY);

    for(int i = 0; i < numIterations && !solved; i++) {
        for(int j = m_segments.size(); j > 0; j--) {
            // Get angle between joint-effector & joint-target
            Eigen::Vector2f currentPos = calculateEffectorPosition();
            Eigen::Vector2f jointPos = calculateSegmentStartPosition(j - 1);

            if((currentPos - targetPos).norm() < tolerance ) {
                solved = true;
                continue;
            }

            Eigen::Vector2f targetVec = ( targetPos - jointPos ).normalized();
            Eigen::Vector2f effVec = ( currentPos - jointPos ).normalized();

            float dotP = effVec.dot(targetVec);
            float crossP = effVec.cross(targetVec);

            float costheta =  dotP / (targetVec.norm() * effVec.norm());
            float theta = acos(costheta);

            if(std::isnan(theta))
                break;

            float sign = crossP > 0.0f ? 1.0f : -1.0f;
            rotateSegment(j - 1, (sign * theta) / 5.0f);
        }
    }
}
