#include "Chain.h"
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Dense>


Chain::Chain(int origin_x, int origin_y)
{
    m_origin = Eigen::Vector2f(origin_x, origin_y);
}

void Chain::addSegment(float length, float rotation)
{
    m_segments.push_back( segment(length, rotation) );
}

void Chain::rotateSegment(int segmentIdx, float amount)
{
    if(m_segments.size() )
        m_segments[segmentIdx].rot += amount;
}

static Eigen::Matrix3f createTranslationMatrix(Eigen::Vector2f translation)
{
    return Eigen::Affine2f(Eigen::Translation2f(translation)).matrix();
}

void Chain::drawChain(DisplayWindow &window, COLOUR &c)
{
    Eigen::Vector2f current = m_origin;
    float totalRotf = 0.0f;
    for(const auto& seg : m_segments) {
        totalRotf += seg.rot;
        Eigen::Matrix3f reverseTranslate = createTranslationMatrix(current);
        Eigen::Matrix3f rotMat = Eigen::Matrix3f(Eigen::AngleAxisf(totalRotf, Eigen::Vector3f(0, 0, 1)).toRotationMatrix());
        Eigen::Matrix3f translate = createTranslationMatrix(-current);

        Eigen::Vector3f end_1 = Eigen::Vector3f(current(0) + seg.length, current(1), 1);

        end_1 = translate * end_1;
        end_1 = rotMat * end_1;
        end_1 = reverseTranslate * end_1;

        float start[2], end[2];
        end[0] = end_1(0);
        end[1] = end_1(1);

        start[0] = current(0);
        start[1] = current(1);

        WindowLine x(start, end);
        window.drawLine(x, c);
        current = Eigen::Vector2f(end_1.x(), end_1.y());
    }
}

Eigen::Vector2f Chain::calculateEffectorPosition()
{
    Eigen::Vector2f current = m_origin;
    float totalRotf = 0.0f;
    for(const auto& seg : m_segments) {
        totalRotf += seg.rot;
        Eigen::Matrix3f reverseTranslate = createTranslationMatrix(current);
        Eigen::Matrix3f rotMat = Eigen::Matrix3f(Eigen::AngleAxisf(totalRotf, Eigen::Vector3f(0, 0, 1)).toRotationMatrix());
        Eigen::Matrix3f translate = createTranslationMatrix(-current);

        Eigen::Vector3f end_1 = Eigen::Vector3f(current(0) + seg.length, current(1), 1);

        end_1 = translate * end_1;
        end_1 = rotMat * end_1;
        end_1 = reverseTranslate * end_1;

        current(0) = end_1(0);
        current(1) = end_1(1);
    }

    return current;
}

Eigen::Vector2f Chain::calculateSegmentPosition(int segmentIndex)
{
    if(segmentIndex >= m_segments.size() || segmentIndex < 0)
        return Eigen::Vector2f(0);

    Eigen::Vector2f current = m_origin;
    float totalRotf = 0.0f;

    int currentindex = 0;

    do {
        segment& seg = m_segments[currentindex++];
        totalRotf += seg.rot;

        Eigen::Matrix3f reverseTranslate = createTranslationMatrix(current);
        Eigen::Matrix3f rotMat = Eigen::Matrix3f(Eigen::AngleAxisf(totalRotf, Eigen::Vector3f(0, 0, 1)).toRotationMatrix());
        Eigen::Matrix3f translate = createTranslationMatrix(-current);

        Eigen::Vector3f end_1 = Eigen::Vector3f(current(0) + seg.length, current(1), 1);

        end_1 = translate * end_1;
        end_1 = rotMat * end_1;
        end_1 = reverseTranslate * end_1;

        current(0) = end_1(0);
        current(1) = end_1(1);
    }
    while(currentindex < segmentIndex);

    return current;
}

void Chain::solveForTargetIKWithCCD(Eigen::Vector2f targetPos, unsigned int numIterations)
{
    bool solved = false;
    for(int i = 0; i < numIterations; i++) {
        if(solved) {
            printf("Iterations needed: %d\n", i);
            break;
        }

        for(int j = 0; j < m_segments.size(); j++) {
            // Get angle between joint-effector & joint-target
            Eigen::Vector2f currentPos = calculateEffectorPosition();
            Eigen::Vector2f jointPos = calculateSegmentPosition(j);

            if((currentPos - targetPos).norm() < 1.0f )
                solved = true;

            Eigen::Vector2f targetVec = ( targetPos - jointPos ).normalized();
            Eigen::Vector2f effVec = ( currentPos - jointPos ).normalized();

            //float dotP = glm::dot(effVec, targetVec);
            float dotP = effVec.dot(targetVec);
            float crossP = effVec.cross(targetVec);

            float costheta =  dotP / (targetVec.norm() * effVec.norm());
            float theta = acos(costheta);

            //printf("Segment %d dot product: %.2f\n",  );

            if(std::isnan(theta))
                break;

            float sign = crossP > 0 ? 1.0f : -1.0f;

            m_segments[j].rot += (sign * theta) / 5.0f;
        }
    }

}
