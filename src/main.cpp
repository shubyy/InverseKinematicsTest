#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "DisplayWindow.h"
#include "Chain.h"

#include <unistd.h>
#include <iostream>

int main()
{
    DisplayWindow window("Inverse Kinematics Test", 800, 500);

    Chain customChain(400.0f, 250.0f);
    COLOUR backColour{0, 0, 0, SDL_ALPHA_OPAQUE};
    COLOUR chainColour{255, 255, 200, SDL_ALPHA_OPAQUE};
    COLOUR targetColour{255, 0, 255, SDL_ALPHA_OPAQUE};

    window.setClearColour(backColour);

    //add segments
    customChain.addSegment(40.0f, M_PI / 4.0f);
    customChain.addSegment(40.0f, 0);
    customChain.addSegment(40.0f, 0);
    customChain.addSegment(40.0f, 0);
    customChain.addSegment(40.0f, 0.0f);
    customChain.addSegment(40.0f, M_PI / 4.0f);
    customChain.addSegment(20.0f, M_PI / 4.0f);

    //printf("Target Distance: %f\n", glm::length(target - glm::vec2(50.0f, 250.0f)));
    Eigen::Vector2f target(500, 400);
    //customChain.solveForTargetIKWithCCD(target, 40);

    float point[2];
    point[0] = target(0);
    point[1] = target(1);
    WindowPoint targetPoint(point, 3.0f);

    while (!window.shouldQuit()){
        window.clear();

        float targetX, targetY;
        window.getMousePosition(targetX, targetY);

        customChain.solveForTargetIKWithCCD(Eigen::Vector2f(targetX, targetY), 10);
        customChain.drawChain(window, chainColour);

        window.drawPoint(targetPoint, targetColour);


        window.runWindowLoop();

        usleep(1000 * 10);
    }

    return 0;
}
