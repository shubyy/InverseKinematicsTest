#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "DisplayWindow.h"
#include "Chain.h"

#include <unistd.h>

int main()
{
    DisplayWindow window("Inverse Kinematics Test", 800, 500);

    Chain customChain1(400.0f, 250.0f);
    Chain customChain2(400.0f, 250.0f);

    COLOUR backColour{0, 0, 0, SDL_ALPHA_OPAQUE};
    COLOUR chain1Colour{255, 0, 0, SDL_ALPHA_OPAQUE};
    COLOUR chain2Colour{0, 255, 0, SDL_ALPHA_OPAQUE};
    COLOUR targetColour{255, 0, 255, SDL_ALPHA_OPAQUE};

    window.setClearColour(backColour);

    //add segments
    customChain1.addSegment(40.0f, M_PI / 4.0f);
    customChain1.addSegment(40.0f, 0);
    customChain1.addSegment(40.0f, 0);
    customChain1.addSegment(40.0f, 0);
    customChain1.addSegment(40.0f, 0.0f);
    customChain1.addSegment(40.0f, M_PI / 4.0f);
    customChain1.addSegment(20.0f, M_PI / 4.0f);

    //add segments
    customChain2.addSegment(40.0f, M_PI / 4.0f);
    customChain2.addSegment(40.0f, 0);
    customChain2.addSegment(40.0f, 0);
    customChain2.addSegment(40.0f, 0);
    customChain2.addSegment(40.0f, 0.0f);
    customChain2.addSegment(40.0f, M_PI / 4.0f);
    customChain2.addSegment(20.0f, M_PI / 4.0f);

    while (!window.shouldQuit()){
        window.clear();

        float targetX, targetY;
        window.getMousePosition(targetX, targetY);

        // test 2 different chains

        // chain 1 solved back -> forward
        customChain1.solveForTargetIKWithCCD(Eigen::Vector2f(targetX, targetY), 10);
        customChain1.drawChain(window, chain1Colour);

        // chain 2 solved forward -> back
        customChain2.solveForTargetIKWithCCD(Eigen::Vector2f(targetX, targetY), 10, true);
        customChain2.drawChain(window, chain2Colour);

        window.runWindowLoop();

        usleep(1000 * 10);
    }

    return 0;
}
