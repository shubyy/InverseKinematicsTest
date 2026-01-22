#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "DisplayWindow.h"
#include "Chain.h"

#include <unistd.h>

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

    while (!window.shouldQuit()){
        window.clear();

        float targetX, targetY;
        window.getMousePosition(targetX, targetY);

        customChain.solveForTargetIKWithCCD(Eigen::Vector2f(targetX, targetY), 10);
        customChain.drawChain(window, chainColour);

        window.runWindowLoop();

        usleep(1000 * 10);
    }

    return 0;
}
