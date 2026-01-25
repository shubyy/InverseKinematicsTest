#include "DisplayWindow.h"
#include "Chain.h"

#include <unistd.h>

int main()
{
    COLOUR backColour{0, 0, 0, SDL_ALPHA_OPAQUE};
    COLOUR chain1Colour{255, 0, 0, SDL_ALPHA_OPAQUE};
    COLOUR targetColour{255, 0, 255, SDL_ALPHA_OPAQUE};

    DisplayWindow window("Inverse Kinematics Test", 800, 500);
    window.setClearColour(backColour);

    Chain customChain1(400.0f, 250.0f);

    //add segments
    customChain1.addSegment(40.0f, M_PI / 8.0f, -M_PI_2, M_PI_2);
    customChain1.addSegment(40.0f, 0, -M_PI_2, M_PI_2);
    customChain1.addSegment(40.0f, 0, -M_PI_4, M_PI_4);
    customChain1.addSegment(40.0f, 0, -M_PI_4, M_PI_4);
    customChain1.addSegment(40.0f, 0.0f, -M_PI_4, M_PI_4);
    customChain1.addSegment(40.0f, M_PI / 8.0f, -M_PI_4, M_PI_4);
    customChain1.addSegment(20.0f, M_PI / 8.0f, -M_PI_4, M_PI_4);

    while (!window.shouldQuit()){
        window.clear();

        float targetX, targetY;
        window.getMousePosition(targetX, targetY);

        // chain 1
        customChain1.solveForTargetIKWithCCD(targetX, targetY, 10);
        customChain1.drawChain(window, chain1Colour);

        window.runWindowLoop();

        usleep(1000 * 5);
    }

    return 0;
}
