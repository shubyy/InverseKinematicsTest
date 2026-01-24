#include "DisplayWindow.h"
#include "Chain.h"

#include <unistd.h>

int main()
{
    DisplayWindow window("Inverse Kinematics Test", 800, 500);

    Chain customChain1(400.0f, 250.0f);

    COLOUR backColour{0, 0, 0, SDL_ALPHA_OPAQUE};
    COLOUR chain1Colour{255, 0, 0, SDL_ALPHA_OPAQUE};
    COLOUR targetColour{255, 0, 255, SDL_ALPHA_OPAQUE};

    window.setClearColour(backColour);

    //add segments
    customChain1.addSegment(40.0f, M_PI / 4.0f);
    //customChain1.addSegment(40.0f, 0);
    customChain1.addSegment(40.0f, 0);
    customChain1.addSegment(40.0f, 0);
    customChain1.addSegment(40.0f, 0.0f);
    customChain1.addSegment(40.0f, M_PI / 4.0f);
    customChain1.addSegment(20.0f, M_PI / 4.0f);

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
