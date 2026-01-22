#pragma once
#include <SDL3/SDL.h>
#include "WindowShapes.h"

typedef struct _colour {
    int r;
    int g;
    int b;
    int a;
} COLOUR;

class DisplayWindow
{
private:
    static bool SDLInitialised;
    bool quit;

    SDL_Window *window;
    SDL_Renderer *renderer;

    float mouseX;
    float mouseY;

    COLOUR clearColour;

public:
    DisplayWindow(const char *title, int width, int height);
    ~DisplayWindow();

    bool shouldQuit();

    void drawLine(WindowLine& l, COLOUR& c);
    void drawPoint(WindowPoint& circ, COLOUR& c);

    void getMousePosition(float& x, float& y);

    void setClearColour(COLOUR& col);
    void clear();
    void runWindowLoop();
};
