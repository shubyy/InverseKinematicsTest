#include "DisplayWindow.h"

bool DisplayWindow::SDLInitialised = false;

static void print_sdl_error()
{
    printf("Error: %s\n", SDL_GetError());
}

DisplayWindow::DisplayWindow(const char *title, int width, int height)
{
    int success = true;
    if(!SDLInitialised) {
        if(!SDL_Init(SDL_INIT_VIDEO)) {
            print_sdl_error();
            return;
        }
    }

    success = SDL_CreateWindowAndRenderer(title, width, height, 0, &window, &renderer);
    if(!success) {
        print_sdl_error();
        return;
    }

    quit = false;
    mouseX = 0;
    mouseY = 0;
}

DisplayWindow::~DisplayWindow()
{
}

bool DisplayWindow::shouldQuit()
{
    return quit;
}

void DisplayWindow::drawLine(WindowLine& l, COLOUR& c)
{
    SDL_SetRenderScale( renderer, 1, 1);
    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
    SDL_RenderLine(renderer, l.start[0], l.start[1], l.end[0], l.end[1]);
}

void DisplayWindow::drawPoint(WindowPoint &circ, COLOUR &c)
{
    SDL_SetRenderScale( renderer, 1, 1);
    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);

    for(int x = -circ.radius; x < circ.radius; x++) {
        for(int y = -circ.radius; y < circ.radius; y++) {
            SDL_RenderPoint(renderer, circ.pos[0] + x, circ.pos[1] + y);
        }
    }
}


void DisplayWindow::getMousePosition(float& x, float& y)
{
    x = mouseX;
    y = mouseY;
}

void DisplayWindow::setClearColour(COLOUR& col)
{
    clearColour = col;
}

void DisplayWindow::clear()
{
    SDL_SetRenderDrawColor(renderer, clearColour.r, clearColour.g, clearColour.b, clearColour.a);
    SDL_RenderClear(renderer);
}

void DisplayWindow::runWindowLoop()
{
    SDL_RenderPresent(renderer);

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_EVENT_QUIT){
            quit = true;
        }

        if (e.type == SDL_EVENT_MOUSE_MOTION){
            mouseX = e.motion.x;
            mouseY = e.motion.y;
        }
    }
}
