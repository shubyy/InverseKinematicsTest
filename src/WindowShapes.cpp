#include "WindowShapes.h"

WindowLine::WindowLine(float start[2], float end[2])
{
    this->start[0] = start[0];
    this->start[1] = start[1];
    this->end[0] = end[0];
    this->end[1] = end[1];
}

WindowPoint::WindowPoint(float pos[2], float c_radius)
{
    this->pos[0] = pos[0];
    this->pos[1] = pos[1];
    radius = c_radius;
}
