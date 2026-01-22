#pragma once


class WindowLine
{
private:
/* data */
public:
    float start[2];
    float end[2];

    WindowLine(float start_pos[2], float end_pos[2]);
};

class WindowPoint
{
private:
    /* data */
public:
    float pos[2];
    float radius;

    WindowPoint(float pos[2], float radius);
};
