#pragma once

#include <vector>

class obstacle
{
public:
    int x, y, width, height;
    obstacle() : x(0), y(0), width(1), height(1) {}
    obstacle(int a, int b, int c, int d) : x(a), y(b), width(c), height(d) {}
};

class node
{
public:
    int x, y;
    std::vector<node*> neighbors;
};

std::vector<std::vector<node*>> map_grid;

void create_map(int width, int height, std::vector<obstacle> obs);
