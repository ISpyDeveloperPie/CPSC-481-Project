#pragma once

#include <vector>
#include <cmath>
#include <queue>
#include <set>

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
    node* parent;
    double g_cost; // Cost from start
    double h_cost; // Heuristic cost to goal
    double f_cost; // g_cost + h_cost
    
    node() : x(0), y(0), parent(nullptr), g_cost(0), h_cost(0), f_cost(0) {}
    
    bool operator>(const node& other) const
    {
        return f_cost > other.f_cost;
    }
};

std::vector<std::vector<node*>> map_grid;
int nodes_explored;

void create_map(int width, int height, std::vector<obstacle> obs);
std::vector<node*> a_star(node* start, node* goal);
double calculate_turn_angle(node* prev, node* current, node* next);
double calculate_heuristic(node* current, node* goal);
double calculate_move_cost(int dx, int dy);
void display_path(int width, int height, std::vector<node*> path, std::vector<obstacle> obs, node* start, node* goal);
node* get_node(int x, int y);
bool is_node_blocked(int x, int y, std::vector<obstacle> obs);
