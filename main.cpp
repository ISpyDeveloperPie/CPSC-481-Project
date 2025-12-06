#include <iostream>
#include "main.h"

node* node::goal_node = nullptr;
std::vector<std::vector<node*>> map_grid;
std::vector<std::vector<node*>> fov_grid;

int global_width = 0;
int global_height = 0;

// -- MAP CREATION -- //

node* create_map(int width, int height, std::vector<obstacle> obs, int start_x, int start_y, int goal_x, int goal_y) 
{
    map_grid.resize(height, std::vector<node*>(width, nullptr));
    fov_grid.resize(height, std::vector<node*>(width, nullptr));
    node* start_node = nullptr;
    std::cout << "\n\n----------------------" << std::endl;
    for (int y = 0; y < height; ++y) 
    {
        for (int x = 0; x < width; ++x) 
        {
            bool skip = false;
            for (auto obs : obs) 
            {
                if (y >= obs.x && y < obs.x + obs.width && x >= obs.y && x < obs.y + obs.height) 
                {
                    std::cout << "⬛";
                    skip = true;
                }
            }
            if (!skip)
            {
                std::cout << "⬜";
                node* newNode = new node();
                newNode->x = x;
                newNode->y = y;
                if (x > 0 && map_grid[y][x - 1] != nullptr) 
                {
                    newNode->neighbors.push_back(map_grid[y][x - 1]);
                    map_grid[y][x - 1]->neighbors.push_back(newNode);
                }
                if (y > 0 && map_grid[y - 1][x] != nullptr) 
                {
                    newNode->neighbors.push_back(map_grid[y - 1][x]);
                    map_grid[y - 1][x]->neighbors.push_back(newNode);
                }
                map_grid[y][x] = newNode;
                if (x == start_x && y == start_y) 
                {
                    start_node = newNode;
                }
                else if (x == goal_x && y == goal_y) 
                {
                    node::goal_node = newNode;
                }
            }
        }
        std::cout << std::endl;
    }
    std::cout << "----------------------\n\n" << std::endl; 
    std::cout << "Map created!" << std::endl; 
    global_height = height;
    global_width = width;
    return start_node;
}

// -- PATH PRINTING -- //

void print_path(const std::vector<node*>& path) 
{
    for (int i = path.size() - 1; i >= 0; i--)
    {
        auto n = path[i];
        std::cout << "(" << n->x << ", " << n->y << ") -> ";
    }
    std::cout << "GOAL\n\n" << std::endl;

    for (int y = 0; y < global_height; ++y) 
    {
        for (int x = 0; x < global_width; ++x) 
        {
            bool on_path = false;
            for (auto p_node : path) 
            {
                if (p_node->x == x && p_node->y == y) 
                {
                    on_path = true;
                    break;
                }
            }
            if (on_path && map_grid[y][x] != node::goal_node) 
            {
                std::cout << "\x1b[32m██\x1b[0m";
            } 
            else 
            {
                bool is_obstacle = true;
                for (auto p_node : path) 
                {
                    if (map_grid[y][x] == p_node) 
                    {
                        is_obstacle = false;
                        break;
                    }
                }
                if (is_obstacle && map_grid[y][x] == nullptr) 
                {
                    std::cout << "\x1b[30m██\x1b[0m";
                } 
                else if (fov_grid[y][x] != nullptr && map_grid[y][x] != node::goal_node) 
                {
                    std::cout << "\x1b[34m██\x1b[0m";
                }
                else if (map_grid[y][x] == node::goal_node) 
                {
                    std::cout << "\x1b[31m██\x1b[0m";
                }
                else 
                {
                    std::cout << "\x1b[37m██\x1b[0m";
                }
            }
        }
        std::cout << std::endl;
    }
}

// -- TURN-BASED FOV -- //



// -- MAIN FUNCTION -- //

int main() 
{
    std::cout << "Hello, World!" << std::endl; 
    std::vector<obstacle> obstacles;
    auto obs1 = obstacle(0, 3, 2, 2);
    //auto obs2 = obstacle(3, 0, 3, 4);
    obstacles.push_back(obs1);
    //obstacles.push_back(obs2);
    int goal_x = 9;
    int goal_y = 3;
    auto start = create_map(10, 10, obstacles, 0, 0, goal_x, goal_y);
    path_node* start_pathnode = new path_node(vector2d(start->x, start->y), vector2d(0,1), map_grid[0][0]);
    path_node* goal_pathnode = new path_node(vector2d(node::goal_node->x, node::goal_node->y), vector2d(0,0), map_grid[goal_y][goal_x]);
    fov_grid[start->y][start->x] = start;
    auto path = AStar::get_path(start_pathnode, goal_pathnode);
    print_path(path);
    std::cout << "EXIT NOW" << std::endl;
    return 0;
}