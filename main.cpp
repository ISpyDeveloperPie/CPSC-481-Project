#include <iostream>
#include "main.h"

node* node::goal_node = nullptr;

int global_width = 0;
int global_height = 0;

// -- MAP CREATION -- //

node* create_map(int width, int height, std::vector<obstacle> obs, int start_x, int start_y, int goal_x, int goal_y) 
{
    map_grid.resize(height, std::vector<node*>(width, nullptr));
    node* start_node = nullptr;
    std::cout << "\n\n----------------------" << std::endl;
    for (int x = 0; x < height; ++x) 
    {
        for (int y = 0; y < width; ++y) 
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
                newNode->x = y;
                newNode->y = x;
                if (x > 0 && map_grid[x - 1][y] != nullptr) 
                {
                    newNode->neighbors.push_back(map_grid[x - 1][y]);
                    map_grid[x - 1][y]->neighbors.push_back(newNode);
                }
                if (y > 0 && map_grid[x][y - 1] != nullptr) 
                {
                    newNode->neighbors.push_back(map_grid[x][y - 1]);
                    map_grid[x][y - 1]->neighbors.push_back(newNode);
                }
                map_grid[x][y] = newNode;
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

    for (int x = 0; x < global_height; ++x) 
    {
        for (int y = 0; y < global_width; ++y) 
        {
            bool on_path = false;
            for (auto p_node : path) 
            {
                if (p_node->x == y && p_node->y == x) 
                {
                    on_path = true;
                    break;
                }
            }
            if (on_path) 
            {
                std::cout << "🟩";
            } 
            else 
            {
                bool is_obstacle = true;
                for (auto p_node : path) 
                {
                    if (map_grid[x][y] == p_node) 
                    {
                        is_obstacle = false;
                        break;
                    }
                }
                if (is_obstacle && map_grid[x][y] == nullptr) 
                {
                    std::cout << "⬛";
                } 
                else 
                {
                    std::cout << "⬜";
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
    auto obs1 = obstacle(1, 0, 2, 2);
    auto obs2 = obstacle(3, 0, 3, 4);
    obstacles.push_back(obs1);
    obstacles.push_back(obs2);
    int goal_x = 9;
    int goal_y = 9;
    auto start = create_map(10, 10, obstacles, 0, 0, goal_x, goal_y);
    auto path = AStar::get_path(map_grid[0][0], map_grid[goal_x][goal_y]);
    print_path(path);
    return 0;
}