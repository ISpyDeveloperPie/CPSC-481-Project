#include <iostream>
#include "main.h"
#include <cmath>
#include <queue>
#include <set>
#include <algorithm>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Get node from grid safely
node* get_node(int x, int y)
{
    if (x >= 0 && x < (int)map_grid[0].size() && y >= 0 && y < (int)map_grid.size())
    {
        return map_grid[y][x];
    }
    return nullptr;
}

// Check if a node position is blocked by an obstacle
bool is_node_blocked(int x, int y, std::vector<obstacle> obs)
{
    for (size_t i = 0; i < obs.size(); ++i)
    {
        if (x >= obs[i].x && x < obs[i].x + obs[i].width &&
            y >= obs[i].y && y < obs[i].y + obs[i].height)
        {
            return true;
        }
    }
    return false;
}

// Calculate movement cost based on direction
// Orthogonal moves (up/down/left/right) cost 1.0
// Diagonal moves cost sqrt(2) ≈ 1.414
double calculate_move_cost(int dx, int dy)
{
    if ((dx != 0 && dy != 0))
    {
        // Diagonal move
        return std::sqrt(2.0);
    }
    // Orthogonal move
    return 1.0;
}

// Calculate heuristic distance (Euclidean distance)
double calculate_heuristic(node* current, node* goal)
{
    double dx = current->x - goal->x;
    double dy = current->y - goal->y;
    return std::sqrt(dx * dx + dy * dy);
}

// Calculate the angle (in degrees) between three points
// Returns the angle at 'current' formed by prev->current->next
double calculate_turn_angle(node* prev, node* current, node* next)
{
    if (prev == nullptr || current == nullptr || next == nullptr)
        return 0.0;
    
    // Vector from prev to current
    double v1x = current->x - prev->x;
    double v1y = current->y - prev->y;
    
    // Vector from current to next
    double v2x = next->x - current->x;
    double v2y = next->y - current->y;
    
    // Normalize vectors
    double v1_len = std::sqrt(v1x * v1x + v1y * v1y);
    double v2_len = std::sqrt(v2x * v2x + v2y * v2y);
    
    if (v1_len < 1e-9 || v2_len < 1e-9)
        return 0.0;
    
    v1x /= v1_len;
    v1y /= v1_len;
    v2x /= v2_len;
    v2y /= v2_len;
    
    // Calculate angle using dot product
    double dot = v1x * v2x + v1y * v2y;
    dot = std::max(-1.0, std::min(1.0, dot)); // Clamp to [-1, 1]
    
    double angle_rad = std::acos(dot);
    double angle_deg = angle_rad * 180.0 / M_PI;
    
    return angle_deg;
}

// A* Pathfinding Algorithm with turning radius penalty
std::vector<node*> a_star(node* start, node* goal)
{
    nodes_explored = 0;
    std::vector<node*> path;
    
    if (start == nullptr || goal == nullptr)
        return path;
    
    // Priority queue for open set (sorted by f_cost)
    std::priority_queue<node*, std::vector<node*>, std::greater<node*>> open_set;
    std::set<node*> closed_set;
    std::set<node*> open_set_tracker;
    
    start->g_cost = 0;
    start->h_cost = calculate_heuristic(start, goal);
    start->f_cost = start->h_cost;
    start->parent = nullptr;
    
    open_set.push(start);
    open_set_tracker.insert(start);
    
    while (!open_set.empty())
    {
        node* current = open_set.top();
        open_set.pop();
        open_set_tracker.erase(current);
        
        nodes_explored++;
        
        // Goal reached
        if (current == goal)
        {
            // Reconstruct path
            node* temp = goal;
            while (temp != nullptr)
            {
                path.insert(path.begin(), temp);
                temp = temp->parent;
            }
            return path;
        }
        
        closed_set.insert(current);
        
        // Explore 8 neighbors (4 orthogonal + 4 diagonal)
        int dx_list[] = {-1, 0, 1, 0, -1, 1, 1, -1};
        int dy_list[] = {0, -1, 0, 1, -1, -1, 1, 1};
        
        for (int i = 0; i < 8; ++i)
        {
            int nx = current->x + dx_list[i];
            int ny = current->y + dy_list[i];
            
            node* neighbor = get_node(nx, ny);
            
            if (neighbor == nullptr || closed_set.find(neighbor) != closed_set.end())
                continue;
            
            // Calculate movement cost (1.0 for orthogonal, sqrt(2) for diagonal)
            double movement_cost = calculate_move_cost(dx_list[i], dy_list[i]);
            
            // Apply turning radius penalty for harsh turns (> 90 degrees)
            double turn_penalty = 1.0;
            if (current->parent != nullptr)
            {
                double angle = calculate_turn_angle(current->parent, current, neighbor);
                
                // Penalize turns > 90 degrees heavily
                // Angles between 90-180 degrees get increasing penalty
                if (angle > 90.0)
                {
                    // Heavy penalty for turns > 90 degrees
                    turn_penalty = 1.0 + (angle - 90.0) / 90.0 * 3.0; // Scale from 1.0 to 4.0
                }
            }
            
            double tentative_g = current->g_cost + (movement_cost * turn_penalty);
            
            // If neighbor is in open set and we found a worse path, skip
            if (open_set_tracker.find(neighbor) != open_set_tracker.end())
            {
                if (tentative_g >= neighbor->g_cost)
                    continue;
            }
            
            // Update neighbor
            neighbor->parent = current;
            neighbor->g_cost = tentative_g;
            neighbor->h_cost = calculate_heuristic(neighbor, goal);
            neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
            
            if (open_set_tracker.find(neighbor) == open_set_tracker.end())
            {
                open_set.push(neighbor);
                open_set_tracker.insert(neighbor);
            }
        }
    }
    
    // No path found
    return path;
}

void create_map(int width, int height, std::vector<obstacle> obs) 
{
    map_grid.resize(height, std::vector<node*>(width, nullptr));

    std::cout << "\n\n" << std::string(40, '-') << std::endl;
    for (int y = 0; y < height; ++y) 
    {
        for (int x = 0; x < width; ++x) 
        {
            bool skip = false;
            for (size_t i = 0; i < obs.size(); ++i)
            {
                if (x >= obs[i].x && x < obs[i].x + obs[i].width && 
                    y >= obs[i].y && y < obs[i].y + obs[i].height) 
                {
                    std::cout << "[#]";
                    skip = true;
                }
            }
            if (!skip)
            {
                std::cout << "[ ]";
                node* newNode = new node();
                newNode->x = x;
                newNode->y = y;
                
                // Connect to left neighbor
                if (x > 0 && map_grid[y][x - 1] != nullptr) 
                {
                    newNode->neighbors.push_back(map_grid[y][x - 1]);
                    map_grid[y][x - 1]->neighbors.push_back(newNode);
                }
                
                // Connect to top neighbor
                if (y > 0 && map_grid[y - 1][x] != nullptr) 
                {
                    newNode->neighbors.push_back(map_grid[y - 1][x]);
                    map_grid[y - 1][x]->neighbors.push_back(newNode);
                }
                
                // Connect to top-left diagonal
                if (x > 0 && y > 0 && map_grid[y - 1][x - 1] != nullptr) 
                {
                    newNode->neighbors.push_back(map_grid[y - 1][x - 1]);
                    map_grid[y - 1][x - 1]->neighbors.push_back(newNode);
                }
                
                // Connect to top-right diagonal
                if (x < width - 1 && y > 0 && map_grid[y - 1][x + 1] != nullptr) 
                {
                    newNode->neighbors.push_back(map_grid[y - 1][x + 1]);
                    map_grid[y - 1][x + 1]->neighbors.push_back(newNode);
                }
                
                map_grid[y][x] = newNode;
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::string(40, '-') << "\n\n" << std::endl; 
    std::cout << "Map created successfully!" << std::endl; 
}

void display_path(int width, int height, std::vector<node*> path, std::vector<obstacle> obs, node* start, node* goal)
{
    // Create display grid
    std::vector<std::vector<char>> display(height, std::vector<char>(width, '.'));
    
    // Mark obstacles
    for (size_t i = 0; i < obs.size(); ++i)
    {
        for (int x = obs[i].x; x < obs[i].x + obs[i].width && x < width; ++x)
        {
            for (int y = obs[i].y; y < obs[i].y + obs[i].height && y < height; ++y)
            {
                display[y][x] = '#';
            }
        }
    }
    
    // Mark path
    if (!path.empty())
    {
        for (size_t i = 0; i < path.size(); ++i)
        {
            if (i == 0)
                display[path[i]->y][path[i]->x] = 'S'; // Start
            else if (i == path.size() - 1)
                display[path[i]->y][path[i]->x] = 'G'; // Goal
            else
                display[path[i]->y][path[i]->x] = '*'; // Path
        }
    }
    
    // Display
    std::cout << "\n\n" << std::string(40, '=') << std::endl;
    std::cout << "FINAL PATH VISUALIZATION" << std::endl;
    std::cout << std::string(40, '=') << "\n" << std::endl;
    
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            std::cout << display[y][x];
        }
        std::cout << std::endl;
    }
    std::cout << "\n" << std::string(40, '=') << std::endl;
    
    // Print statistics
    std::cout << "\nPath Statistics:" << std::endl;
    std::cout << "  Start: S | Goal: G | Path: * | Obstacle: #" << std::endl;
    std::cout << "  Path length: " << path.size() << " nodes" << std::endl;
    std::cout << "  Total path cost: " << std::fixed << std::setprecision(2) << (path.empty() ? 0 : path.back()->g_cost) << std::endl;
    std::cout << "  Nodes explored: " << nodes_explored << std::endl;
    std::cout << std::string(40, '=') << "\n" << std::endl;
}

int main() 
{
    std::cout << "\n" << std::string(50, '=') << std::endl;
    std::cout << "  A* PATHFINDING WITH TURNING RADIUS CONSTRAINTS" << std::endl;
    std::cout << std::string(50, '=') << "\n" << std::endl;
    
    // Create obstacles
    std::vector<obstacle> obstacles;
    obstacle obs1(1, 0, 2, 2);
    obstacle obs2(3, 0, 3, 4);
    obstacles.push_back(obs1);
    obstacles.push_back(obs2);
    
    int map_width = 10;
    int map_height = 10;
    
    // Create map
    std::cout << "Creating map (" << map_width << "x" << map_height << ")..." << std::endl;
    create_map(map_width, map_height, obstacles);
    
    // Get user input for start and goal nodes
    int start_x, start_y, goal_x, goal_y;
    
    std::cout << "Enter start node coordinates (x y): ";
    std::cin >> start_x >> start_y;
    
    std::cout << "Enter goal node coordinates (x y): ";
    std::cin >> goal_x >> goal_y;
    
    node* start = get_node(start_x, start_y);
    node* goal = get_node(goal_x, goal_y);
    
    // Validate nodes
    if (start == nullptr)
    {
        std::cout << "\nError: Start node (" << start_x << ", " << start_y << ") is blocked or invalid!" << std::endl;
        return 1;
    }
    
    if (goal == nullptr)
    {
        std::cout << "\nError: Goal node (" << goal_x << ", " << goal_y << ") is blocked or invalid!" << std::endl;
        return 1;
    }
    
    std::cout << "\n" << std::string(50, '-') << std::endl;
    std::cout << "Start node: (" << start->x << ", " << start->y << ")" << std::endl;
    std::cout << "Goal node: (" << goal->x << ", " << goal->y << ")" << std::endl;
    std::cout << std::string(50, '-') << "\n" << std::endl;
    
    // Run A* algorithm
    std::cout << "Running A* pathfinding algorithm..." << std::endl;
    std::vector<node*> path = a_star(start, goal);
    
    // Display results
    if (!path.empty())
    {
        std::cout << "SUCCESS: Path found!\n" << std::endl;
        display_path(map_width, map_height, path, obstacles, start, goal);
    }
    else
    {
        std::cout << "\nFAILURE: No path found between start and goal!" << std::endl;
        std::cout << "Nodes explored: " << nodes_explored << std::endl;
    }
    
    return 0;
}