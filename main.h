
#include <vector>
#include <math.h>
#include <queue>
#include <unordered_set>

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
    float cost = -1;
    static node* goal_node;
    node* previous = nullptr;
    std::vector<node*> neighbors;
    void set_cost(node* b) //const
    { 
        float new_b_cost = b->cost + distance(this, b) + distance(b, goal_node);
        if (b->cost == -1 || b->cost > new_b_cost)
        {
            b->cost = new_b_cost;
            b->previous = this;
        }
    }
    static float distance(const node* a, const node* b)
    {
        return sqrt(pow(b->x - a->x, 2) + pow(b->y - a->y, 2));
    }
};
std::vector<std::vector<node*>> map_grid;

class AStarComparator
{
public:
    bool operator()(const node* a, const node* b) const
    {
        return a->cost > b->cost;
    }
};

class AStar
{
public:
    static std::vector<node*> get_path(node* start, node* goal)
    {
        std::priority_queue<node*, std::vector<node*>, AStarComparator> frontier;
        std::unordered_set<node*> explored;
        frontier.push(start);
        start->cost = start->distance(start, goal);
        while (frontier.top() != goal)
        {
            node* current = frontier.top();
            frontier.pop();
            explored.insert(current);
            for (auto neighbor : current->neighbors)
            {
                if (explored.find(neighbor) == explored.end())
                {
                    current->set_cost(neighbor);
                    frontier.push(neighbor);
                }
            }
        }
    }
};

node* create_map(int width, int height, std::vector<obstacle> obs, int start_x, int start_y, int goal_x, int goal_y);
