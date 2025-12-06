
#include <vector>
#include <math.h>
#include <queue>
#include <unordered_set>

class vector2d
{
public:
    float x, y;
    vector2d() : x(0), y(0) {}
    vector2d(float a, float b) : x(a), y(b) {}

    // basic operations

    vector2d operator+(const vector2d& other) const
    {
        return vector2d(x + other.x, y + other.y);
    }
    vector2d operator-(const vector2d& other) const
    {
        return vector2d(x - other.x, y - other.y);
    }
    vector2d operator*(float scalar) const
    {
        return vector2d(x * scalar, y * scalar);
    }
    vector2d operator/(float scalar) const
    {
        return vector2d(x / scalar, y / scalar);
    }
    bool operator==(const vector2d& other) const
    {
        return x == other.x && y == other.y;
    }
    bool operator!=(const vector2d& other) const
    {
        return !(*this == other);
    }


    // math functions

    inline float magnitude() const
    {
        return sqrt(x * x + y * y);
    }
    inline vector2d normalize() const
    {
        float mag = magnitude();
        return vector2d(x / mag, y / mag);
    }
    inline float dot(const vector2d& other) const
    {
        return x * other.x + y * other.y;
    }
    inline vector2d crossproduct(const vector2d& other) const
    {
        return vector2d(y * other.x - x * other.y, x * other.y - y * other.x);
    }
    inline vector2d perpendicular(int mode) const
    {
        if (mode == 0) // right
            return vector2d(y, -x);
        else // left
            return vector2d(-y, x);
    }

};

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
    vector2d agent_internal_position;
    vector2d agent_internal_direction = vector2d(0, 1); // facing down initially
    float cost = -1;
    static node* goal_node;
    node* previous = nullptr;
    std::vector<node*> neighbors;
    std::vector<node*> a_star_neighbors;
    inline void set_cost(node* b) //const
    { 
        float new_b_cost = b->cost + distance(this, b) + distance(b, goal_node);
        if (b->cost == -1 || b->cost > new_b_cost)
        {
            b->cost = new_b_cost;
            b->previous = this;
        }
    }
    inline static float distance(const node* a, const node* b)
    {
        return sqrt(pow(b->x - a->x, 2) + pow(b->y - a->y, 2));
    }

    // special FOV functions
    // Stack Overflow
    inline int segmentCircleIntersection(const vector2d& p1, const vector2d& p2, const vector2d& center, float radius, vector2d out[2])
    {
        vector2d d = p2 - p1;
        vector2d f = p1 - center;

        float a = d.dot(d);
        float b = 2 * f.dot(d);
        float c = f.dot(f) - radius * radius;

        float disc = b*b - 4*a*c;
        if (disc < 0)
            return 0;

        disc = sqrtf(disc);
        float t1 = (-b - disc) / (2*a);
        float t2 = (-b + disc) / (2*a);

        int count = 0;

        if (t1 >= 0.0f && t1 <= 1.0f) {
            out[count++] = p1 + d * t1;
        }
        if (t2 >= 0.0f && t2 <= 1.0f && disc != 0.0f) {
            out[count++] = p1 + d * t2;
        }

        return count;
    }

    inline std::vector<vector2d> nodeCircleIntersections2(const node* n, const vector2d& circle_center, float circle_radius)
    {
        std::vector<vector2d> intersections;

        // Define voxel corners (node is 1×1 tile)
        vector2d p1 = { float(n->x),     float(n->y)     };
        vector2d p2 = { float(n->x + 1), float(n->y)     };
        vector2d p3 = { float(n->x + 1), float(n->y + 1) };
        vector2d p4 = { float(n->x),     float(n->y + 1) };

        vector2d out[2];

        // Edge 1: p1 → p2
        int c = segmentCircleIntersection(p1, p2, circle_center, circle_radius, out);
        for (int i = 0; i < c; i++) intersections.push_back(out[i]);

        // Edge 2: p2 → p3
        c = segmentCircleIntersection(p2, p3, circle_center, circle_radius, out);
        for (int i = 0; i < c; i++) intersections.push_back(out[i]);

        // Edge 3: p3 → p4
        c = segmentCircleIntersection(p3, p4, circle_center, circle_radius, out);
        for (int i = 0; i < c; i++) intersections.push_back(out[i]);

        // Edge 4: p4 → p1
        c = segmentCircleIntersection(p4, p1, circle_center, circle_radius, out);
        for (int i = 0; i < c; i++) intersections.push_back(out[i]);

        return intersections;
    }


};

class path_node
{
public:
    vector2d position;
    vector2d direction;
    node* parent_node = nullptr;
    path_node* previous = nullptr;
    std::vector<path_node*> neighbors;
    float cost = -1;
    path_node(vector2d pos, vector2d dir, node* parent) : position(pos), direction(dir), parent_node(parent) {}

    inline void set_other_cost(path_node* b)
    {
        if (cost == -1)
        {
            cost = 0;
        }
        float new_b_cost = cost + (position - b->position).magnitude() + (b->position - vector2d(node::goal_node->x, node::goal_node->y)).magnitude();
        if (b->cost == -1 || b->cost > new_b_cost)
        {
            b->cost = new_b_cost;
            b->previous = this;
        }
    }

    inline std::vector<vector2d> nodeCircleIntersections(const path_node* n, const vector2d& circle_center, float circle_radius)
    {
        std::vector<vector2d> intersections;

        // Define voxel corners (node is 1×1 tile)
        vector2d p1 = { float(parent_node->x),     float(parent_node->y)     };
        vector2d p2 = { float(parent_node->x + 1), float(parent_node->y)     };
        vector2d p3 = { float(parent_node->x + 1), float(parent_node->y + 1) };
        vector2d p4 = { float(parent_node->x),     float(parent_node->y + 1) };

        vector2d out[2];

        // Edge 1: p1 → p2
        int c = parent_node->segmentCircleIntersection(p1, p2, circle_center, circle_radius, out);
        for (int i = 0; i < c; i++) intersections.push_back(out[i]);

        // Edge 2: p2 → p3
        c = parent_node->segmentCircleIntersection(p2, p3, circle_center, circle_radius, out);
        for (int i = 0; i < c; i++) intersections.push_back(out[i]);

        // Edge 3: p3 → p4
        c = parent_node->segmentCircleIntersection(p3, p4, circle_center, circle_radius, out);
        for (int i = 0; i < c; i++) intersections.push_back(out[i]);

        // Edge 4: p4 → p1
        c = parent_node->segmentCircleIntersection(p4, p1, circle_center, circle_radius, out);
        for (int i = 0; i < c; i++) intersections.push_back(out[i]);

        return intersections;
    }
};


extern std::vector<std::vector<node*>> map_grid;
extern std::vector<std::vector<node*>> fov_grid;

class AStarComparator
{
public:
    bool operator()(const path_node* a, const path_node* b) const
    {
        return a->cost > b->cost;
    }
};

class AStar
{
public:
    inline static std::vector<node*> get_path(path_node* start_pathnode, path_node* goal_pathnode)//?(node* start, node* goal)
    {
        std::priority_queue<path_node*, std::vector<path_node*>, AStarComparator> frontier;
        std::unordered_set<node*> explored;
        frontier.push(start_pathnode);
        start_pathnode->cost = (start_pathnode->position - goal_pathnode->position).magnitude();
        while (frontier.size() > 0 && frontier.top()->parent_node != node::goal_node)
        {
            path_node* current = frontier.top();
            if (current->parent_node == nullptr)
            {
                frontier.pop();
                continue;
            }
            // -- grid graphics -- //
            if (current->position.y >= 0 && current->position.y < fov_grid.size() &&
                current->position.x >= 0 && current->position.x < fov_grid[0].size())
            {
                fov_grid[int(current->position.y)][int(current->position.x)] = current->parent_node;
            }
            // -- end grid graphics -- //
            frontier.pop();
            explored.insert(current->parent_node);
            // get neighbors
            float radius = 5.5f;
            vector2d circle_pos1 = current->position + (current->direction.perpendicular(0).normalize() * radius);
            vector2d circle_pos2 = current->position + (current->direction.perpendicular(1).normalize() * radius);
            auto intersections_1 = current->nodeCircleIntersections(current, circle_pos1, radius);
            auto intersections_2 = current->nodeCircleIntersections(current, circle_pos2, radius);

            for (auto inter : intersections_1)
            {
                vector2d new_dir = (inter - circle_pos1).normalize().perpendicular(0).normalize();
                if (current->direction.dot(new_dir) <= 0)
                {
                    new_dir = new_dir * -1.0f;//(inter - circle_pos1).normalize().perpendicular(1).normalize();
                }
                inter = inter + new_dir * 0.01f; // nudge a bit to avoid sticking exactly on corners
                if (int(inter.y) >= 0 && int(inter.y) < map_grid.size() &&
                    int(inter.x) >= 0 && int(inter.x) < map_grid[0].size())
                {
                    auto new_pathnode = new path_node(inter, new_dir, map_grid[int(inter.y)][int(inter.x)]);
                    new_pathnode->previous = current;
                    current->neighbors.push_back(new_pathnode);
                }
            }

            for (auto inter : intersections_2)
            {
                vector2d new_dir = (inter - circle_pos2).normalize().perpendicular(1).normalize();
                if (current->direction.dot(new_dir) <= 0)
                {
                    new_dir = new_dir * -1.0f;
                }
                inter = inter + new_dir * 0.01f; // nudge a bit to avoid sticking exactly on corners
                if (int(inter.y) >= 0 && int(inter.y) < map_grid.size() &&
                    int(inter.x) >= 0 && int(inter.x) < map_grid[0].size())
                {
                    auto new_pathnode = new path_node(inter, new_dir, map_grid[int(inter.y)][int(inter.x)]);
                    new_pathnode->previous = current;
                    current->neighbors.push_back(new_pathnode);
                }
            }

            // explore neighbors
            for (auto a_star_neighbor : current->neighbors)
            {
                if (explored.find(a_star_neighbor->parent_node) == explored.end())
                {
                    current->set_other_cost(a_star_neighbor);
                    frontier.push(a_star_neighbor);
                }
            }
        }
        std::vector<node*> path;
        if (frontier.top()->parent_node == node::goal_node)
        {
            std::cout << "Path found!" << std::endl;
        }
        else
        {
            std::cout << "No path found." << std::endl;
            return path;
        }
        path_node* step = frontier.top();
        while (step != nullptr)
        {
            path.push_back(step->parent_node);
            step = step->previous;
        }
        return path;
    }
};

node* create_map(int width, int height, std::vector<obstacle> obs, int start_x, int start_y, int goal_x, int goal_y);
