### A* Pathfinding with Turning Radius
- This project was done for the CPSC-481 "Introduction into AI" class at CSUF.
- Given a map, initial position, initial direction, maximum turning radius, and goal, the program will attempt to find a path while accounting for its maximum turning radius limitation.

### How to use
- Edit the start position, direction, map, and goal position within the main.cpp file.
- Run the main.exe program.
- You will see an ASCII map.
    - The white squares are unexplored nodes.
    - The black/gray squares are obstacles.
    - The yellow/orange square is the starting node.
    - The red square is the goal node.
    - The blue squares are explored nodes.
    - The green squares are the final path.
- In addition to the ASCII map, if a path is found, the nodes traveled will be returned.