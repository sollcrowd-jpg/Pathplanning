# Path Planning Project

## Overview
This is a C++ path planning application that implements graph search algorithms for robot navigation. The project includes implementations of:
- Depth-First Search (DFS)
- Breadth-First Search (BFS)  
- A* Search

The program loads grid maps and finds paths from a start position to a goal position while avoiding obstacles.

## Recent Changes
- **December 3, 2025**: Fixed compilation issues
  - Added missing `#include <array>` header for std::array support
  - Removed duplicate `visited_cells` field declaration in GridGraph struct
  - Successfully built the project

## Project Architecture
- **Language**: C++14
- **Build System**: CMake
- **Directory Structure**:
  - `src/`: Source code files
    - `path_planner_cli.cpp`: Main CLI application
    - `graph_search/`: Search algorithm implementations
    - `utils/`: Utility functions for graph operations
  - `include/path_planning/`: Header files
  - `data/`: Map files (.map format with corresponding .png visualizations)
  - `build/`: Build output directory

## Usage
The path planner can be run interactively or with command-line arguments:

**Interactive mode** (via workflow):
```
./nav_cli
```
Then provide:
- Path to map file (e.g., `../data/maze1.map`)
- Start cell coordinates (i, j)
- Goal cell coordinates (i, j)  
- Algorithm choice (dfs, bfs, astar)

**Command-line mode**:
```
./nav_cli [map_file] [algorithm] [start_i] [start_j] [goal_i] [goal_j]
```

## Available Maps
- `empty_map.map`: Empty environment
- `maze1.map` through `maze4.map`: Various maze configurations
- `narrow.map`: Narrow corridor test
- `one_obstacle.map`, `two_obstacles.map`: Simple obstacle scenarios
- `tiny_map.map`: Small test map

## Build Instructions
The project uses CMake and is configured to build for laptop (not the MBot robot):
```
mkdir -p build
cd build
cmake ..
make
```

The executable `nav_cli` will be created in the build directory.
