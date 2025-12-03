#include <iostream>
#include <cmath>
#include <queue>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/graph_search.h>

#include <queue>
#include <algorithm>
#include <cmath>

/**
 * General graph search instructions:
 *
 * First, define the correct data type to keep track of your visited cells
 * and add the start cell to it. If you need to initialize any properties
 * of the start cell, do that too.
 *
 * Next, implement the graph search function. Save the result in the path
 * variable defined for you.
 *
 * To visualize which cells are visited in the navigation webapp, save each
 * visited cell in the vector in the graph struct as follows:
 *      graph.visited_cells.push_back(c);
 * where c is a Cell struct corresponding to the visited cell you want to
 * visualize.
 *
 * The tracePath() function will return a path (which you should assign to
 * the path variable above) given the goal index, if you have kept track
 * of the parent of each node correctly and have implemented the
 * getParent() function. If you do not find a path, return an empty path
 * vector.
*/

std::vector<Cell> depthFirstSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /**
     * TODO (P3): Implement DFS.
     */

    return path;
}

std::vector<Cell> breadthFirstSearch(GridGraph& graph,
                                     const Cell& start,
                                     const Cell& goal)
{
    std::vector<Cell> path;

    // Reset node data
    initGraph(graph);

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx  = cellToIdx(goal.i,  goal.j,  graph);

    std::queue<int> q;

    // Initialize start node
    graph.visited[start_idx]  = true;
    graph.distance[start_idx] = 0;
    graph.parent[start_idx]   = -1;

    q.push(start_idx);

    // For visualization: record the start
    graph.visited_cells.push_back(start);

    while(!q.empty())
    {
        int current = q.front();
        q.pop();

        // Check for goal
        if(current == goal_idx)
            break;

        // Visualization: mark which cell we’re expanding
        Cell c = idxToCell(current, graph);
        graph.visited_cells.push_back(c);

        // Expand neighbors
        std::vector<int> neighbors = findNeighbors(current, graph);
        for(int n_idx : neighbors)
        {
            // Already visited?
            if(graph.visited[n_idx])
                continue;

            // In collision?
            if(checkCollision(n_idx, graph))
                continue;

            graph.visited[n_idx]  = true;
            graph.parent[n_idx]   = current;
            graph.distance[n_idx] = graph.distance[current] + 1;

            q.push(n_idx);
        }
    }

    // If goal never reached, return empty path
    if(!graph.visited[goal_idx])
        return path;

    // Reconstruct path using helper
    path = tracePath(goal_idx, graph);
    return path;
}

std::vector<Cell> aStarSearch(GridGraph& graph, const Cell& start, const Cell& goal)
{
    std::vector<Cell> path;  // The final path should be placed here.

    initGraph(graph);  // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /**
     * TODO (P3): Implement A-star search.
     */

    return path;
}
