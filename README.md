# Campus Map Pathfinding
C++ campus map application that finds a meeting building and shortest walking paths for two people.

## How it works
The program loads campus buildings and walking paths from map data and builds a weighted graph to represent their connections.

The program selects a meeting building near the midpoint between two people, then uses Dijkstra’s algorithm to find the shortest walking path for each person to reach it.
