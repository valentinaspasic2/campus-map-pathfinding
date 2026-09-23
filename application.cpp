#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue> // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include "json.hpp"
#include "dist.h"
#include "graph.h"

using namespace std;
using json = nlohmann::json;

double INF = numeric_limits<double>::max(); // largest value double can hold

//turn campus map data into a graph
void buildGraph(istream &input, graph<long long, double> &g,
                vector<BuildingInfo> &buildings,
                unordered_map<long long, Coordinates> &coords) {
  // load json
  json j;
  input >> j;

  // get main parts
  auto buildingsJSON = j["buildings"]; //buildings
  auto waypointsJSON = j["waypoints"]; //points along paths
  auto footwaysJSON = j["footways"]; //lists of waypoints

  // make a vertex for each building
  for (auto& b : buildingsJSON) {
    // read info from JSON
    long long id = b["id"];
    double lat = b["lat"];
    double lon = b["lon"];
    string name = b["name"];
    string abbr = b["abbr"];

    // store building info into buildings vector
    buildings.push_back(BuildingInfo(id, Coordinates{lat, lon}, name, abbr));

    // add vertex to graph
      g.addVertex(id);
    }

  // add all waypoints as vertices
  for (auto& w : waypointsJSON) {
    // read info from JSON
    long long id = w["id"];
    double lat = w["lat"];
    double lon = w["lon"];

    // store coordinates
    coords[id] = Coordinates{lat, lon};

    // add to graph
    g.addVertex(id);
  }

  // connect waypoints along each footway
  for (auto& footway : footwaysJSON) {
    //loop through connected waypoints
    for (int i = 0; i < footway.size() - 1; i++) {
      // create from -> to variables
      long long from = footway[i];
      long long to = footway[i + 1];

      // calculate distance once for both directions
      double dist = distBetween2Points(coords[from], coords[to]);

      // footways go both directions
      g.addEdge(from, to, dist);
      g.addEdge(to, from, dist);
    }
  }

  // connect buildings to nearby waypoints
  for (auto& building : buildings) {
    // loop through each waypoint and get its info
    for (auto& pair : coords) {
      long long waypointID = pair.first;
      Coordinates waypointCoord = pair.second;

      // calculate distance
      double dist = distBetween2Points(building.location, waypointCoord);

      // connect if close enough
      if (dist <= 0.036) {
        g.addEdge(building.id, waypointID, dist);
        g.addEdge(waypointID, building.id, dist);
      }
    }
  }
}

// look up building by name / abbreviation
BuildingInfo getBuildingInfo(const vector<BuildingInfo> &buildings,
                             const string &query) {

  for (const auto &building : buildings) {
    // if input abbreviation matches
    if (building.abbr == query) {
      return building;
    // if input is part of building full name
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  // no match, create new object fail, assign ID -1
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

// find closest building to given location
BuildingInfo getClosestBuilding(const vector<BuildingInfo> &buildings,
                                Coordinates c) {
  // hold smallest distance found so far
  double minDestDist = INF;
  // hold closest building found so far
  BuildingInfo closestBuilding = buildings.at(0);

  for (const BuildingInfo &building : buildings) {
    // calculate disctance between current building and the midpoint
    double dist = distBetween2Points(building.location, c);
    // update closest building and distance
    if (dist < minDestDist) {
      minDestDist = dist;
      closestBuilding = building;
    }
  }
  return closestBuilding;
}

// find shortest path through graph (dijkstra’s algorithm)
vector<long long> dijkstra(const graph<long long, double> &G, long long start,
                           long long target,
                           const set<long long> &ignoreNodes) {
  // if building already destination
  if (start == target) {
    return vector<long long>{start};
  }

  // choose smallest distance first
  class prioritize {
   public:
   // compare distances to know priority
    bool operator()(const pair<long long, double>& p1, const pair<long long, double>& p2) const {
      return p1.second > p2.second;
    }
  };

  // shortest distances
  unordered_map<long long, double> distances;

  // previous locations
  unordered_map<long long, long long> previous;

  // set all distances to infinity
  for (long long v : G.getVertices()) {
    distances[v] = INF;
  }

  distances[start] = 0;

  // create priority queue
  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize>
  worklist;

  // start with the starting node
  worklist.push({start, 0});

  // main search loop
  while (!worklist.empty()) {
    long long current = worklist.top().first;
    double currentDist = worklist.top().second;
    worklist.pop();

    // skip old duplicates
    if (currentDist > distances[current]) {
      continue;
    }

    // check every neighbor
    for (long long neighbor : G.neighbors(current)) {
      // skip ignored nodes, except target
      if (ignoreNodes.count(neighbor) && neighbor != target) {
        continue;
      }

      // get dist between current location & neighbor
      double weight;
      G.getWeight(current, neighbor, weight);

      double newDist = distances[current] + weight;

      // found a shorter path
      if (newDist < distances[neighbor]) {
        distances[neighbor] = newDist;
        previous[neighbor] = current;
        worklist.push({neighbor, newDist});
      }
    }
  }

  // no path found
  if (distances[target] == INF) {
    return vector<long long>{};
  }

  vector<long long> path;

  // rebuild path backwards
  long long current = target;
  while (current != start) {
    path.push_back(current);
    current = previous[current];
  }

  path.push_back(start);

  // put path in correct order
  reverse(path.begin(), path.end());

  return path;
}

// calculate total distance
double pathLength(const graph<long long, double> &G,
                  const vector<long long> &path) {
  double length = 0.0;
  double weight;
  // loop through consecutive locations
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

// formatted output
void outputPath(const vector<long long> &path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

// application() provided by the course instructor
void application(const vector<BuildingInfo> &buildings,
                 const graph<long long, double> &G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto &building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
