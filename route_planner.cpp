#include "route_planner.h"
#include "route_model.h"
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <vector>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to
  // the starting and ending coordinates. Store the nodes you find in the
  // RoutePlanner's start_node and end_node attributes.
  RouteModel::Node *start_node = &m_Model.FindClosestNode(start_x, start_y);
  RouteModel::Node *end_node = &m_Model.FindClosestNode(end_x, end_y);
  RoutePlanner::start_node = start_node;
  RoutePlanner::end_node = end_node;

}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another
// node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  // node -> distance is to call methods on a pointer object
  //
  float result = node->distance(*end_node);
  return result;
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding
// all unvisited neighbors to the open list. Tips:
// - Use the FindNeighbors() method of the current_node to populate
// current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the
// g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and
// set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

  current_node->FindNeighbors();
  // std::cout << "are we even here? \n";
  if (current_node->neighbors.empty()) {
      // std::cout << "No neighbors found for node at (" << current_node->x << ", " << current_node->y << ")\n";
  }
  for (RouteModel::Node *neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value =
        current_node->g_value + current_node->distance(*neighbor);
    neighbor->visited = true;
    open_list.push_back(neighbor);
  }
}

// TODO 5: Complete the NextNode method to sort the open list and return the
// next node. Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

  auto &list = open_list;

  int length = list.size();
  for (int i = 0; i < length - 1; i++) {
    RouteModel::Node *node = list[i];
    RouteModel::Node *next_node = list[i + 1];
    float sum = node->h_value + node->g_value;
    float nsum = next_node->h_value + next_node->g_value;
    if (sum < nsum) {
      RouteModel::Node *temp = list[i];
      list[i] = list[i + 1];
      list[i + 1] = temp;
      i = -1;
    }
  }
  RouteModel::Node *abc = list.back();
  list.pop_back();
  return abc;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found
// from your A* search. Tips:
// - This method should take the current (final) node as an argument and
// iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to
// the distance variable.
// - The returned vector should be in the correct order: the start node should
// be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  // TODO: Implement your solution here.

  auto parent = current_node->parent;
  auto child = current_node;
  while (parent != nullptr) {
    distance += parent->distance(*child);
    path_found.insert(path_found.begin(), *child);
    child = parent;
    parent = child->parent;
  }
  // handle start_node
  path_found.insert(path_found.begin(), *child);

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node
// to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method
// to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits.
// This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
// Check for valid start and end nodes
    if (!start_node || !end_node) {
        std::cout << "Invalid start_node or end_node\n";
        return;
    }
  RouteModel::Node *current_node = nullptr;

  // TODO: Implement your solution here.
  current_node = start_node;

  current_node->h_value = CalculateHValue(current_node);
  current_node->visited = true;
  open_list.push_back(current_node);

  while (open_list.size() > 0) {
    current_node = NextNode();
    // std::cout << current_node->x << "  " << current_node->y << "\n";
    // std::cout << end_node->x << "  " << end_node->y << "\n";
    // std::cout << open_list.size() << "\n";
    if (current_node == end_node) {
      m_Model.path = ConstructFinalPath(current_node);
      std::cout << "success!! \n";
      return;
    }
    AddNeighbors(current_node);

    std::cout << open_list.size() << "\n";
  }

  // if no path found return empty path
  m_Model.path = std::vector<RouteModel::Node>{};
}
