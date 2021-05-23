#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto &node : current_node->neighbors) {
    node->parent = current_node;
    node->h_value = CalculateHValue(node);
    node->g_value = current_node->g_value + current_node->distance(*node);
    node->visited = true;
    open_list.push_back(node);
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(open_list.begin(), open_list.end(),
            [](const RouteModel::Node *a, const RouteModel::Node *b) {
              return a->g_value + a->h_value > b->g_value + b->h_value;
            });
  auto least = open_list.back();
  open_list.pop_back();
  return least;
}

std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  path_found.push_back(*current_node);
  while (current_node != start_node) {
    distance += current_node->distance(*current_node->parent);
    current_node = current_node->parent;
    path_found.emplace_back(*current_node);
  }

  std::reverse(path_found.begin(), path_found.end());
  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

  current_node = start_node;
  current_node->visited = true;
  open_list.push_back(current_node);

  while (!open_list.empty()) {
    current_node = NextNode();
    if (current_node == end_node) {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
    AddNeighbors(current_node);
  }
  std::cout << "No path found from " << start_node->x << "(," << start_node->y
            << ") to (" << end_node->x << "," << end_node->y << ")"
            << std::endl;
}