#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use of the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    //The nodes found are stored in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));

}


// Implementation of the CalculateHValue method.
// - h value: the distance to the end_node.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);

}


// The AddNeighbors method expands the current node by adding all unvisited neighbors to the open list.
// The FindNeighbors() method of the current_node populates current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, the parent, the h_value, the g_value are set.
// - For each node in current_node.neighbors, the neighbor is added to open_list and the node's visited attribute is set to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (RouteModel::Node* node: current_node->neighbors){
        if (node->visited == false){
            node->parent = current_node;
            node->h_value = RoutePlanner::CalculateHValue(node);
            node->g_value = current_node->g_value + node->distance(*current_node);
            open_list.emplace_back(node);
            node->visited = true;
        }
    }
}


// The NextNode method sorts the open list and returns the next node.
// - The open_list is sorted according to the sum of the h value and g value.
// - A pointer to the node in the list with the lowest sum is created.
// - That node is removed from the open_list.
// - The pointer is returned.


RouteModel::Node *RoutePlanner::NextNode() {
    std::vector<RouteModel::Node*>* p_open_list;
    p_open_list = &open_list;

    sort(p_open_list->begin(), p_open_list->end(),
    [](const RouteModel::Node* n1, const RouteModel::Node* n2)
    {return (n1->h_value + n1->g_value) > (n2->h_value + n2->g_value);});

    RouteModel::Node * lowest_node = open_list.back();
    open_list.pop_back();

    return lowest_node;
}


// The ConstructFinalPath method returns the final path found from the A* search.
// - This method takes the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, the distance from the node to its parent is added to the distance variable.
// - The returned vector has the correct order: the start node is the first element
//   of the vector, the end node is the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while(current_node != start_node){
    path_found.emplace_back(*current_node);
    distance+= current_node->distance(*(current_node->parent));
    current_node = current_node->parent;

  }
  // the while loop exists when current node is start node
  // so the start node is not added to the path_found_reversed list
  // we added below

  path_found.emplace_back(*current_node);
  reverse(path_found.begin(), path_found.end()); //from the algorithm class

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

  return path_found;


}


// The A* Search algorithm.
// - The AddNeighbors method adds all of the neighbors of the current node to the open_list.
// - The NextNode() method sorts the open_list and returns the next node.
// - When the search has reached the end_node, the ConstructFinalPath method is used to return the final path that was found.
// - The final path is stored in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    current_node = start_node;
    current_node->g_value = 0;
    current_node->h_value = current_node->distance(*end_node);
    current_node->visited = true;
    open_list.emplace_back(current_node);

    while (current_node != end_node){
    current_node = RoutePlanner::NextNode();
    RoutePlanner::AddNeighbors(current_node);

  }

    std::vector<RouteModel::Node> final_path;
    m_Model.path = RoutePlanner::ConstructFinalPath(current_node);

}
