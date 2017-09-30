//
// Created by porko on 9/30/17.
//

#ifndef PATH_PLANNING_PATH_FINDER_H
#define PATH_PLANNING_PATH_FINDER_H

#include <iostream>
#include <vector>
#include <queue>

#define GRID_COLS (3)
#define FRONT_GRID (5)
#define REAR_GRID (10)
#define GRID_ROWS (FRONT_GRID + REAR_GRID ) /* Number of lane lines */

enum lane_id
{
  rigthmost_lane = 0,
  center_lane,
  leftmost_lane
};

class path_finder
{
  class node
  {
  public:
    // bool visited; not needed as it a tree
    // node* last; this will be implemented in the parent
    std::vector<node *> child_expand;
    std::vector<node *> parent_expand;
    int x;
    int y;
    char val;
    bool expanded;

    node() : child_expand {nullptr, nullptr, nullptr}, parent_expand {nullptr, nullptr, nullptr}, x(0), y(0), val('#'),
             expanded(false)
    {
      /* Empty */
    }

    node *get_parent(enum lane_id lane);

    node *get_child(enum lane_id lane);

    void set_parent(enum lane_id lane, node *n);

    void set_child(enum lane_id lane, node *n);

  };

  void set_val(std::vector<std::vector<node>>& v, size_t x, size_t y, char c);
  node* check_parent(node* n, std::vector<char>& sol);
  bool found;
  bool finished;
  std::vector<std::vector<node>> node_map;
public:

  path_finder(std::vector<std::vector<node>> m): found(false), finished(false), node_map(m)
  {

  }
  std::vector<char> solution;
  std::vector<char> find_path(node* root);
};


#endif //PATH_PLANNING_PATH_FINDER_H
