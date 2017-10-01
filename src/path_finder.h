//
// Created by porko on 9/30/17.
//

#ifndef PATH_PLANNING_PATH_FINDER_H
#define PATH_PLANNING_PATH_FINDER_H

#include <iostream>
#include <vector>
#include <queue>

#define GRID_COLS (3)
#define FRONT_GRID (2)
#define REAR_GRID (13)
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
  std::vector<char> _find_path(node* root);
  bool found;
  bool finished;
  int me_x;
  int me_y;
  std::vector<std::vector<node>> node_map;
public:
  void set_vehicle(int x, int y);
  void set_me(int x, int y);
  void set_goal(int x, int y);
  path_finder(): found(false), finished(false), me_x(0),me_y(0)
  {
    /*Create the grid*/
    node_map.resize(GRID_ROWS);
    for (int i = 0; i < GRID_ROWS; ++i) {
        node_map[i].resize(GRID_COLS);
    }
    std::cout << "Map size: " << node_map.size() << "x" << node_map[0].size() << std::endl;
  }

  void clean_grid(void);
  void show_grid(void);
  bool is_cell_free(int x, int y);
  std::vector<char> find_path();
};


#endif //PATH_PLANNING_PATH_FINDER_H
