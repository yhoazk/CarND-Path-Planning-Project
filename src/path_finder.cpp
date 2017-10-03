//
// Created by porko on 9/30/17.
//

#include "path_finder.h"



path_finder::node* path_finder::node::get_parent(enum lane_id lane)
{
  return this->parent_expand[lane];
}

path_finder::node* path_finder::node::get_child(enum lane_id lane)
{
  return this->child_expand[lane];
}


void path_finder::node::set_parent(enum lane_id lane, node* n)
{
  this->parent_expand[lane] = n;
}

void path_finder::node::set_child(enum lane_id lane, node* n)
{
  this->child_expand[lane] = n;
}


void path_finder::set_val(std::vector<std::vector<node>>& v, size_t x, size_t y, char c)
{
  if(x < GRID_COLS && y < GRID_ROWS)
  {
    v[y][x].val = c;
    v[y][x].x = x;
    v[y][x].y = y;

  }
}

path_finder::node* path_finder::check_parent(path_finder::node* n, std::vector<char>& sol)
{
  char decoder[] = {'/','|','\\'};
  static const std::vector<size_t> prio = {1,2,0}; // first keep lane, second left, tird right
  //vector<node*> result = {nullptr, nullptr, nullptr};
  path_finder::node* result;// = {nullptr, nullptr, nullptr};
  int refs = 0; //number of encountered references
  size_t i = 0;

  for(auto k: prio)
  {
    if(nullptr != n->parent_expand[k])
    {
//      std::cout << decoder[k];
      sol.push_back(decoder[k]);
      result = n->parent_expand[k];
      result->val = decoder[k];
      break;
    }
  }

  //print_grid(node_map);
  return result;
}



std::vector<char> path_finder::_find_path(node* root)
{
  found = false;
  finished = false;
  std::queue<node*> node_queue;
  node* current_node;
  node* next_node;
  node* goal_node; // if found
  node_queue.push(root);
  int inc_x;
  static std::vector<char> path;
  std::vector<int> possible_movs;
  path.clear();
  while (node_queue.empty() != true && found == false && finished != true)
  {
    possible_movs = {center_lane, leftmost_lane,  rigthmost_lane};
    current_node = node_queue.front();
    node_queue.pop();
    if(current_node->expanded == true)
    {
      continue;
    }
//    std::cout << "______________________________________ Queue size: " << node_queue.size() << std::endl;
//    std::cout << "current_node x: " << current_node->x << "  y: " << current_node->y << " val: " << current_node->val << '\n';
//    for(size_t i =0; i < current_node->x; ++i)
//      std::cout <<  "   ";
    /* check possible paths in which the node can move */

    /**/
    if(rigthmost_lane == current_node->x)
    {
      possible_movs.erase( possible_movs.begin() +  2);
    }
    else if(leftmost_lane == current_node->x)
    {
      possible_movs.erase( possible_movs.begin() + 1 );
    }

    /*By reversing the elements the move to the left comes first in prio*/
    //reverse(possible_movs.begin(), possible_movs.end());
    char decode_show[] = {'/','|','\\'};
    for(auto lane: possible_movs)
    {
      switch (lane)
      {
        case rigthmost_lane:
          inc_x = -1;
          break;

        case leftmost_lane:
          inc_x = 1;
          break;

        case center_lane:
        default:
          inc_x = 0;
      }
      /* Check that the next node is in limits */
      next_node = &node_map[ (current_node->y)+1 ][ (current_node->x)+inc_x ];
      if(current_node->x < GRID_COLS && current_node->y < GRID_ROWS) {
        next_node->x = (current_node->x)+inc_x;
        next_node->y = (current_node->y)+1;
      } else {
        finished = true; //reached the end and not able to find the goal, keep the same movement
        continue;
      }


      /*check that we are not passing in a corner */
      /* The nodes at the the side of the turn must be empty  */
      if(next_node->x < GRID_COLS && next_node->y < GRID_ROWS){
        if(node_map[ (current_node->y) ][ (current_node->x)+inc_x ].val == '.')
        {
          continue;
        }

        if(node_map[ (current_node->y-1) ][ (current_node->x)+inc_x ].val == '.')
        {
          continue;
        }

        if(node_map[ (current_node->y+1) ][ (current_node->x) ].val == '.')
        {
          continue;
        }
      }

      /* Fill the child info */
      if('#' == next_node->val)
      {
        // std::cout << "next_node x: " << next_node->x << "  y: " << next_node->y << '\n';
//        std::cout << decode_show[lane];
        current_node->set_child(lane_id(lane), next_node);
        node_queue.push(next_node);
        next_node->set_parent(lane_id(lane), current_node);
      }

      else if('G' == next_node->val)
      {
//        std::cout << decode_show[lane];
        current_node->set_child(lane_id(lane), next_node);
        next_node->set_parent(lane_id(lane), current_node);
        std::cout << "\nFOUND";
        goal_node = next_node;
        std::cout << " Coordx: " << goal_node->x << " Coordy: " << goal_node->y   << std::endl;
        found = true;
        /* Now that we now exists a path to the goal, find the best one           */
        /* Check which of the possible parent have a refernce to me (the child node) */
        /* If that reference exists, then is part of the path, not otherwise         */
        while(goal_node != root)
        {
          goal_node = check_parent(goal_node, path);
        }

      }

    }
    current_node->expanded = true;
//    std::cout << std::endl;
  }

  if(!found)
  {
    /* There's no possible path to the to the goal*/
    std::cout << "NO PATH!!!!!!!!!!1" << std::endl;
    return {};
  }
  else
  {
    /* Check if the path is overly complex ie there's a lot of traffic*/
    int k = 0;
    for (auto m: path)
    {
      std::cout << m;
      if(m != '|')
      {
        ++k;
      }
    }
    std::cout << std::endl;
    if(k > 5)
    {
      return {};
    }
    else
    {
      return path;
    }
  }

}


std::vector<char> path_finder::find_path(void){
  return _find_path(&node_map[me_y][me_x]);
}

void path_finder::clean_grid(void)
{
  for (int i = 0; i < GRID_ROWS; ++i) {
    for (int j = 0; j < GRID_COLS; ++j) {
//      node_map[i][j].val = '#';      node_map[i][j].expanded = false;
      (&node_map[i][j])->~node();
      new(&node_map[i][j]) node();
    }
  }
}
void path_finder::show_grid(void)
{
  std::cout << "   0 1 2\n";
  for (int i = 0; i < GRID_ROWS; ++i) {
    std::cout.width(2);
    std::cout << i << '|' << ' ';
    for (int j = 0; j < GRID_COLS; ++j) {
      std::cout << node_map[i][j].val <<' ';
    }
    std::cout << std::endl;
  }

}

void path_finder::set_vehicle(int x, int y)
{
  if(x < GRID_COLS && y < GRID_ROWS)
  {
    node_map[y][x].val = '.';
    node_map[y][x].x = x;
    node_map[y][x].y = y;
  }
}
void path_finder::set_me(int x, int y)
{
  if(x < GRID_COLS && y < GRID_ROWS)
  {
    me_x = x;
    me_y = y;
    node_map[y][x].val = 'O';
    node_map[y][x].x = x;
    node_map[y][x].y = y;
  }
}

void path_finder::set_goal(int x, int y)
{
  if(x < GRID_COLS && y < GRID_ROWS)
  {
    node_map[y][x].val = 'G';
  }
}

bool path_finder::is_lane_free(int x, int y)
{
  bool state = false;
  if(x < 0 || x> 2 || y < 0){
    /*Outer lanes*/
   state = true;
  }
  else{

    if(x < GRID_COLS && y < GRID_ROWS)
    {
      if(node_map[y][x].val == '#'
        && node_map[y-1][x].val == '#'
        && node_map[y-2][x].val == '#'
        && node_map[y+1][x].val == '#'
       && node_map[y+2][x].val == '#'
          )
      {
        state = true;
      }
    }
  }
  return state;
}
