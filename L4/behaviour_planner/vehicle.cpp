#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a)
{

  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  state = "CS";
  max_acceleration = -1;

}

Vehicle::~Vehicle()
{}
/* The cost functions return a value between 0-1 */
// this function is for the case when we need to go from the goal lane
// to far from it to rebase an obstacle
#define COMFORT_COST   (1000.0f)
#define GOAL_COST      (100000.0f)
#define COLLISION_COST (1000000.0f)
#define SPPED_COST     (100.0f)
double_t lane_error(int proposed, int goal)
{
  /* This function returns a low value for chaning lane towards the goal lane and high for changing away from it */
  double_t ret_val = 0;
  if(proposed == goal)
  {
    ret_val = -COMFORT_COST;
  }
  else if(proposed != goal)
  {
    ret_val = COMFORT_COST;
  }
  return ret_val;
}

double_t goal_distance(int proposed, int goal_lane, int goal_s, int v_s, int v_lane)
{
  double_t cost = 0;
  int rem_distance;
  int rem_lanes;
  int toa;
  /* get the remainig distance to the goal s */
  rem_distance = goal_s - v_s;
  /* get the lane changes needed to reach the goal lane */
  rem_lanes = fabs(goal_lane-v_lane);
  toa = rem_distance / 10.0f;
  cost =  rem_lanes / float(toa);

  return cost;
}
double_t speed_error(int current_v, int goal_v)
{
  return 0.5;
}
/* Given the sate of the car check if the new state will collide or has a higher chance of colliding */
double_t collision_avoid(vector<int> state, map<int, vector<vector<int> > > predictions )
{

  return 0.5;
}
double_t efficiency_cost(int k, int j){
  return 0.5;
}

typedef double_t (*pfnc)(int, int);
#define NUM_OF_COSTS  ((sizeof(costFunctions))/(sizeof(costFunctions[0])))



// TODO - Implement this method.
void Vehicle::update_state(map<int, vector<vector<int> > > predictions)
{
  map<string, int> possible_states = {{"KL", 0}, {"LCL",1},
                                      {"LCR",2}, {"PLCL",3}, {"PLCR", 4}};

  /*
   * The state of this vehicle can be obtained with this-> s,v,lane,a.
   * The goal velocity and goal lane with this->goal_s and goal_v
   *
   * Define cost functions for:
   *
   * - Plausible lane change
   * - Error in velocity expected vs current
   * - Error in lane expected vs current
   * - plausible collision
   * -
   *
   * */
  /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
  cout<<  display();

  /* Discard unplausible states given the current state: */
  if(this->lane == 0)
  {
    possible_states.erase("LCL");
    possible_states.erase("PLCL");
  } else if (this->lane == (this->lanes_available-1))
  {
    possible_states.erase("LCR");
    possible_states.erase("PLCR");
  }
  /* Store the cost we fall into for each of the possible actions and select the minimum */


  map<string,  vector<double_t>> costs; /* create an array of costst */
  vector<int> state_bkp(4);
  vector<int> state_curr(4); // vector to pass
  string car_state_bkp;
  for(auto it = possible_states.begin(); it!=possible_states.end(); ++it)
  {
    /* From the possible actions to take (KL, LCR, LCL,PLCL,PLCR) calculate trajectory and cost */

    /* Calculate trajectory for each possible action action */
    /* Backup current state */
    state_bkp[0] = this->lane;
    state_bkp[1] = this->s;
    state_bkp[2] = this->v;
    state_bkp[3] = this->a;
    state_curr[0] = this->lane;
    state_curr[1] = this->s;
    state_curr[2] = this->v;
    state_curr[3] = this->a;
    car_state_bkp = this->state;

    /* Take current action from the list */
    this->state = it->first;
    this->realize_state(predictions);
    this->increment(1);

    for (auto it= predictions.begin(); it != predictions.end(); ++it)
    {
      //cout << it->first << "\n";
    }


    costs[it->first].push_back(lane_error(this->lane, this->goal_lane));
    costs[it->first].push_back(collision_avoid(state_curr,predictions)); // this functions does not need any parameter
    costs[it->first].push_back(speed_error(this->v, this->target_speed));

    /* Find the action with the minumum cost */


    /* Restore the last state */

    this->state = car_state_bkp;
    this->lane  = state_bkp[0];
    this->s     = state_bkp[1];
    this->v     = state_bkp[2];
    this->a     = state_bkp[3];
  }

  /* Get the action with the minimum cost associated */
  state = "KL"; // this is an example of how you change state.


}

void Vehicle::configure(vector<int> road_data)
{
  /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
  target_speed = road_data[0];
  lanes_available = road_data[1];
  max_acceleration = road_data[2];
  goal_lane = road_data[3];
  goal_s = road_data[4];
}

string Vehicle::display()
{
  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";
  oss << "----------" << "\n";
  oss << "goal lane: " << this->goal_lane << "\n";
  oss << "goal v:    " << this->target_speed << "\n";

  return oss.str();
}

void Vehicle::increment(int dt = 1)
{
  this->s += this->v * dt;
  this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t)
{
  /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
  int s = this->s + this->v * t + this->a * t * t / 2;
  int v = this->v + this->a * t;
  return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time)
{

  /*
    Simple collision detection.
  */
  vector<int> check1 = state_at(at_time);
  vector<int> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps)
{

  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps + 1; t++) {
    if (collides_with(other, t)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(map<int, vector<vector<int> > > predictions)
{

  /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
  string state = this->state;
  if (state.compare("CS") == 0) {
    realize_constant_speed();
  } else if (state.compare("KL") == 0) {
    realize_keep_lane(predictions);
  } else if (state.compare("LCL") == 0) {
    realize_lane_change(predictions, "L");
  } else if (state.compare("LCR") == 0) {
    realize_lane_change(predictions, "R");
  } else if (state.compare("PLCL") == 0) {
    realize_prep_lane_change(predictions, "L");
  } else if (state.compare("PLCR") == 0) {
    realize_prep_lane_change(predictions, "R");
  }

}

void Vehicle::realize_constant_speed()
{
  a = 0;
}

int Vehicle::_max_accel_for_lane(map<int, vector<vector<int> > > predictions, int lane, int s)
{

  int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > in_front;
  while (it != predictions.end()) {

    int v_id = it->first;

    vector<vector<int> > v = it->second;

    if ((v[0][0] == lane) && (v[0][1] > s)) {
      in_front.push_back(v);

    }
    it++;
  }

  if (in_front.size() > 0) {
    int min_s = 1000;
    vector<vector<int>> leading = {};
    for (int i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0][1] - s) < min_s) {
        min_s = (in_front[i][0][1] - s);
        leading = in_front[i];
      }
    }

    int next_pos = leading[1][1];
    int my_next = s + this->v;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }

  return max_acc;

}

void Vehicle::realize_keep_lane(map<int, vector<vector<int> > > predictions)
{
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector<vector<int> > > predictions, string direction)
{
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<int> > > predictions, string direction)
{
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  int lane = this->lane + delta;

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > at_behind;
  while (it != predictions.end()) {
    int v_id = it->first;
    vector<vector<int> > v = it->second;

    if ((v[0][0] == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);

    }
    it++;
  }
  if (at_behind.size() > 0) {

    int max_s = -1000;
    vector<vector<int> > nearest_behind = {};
    for (int i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }
    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    int delta_v = this->v - target_vel;
    int delta_s = this->s - nearest_behind[0][1];
    if (delta_v != 0) {

      int time = -2 * delta_s / delta_v;
      int a;
      if (time == 0) {
        a = this->a;
      } else {
        a = delta_v / time;
      }
      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }
      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }
      this->a = a;
    } else {
      int my_min_acc = max(-this->max_acceleration, -delta_s);
      this->a = my_min_acc;
    }

  }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10)
{

  vector<vector<int> > predictions;
  for (int i = 0; i < horizon; i++) {
    vector<int> check1 = state_at(i);
    vector<int> lane_s = {check1[0], check1[1]};
    predictions.push_back(lane_s);
  }
  return predictions;

}