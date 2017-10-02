#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "path_finder.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}



#define LANE_0 (2.0f)
#define LANE_1 (6.0f)
#define LANE_2 (10.0f)
#define CTRL_VECTOR_SIZE (60) // Size of the vector holding the planned points
#define MIN_CAR_DIST (40.0f)
#define MAX_SPEED    (46.0f)
/* Grid parameters */


#define LANE_WIDTH (4.0f)
/* there will be 5 rows behind and 9 in front */
//#define set_vechile(x,y) (grid[x][y] = '.')



double current_lane = LANE_1;
double next_lane = LANE_1;
bool   change_lane= false;
double current_tgt_speed = 40.5;
double current_speed = 0.0; // controls the acceleration


// Load up map values for waypoint's x,y,s and d normalized normal vectors
vector<double> map_waypoints_x;
vector<double> map_waypoints_y;
vector<double> map_waypoints_s;
vector<double> map_waypoints_dx;
vector<double> map_waypoints_dy;


double calculateAcceleration(double current, double target)
{
  static double integral_term = 0;
  static double differen_term = 0;
  static double last_err = 0;

  static double P=0.019,I=0.00003,D=0.0019;

  double increment;
  double err = target - current;

  differen_term = err - last_err;
  last_err = err;
  integral_term += err;

  increment = (P * err) + (I * integral_term) + (D * differen_term);

 // cout << "inc: "  << increment <<  "  err: " << err  << " Veh speed: " << current << endl;


  return  increment;
}

/* Returns a vector of XY points as if the highway were clear to go */
tk::spline getNextPoints(double current_s, double current_d, double ref_yaw)
{
  /*Takes the current lane to do the calculation */
  vector<double> X, Y;

  auto ref_vals = getXY(current_s, current_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  for(int i=0; i < 100;)
  {
    i +=25;
    auto p = getXY(current_s+i, current_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    X.push_back(p[0]);
    Y.push_back(p[1]);
  }

  for (int j = 0; j < X.size(); ++j) {
    double transl_x = X[j] - ref_vals[0];
    double transl_y = X[j] - ref_vals[1];
    X[j] = transl_x * cos(0.0f-ref_yaw) - transl_y * sin(0.0f - ref_yaw);
    Y[j] = transl_x * sin(0.0f-ref_yaw) + transl_y * cos(0.0f - ref_yaw);

    cout << "px:" << X[j] << " py:" << Y[j] << endl;
  }

  tk::spline spl;
  spl.set_points(X,Y);


  return spl;
}




int main() {
  uWS::Hub h;

  path_finder* fp = new path_finder();
  unsigned int roll_count = 0;
  unsigned int period = 3; // process every 10 messages
  bool maneuver_done = true;
  /* Resize the grid to the desired size and fill */

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&maneuver_done,  &roll_count, &period, &fp, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;


    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;


          roll_count++;
          bool exists_path = false;

          fp->set_me((GRID_COLS-1)- int(car_d / LANE_WIDTH), FRONT_GRID);


          double pos_x, pos_y, yaw;
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          int current_path_size  = previous_path_x.size();

          /* Get possible collisions */

          vector<double > collide_nwse = {0,0,0,0,0};
          for (int k = 0; k < sensor_fusion.size(); ++k)
          {
            /* sensor fusion is an array of values [ id, x, y, vx, vy, s, d]. */

            /*For each car */
            double vehicle_d = sensor_fusion[k][6];
            double vehicle_s = sensor_fusion[k][5];
            double vx = sensor_fusion[k][3];
            double vy = sensor_fusion[k][4];

            unsigned int col = 0;
            unsigned int row = 0;

            col = (GRID_COLS-1) - int(vehicle_d / LANE_WIDTH);
            //row = FRONT_GRID - int((car_s - vehicle_s)/LANE_WIDTH);
            row = FRONT_GRID - int((car_s - vehicle_s)/(LANE_WIDTH));
            if((row < GRID_ROWS) && (col < GRID_COLS))
            {
              fp->set_vehicle(col, row);
            }

            /* adding the change in position due to speed, and substract my speed */
            vehicle_s += 0.02 * current_path_size * sqrt(vx*vx + vy*vy);
            double delta_s = vehicle_s - car_s;
            double delta_d = fabs(vehicle_d- car_d); // difference in lane
            float v_lane;

            if( (delta_s > 0 ) && delta_s < MIN_CAR_DIST)
            {
              /*The car is close*/
              if (delta_d < 3.0){
                collide_nwse[0] = delta_s;
              }

            }

          }


          /* Get possible collisions */
          //fp->set_goal((GRID_COLS-1)- int(car_d / LANE_WIDTH), 13);
          fp->set_goal(0, 14);
          fp->set_goal(1, 14);
          fp->set_goal(2, 14);
          //fp->set_goal((GRID_COLS-1)- int(car_d / LANE_WIDTH), 14);
          fp->show_grid();
          auto path = fp->find_path();
          exists_path = (path.size() != 0);
          fp->show_grid();
          fp->clean_grid();



          double pos_prev_x, pos_prev_y;
          vector<double> X, Y;
          if(!path.empty() && maneuver_done)
          {
            reverse(path.begin(), path.end());
            cout << "MANEUUUVERRR ----------------" << endl;
            switch (path[0])
            {
              default:
              case '|':
                current_tgt_speed = MAX_SPEED;
                next_lane = current_lane;
                maneuver_done = true;
                break;
              case '\\':
                if(fp->is_cell_free((GRID_COLS-1)- int(car_d / LANE_WIDTH), current_lane-4.0)) // there is not a car at the side
                {
                  if(current_lane > LANE_0)
                  {
//                    current_lane = current_lane - LANE_WIDTH/2.0;
                    cout << "Change to lane: " << current_lane << endl;
                    next_lane = current_lane - LANE_WIDTH;
                    current_tgt_speed = 40.0;
                    maneuver_done = false;
                  }
                } else{
                  cout << "NO PATH: " << current_lane << endl;
                  exists_path = false;
                }
                break;
              case '/':
                if(fp->is_cell_free((GRID_COLS-1)- int(car_d / LANE_WIDTH), current_lane+4.0)) {
                  if (current_lane < LANE_2) {
                    next_lane = current_lane + LANE_WIDTH;
//                    current_lane = current_lane + LANE_WIDTH/2.0;
                    cout << "Change to lane: " << current_lane << endl;
                    current_tgt_speed = 40.0;
                    maneuver_done = false;
                  }
                } else{
                  cout << "xxNO PATH: " << current_lane << endl;
                  exists_path = false;
                }
                break;
            }
          }
          cout << "maneuver done:" << maneuver_done << " car_d-current_lane" << car_d-current_lane << endl;
          /*Keep inside a lane */
          if((fabs(car_d-next_lane) < 0.5f) /*&& ((roll_count%period) == 0)*/ )
          {
            maneuver_done = true;
//            current_lane += (next_lane-current_lane)*0.3;
            cout << "NextLane "  << next_lane<<  endl;
          } else{
            current_lane += (next_lane-current_lane)*0.23;
          }

          current_speed += calculateAcceleration(car_speed, current_tgt_speed);
          if((collide_nwse[0] != 0))
          {

            if(!exists_path || (0.0 < collide_nwse[0]) && ((25.0)> collide_nwse[0])){

              current_tgt_speed = 10.0 + 7.0*(collide_nwse[0]/MIN_CAR_DIST);

              if(current_tgt_speed < 10)
              {
                current_tgt_speed = 10.0;
              }
              cout << "BREAKING!!!!" << endl;
            } else{
              current_tgt_speed = 38; //reduce speed
              cout << "POSSIBLE FRONT COLLISION " << collide_nwse[0] << "\nNew Tgt speed: " << current_tgt_speed << endl;
            }
          }
          else{
            current_tgt_speed = MAX_SPEED;
          }

          for(int i = 0; i < current_path_size; ++i)
          {
            //cout << previous_path_x[i] <<",";
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          /* get two points from the last position to generate a new trajectory */
          if(current_path_size > 1)
          {
            /* Copy the points from the last trajectory to ensure smooth change */
            pos_x = previous_path_x[current_path_size-1];
            pos_y = previous_path_y[current_path_size-1];
            pos_prev_x = previous_path_x[current_path_size-2];
            pos_prev_y = previous_path_y[current_path_size-2];

            yaw = (atan2(pos_y-pos_prev_y,pos_x-pos_prev_x));
            X.push_back(pos_prev_x); Y.push_back(pos_prev_y);

          } else{
            /* Fresh start */
            pos_x = car_x;
            pos_y = car_y;
            yaw = deg2rad(car_yaw);

          }
//          dist_inc = calculateAcceleration(car_speed/100, current_tgt_speed);//

          /**********************************************************************/
          /*Takes the current lane to do the calculation */

          /* The last points to ensure continuity */
//          X.push_back(pos_prev_x); Y.push_back(pos_prev_y);
          X.push_back(pos_x); Y.push_back(pos_y);
          for(int i=0; i < 160;)
          {
            i +=40;
            auto p = getXY(car_s+i, current_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            X.push_back(p[0]);
            Y.push_back(p[1]);
            //cout << "spx:" << p[0] << " spy:" << p[1] << endl;
          }
//          cout << "--------------\npos_x:" << pos_x << " pos_y:" << pos_y << " yaw: " << yaw << endl << endl;
          for (int j = 0; j < X.size(); ++j)
          {
            double transl_x = X[j] - pos_x;
            double transl_y = Y[j] - pos_y;
            double nx,ny;
            nx = transl_x * cos(0.0f-yaw) - transl_y * sin(0.0f - yaw);
            ny = transl_x * sin(0.0f-yaw) + transl_y * cos(0.0f - yaw);
            X[j] = nx;
            Y[j] = ny;
            //cout << "px:" << X[j] << " py:" << Y[j] << " transl_x:" << transl_x << " transly: " << transl_y << " sin: " <<  sin(yaw) << " cos: " << cos(yaw) << endl;
          }

          tk::spline spl;
          spl.set_points(X,Y);
          /***************************************************************/

          cout << "Desired car speed: " << current_speed << endl;
          cout << "Real car speed: " << car_speed << endl;
            /* Generate a smooth trajectory between the current position and the next waypoint */

          //  auto spline = getNextPoints(car_s-1,car_d, yaw);
            //auto cords = getXY(car_s-(1),car_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          double increment = 0;
          double dist_x = 20;
          double dist_y = spl(dist_x);
          double mag_dist = distance(0, 0,dist_x, dist_y);
//          cout << "distx: " << dist_x << " dist y:" << dist_y << " magxy: " << mag_dist << endl;

            /* the spline rotated the coords so we need to rotate them back */


            for(int i = 0; i < CTRL_VECTOR_SIZE-previous_path_x.size(); i++)
            {
//							next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//              next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
//              cout << "." ;
              #if test
              cout << "." ;
							auto cords = getXY(car_s+(dist_inc*i),current_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
              next_x_vals.push_back(cords[0]);
              next_y_vals.push_back(cords[1]);
              #endif
              /* we must take into account the curvature or the speed goes out of bound */
              double N =  (mag_dist/ (0.02f * current_speed / 2.24f));
              double x = increment+(dist_x/N);
              double y = spl(x);
              double x_tmp=0, y_tmp=0;
              increment = x;
              x_tmp = x;
              y_tmp = y;

              x = (x_tmp * cos(yaw) - y_tmp * sin(yaw));
              y = (x_tmp * sin(yaw) + y_tmp * cos(yaw));

              x += pos_x; y += pos_y;

							next_x_vals.push_back(x);
							next_y_vals.push_back(y);
            }
						cout << "car_d:" << car_d << " car_s: " << car_s << endl;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";
          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
