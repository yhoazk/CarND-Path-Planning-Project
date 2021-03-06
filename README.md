# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.



A controller its needed to keep the car within the acceleration and lanes without intervention.
Then the intervention must be done only when needed, for example in the case of a possible collision.
Or in the case that the vehicle is slowing down due to a vehicle in front.

Then some parameters have to be controlled easily:
* Current lane
* Target speed
* Min distance to other cars
*

The controller must take the past points and then create a new trajectory from those points, this reduces the
jerk due to non-smooth transitions.



Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.



---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run  `install-fedora.sh`
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```


## Details of the present implementation

In this project we are limited by the nature of the information flow, in this case the information comes
periodically and also we can send it periodically. Because of this there will always be a delay which we
must compensate.

The method to compensate for delays is to calculate the next position with the data given by the **sensor fusion**.
Another condition to be considered is the position buffer. This buffer is filled with the next positions for
the car, which are processed every 0.02 seconds **BUT** normally not all the points are processed, then the
buffer contains information from the last step.

In an attempt to use the code implemented during the lections, this implementation creates a grid arond
the vehicle then makes a discrete version of the map but much more reduced. Then this grid is passed to the
path finding algorithm which has extra conditions. The extra conditions implement part of the FSM which
we saw in the lections.

- There shall not be change of line if there are vehicles at my side or one position behind of the desired lane.
Example:
```
   0 1 2
 0| # # #
 1| . # #
 2| # | .
 3| # | #
 4| # | #
 5| # | #
 6| . | . <- This maneuver shall not be possible, as it's dangerous
 7| # \ #
 8| # . |
 9| # # |
10| # # |
11| # # |
12| # # |
13| # # |
14| G G G
```

Then no path shall be returned in this case. This is implemented in [`src/path_finder.cpp:128-134`](https://github.com/yhoazk/CarND-Path-Planning-Project/blob/master/src/path_finder.cpp#L128-L134)

- The preferred movements in order of priority are: ([`src/path_finder.cpp:45`](https://github.com/yhoazk/CarND-Path-Planning-Project/blob/master/src/path_finder.cpp#L45))
    1. Keep lane
    2. Change to left lane, as in the real world
    3. Change to right lane.



- If there's a lane of cars in front, return no existing path, and apply brakes. [`src/path_finder.cpp:194`](https://github.com/yhoazk/CarND-Path-Planning-Project/blob/master/src/path_finder.cpp#L194)

Also an external condition implemented in the main loop checks that the given lane in case of change of lane line. This is implemented in the function [`src/path_finder.cpp::is_lane_free`](https://github.com/yhoazk/CarND-Path-Planning-Project/blob/master/src/path_finder.cpp#L279-L301)

* As the vehicle is not capable of moving in lateral direction only, this is represented in the possible next nodes.
  As follows.
```
       O    <- The veicle
     / | \
    #  #  # <- Turn right, keep lane and turn left respectively
```

This way the path finding algorithm represents better the car dynamics.

### Maneuver controller

As we choose to use a discrete map, the decisions made by the path_finder algorithm are also discrete, if we
just change the lane in one step the jerk will exceed the permited value. To ensure a smooth, but yet fast response
in a change of lane a *P-controller* and a small *FSM* were implemented. [`src/main.cpp:447`](https://github.com/yhoazk/CarND-Path-Planning-Project/blob/master/src/main.cpp#L447)

![P-controller](p-controller.png)

At first the car was waving from one way to another because some vershoot due to the delay on the information.
Then the FSM to control the action of the controller was also implemented.

This is a quite simple FSM which only checks if the maneuver is complete.
Once the maneuver is done, the controller is free to act again.

![maneuver_FSM](maneuver-fsm.png)

### Path generation

The path generation is implemented using the [spline library](https://github.com/yhoazk/CarND-Path-Planning-Project/blob/master/src/spline.h), but
the spline library alone does not meets the jerk minimizing trajectories. In this implementation
as a way to minimize the lateral jerk the change of lane line is smoothed by the *P-controller*.

The frenet coordinate `d` changes only proportional to the difference in the desired lane and
current lane. Next is a small diagram which shows how the paths are being generated.

![path_main](path_main.png)


## Result:

The car is able to complete the map without collisions.
This was tested 3 times.

Here is a video, but the quality is horrible. Still demostrates the ability of the car to
aviod crashes and keep a decent speed.
![](https://youtu.be/1gHnJhYkr8E)


[![horrible vieo](https://img.youtube.com/vi/1gHnJhYkr8E/0.jpg)](https://www.youtube.com/watch?v=1gHnJhYkr8E)



## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.


capture traffic to simulator:

sudo tcpdump -Aqi lo src localhost and port 4567 and greater 512
