# Implement a Behavior Planner

In this exercise you will implement a behavior planner for highway driving. It will use prediction data to set the state of the ego vehicle to one of 5 values:

* `"KL"` - Keep Lane
* ` "LCL"` / `"LCR"` - Lane Change Left / Right
* ` "PLCL" `/ `"PLCR"` - Prepare Lane Change Left / Right

### Instructions:

1. Implement the update_state method in the vehicle.cpp class.
2. Hit Test Run and see how your car does! How fast can you get to the goal without colliding?

## Extra Practice
Provided in one of the links below is python_extra_practice, which is the same problem but written in python that you can optionally go through for extra coding practice. Also in the python_solution link the python solution is provided too, if you get stuck on the quiz see if you can convert the python solution to c++ and pass the classroom quiz with it.




### Supporting Materials
[Python Solution](./python-solution.zip)
[Python Extra Practice](./python-extra-practice.zip)





```python
def transition_function(predictions, current_fsm_state, current_pose, cost_functions, weights):
    # only consider states which can be reached from current FSM state.
    possible_successor_states = successor_states(current_fsm_state)

    # keep track of the total cost of each state.
    costs = []
    for state in possible_successor_states:
        # generate a rough idea of what trajectory we would
        # follow IF we chose this state.
        trajectory_for_state = generate_trajectory(state, current_pose, predictions)

        # calculate the "cost" associated with that trajectory.
        cost_for_state = 0
        for i in range(len(cost_functions)) :
            # apply each cost function to the generated trajectory
            cost_function = cost_functions[i]
            cost_for_cost_function = cost_function(trajectory_for_state, predictions)

            # multiply the cost by the associated weight
            weight = weights[i]
            cost_for_state += weight * cost_for_cost_function
            costs.append({'state' : state, 'cost' : cost_for_state})

    # Find the minimum cost state.
    best_next_state = None
    min_cost = 9999999
    for i in range(len(possible_successor_states)):
        state = possible_successor_states[i]
        cost  = costs[i]
        if cost < min_cost:
            min_cost = cost
            best_next_state = state 

    return best_next_state
```