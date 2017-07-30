# Implementing Naive Bayes

In this exercise you will implement a Gaussian Naive Bayes classifier to predict the behavior of vehicles on a highway. In the image below you can see the behaviors you'll be looking for on a 3 lane highway (with lanes of 4 meter width). The dots represent the d (y axis) and s (x axis) coordinates of vehicles as they either...

1. change lanes left (shown in blue)
2. keep lane (shown in black)
3. or change lanes right (shown in red)

![](./naive-bayes.png)

Your job is to write a classifier that can predict which of these three maneuvers a vehicle is engaged in given a single coordinate (sampled from the trajectories shown).

Each coordinate contains 4 pieces of information:
* _s_
* _d_
* _s_dot_
* _d_dot_

You also know the **lane width** is 4 meters (this might be helpful in engineering features for your algorithm).

## Instructions

1. Implement the `train(self, data, labels)` and `predict(self, observation)` methods in the class `GNB` in `classifier.cpp`
2. When you want to test your classifier, run the executable file.

**NOTE:**
You are welcome to use some existing implementation of a Gaussian Naive Bayes classifier.
But to get the best results you will still need to put some thought into what features you provide the algorithm when classifying. Though you will only be given the 4 coordinates listed above, you may find that by "engineering" features you may get better performance. For example: the raw value of the d coordinate may not be that useful. But d % lane_width might be helpful since it gives the relative position of a vehicle in it's lane regardless of which lane the vehicle is in.

### Helpful Resources

* [sklearn documentation on GaussianNB](http://scikit-learn.org/stable/modules/naive_bayes.html#gaussian-naive-bayes)
* [wikipedia article on Naive Bayes / GNB](https://en.wikipedia.org/wiki/Naive_Bayes_classifier#Gaussian_naive_Bayes)

## Extra Practice

Provided in one of the links below is python_extra_practice, which is the same problem but written in python that you can optionally go through for extra coding practice. Also in the python_solution link the python solution is provided too, if you get stuck on the quiz see if you can convert the python solution to c++ and pass the classroom quiz with it. The last link Nd013_Pred_Data has all the training and testing data for this problem in case you want to run the problem offline.

#### Supporting Materials
* [Nd013 Pred Data](./nd013-pred-data.zip)
* [python_extra_practice](./predictionexercise.zip)
* [python_solution](./predicition-solution.zip)


#### solution

Test points
 This points are given in Frenet-Serret coordinates which describe the road
 as a frame of reference  as the example image below.

 ![](./frenet-4.png)


 Example for the first point labeled as left.

 | s | d | s/dt | d/dt |
 |---|---|------|------|
 |34.7680729582807|0.832913812432181|8.20663863464299|-0.998961142457705|

```
34.7680729582807,0.832913812432181,8.20663863464299,-0.998961142457705
39.7084757286983,7.60549751981133,8.15974055685309,0.401671056682588
29.8549416734264,1.20617431036824,11.0470575580595,-1.47480653235101
35.5485197672162,0.496133606739075,9.45597259612092,-0.10563468956278
9.09643459631508,8.27740243210925,8.78513372808377,-0.588014416871305
11.7230679274465,7.28643168137533,7.80371499307478,-0.387755010525606
31.3362319643895,7.63749429518093,10.3943260071232,0.247879678344348
17.4315097932308,7.66663915727218,9.33174617750683,-1.18516657219584
22.1561095024838,0.227879223930593,11.5408197914272,-0.0904488856735072
17.2911040762026,6.74111195691705,9.92272107541862,-1.67616112578641
```

Labels for the points:

```
left
right
left
keep
left
left
keep
left
keep
left
```


The properties which will be taken into account for our classifier are:

* _d_ % lane width.
*   _d/dt_  
  * 0 > _d/dt_ : Rigth
  * 0 ~ _d/dt_ : keep
  * 0 < _d/dt_ : Left


At this point the _s_ and _s/dt_ values appear to not add any useful information
to the classifier.
