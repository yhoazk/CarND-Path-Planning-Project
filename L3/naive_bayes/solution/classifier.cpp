#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"

/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{

  /*
    Trains the classifier with N data points and labels.

    INPUTS
    data - array of N observations
      - Each observation is a tuple with 4 values: s, d,
        s_dot and d_dot.
      - Example : [
          [3.5, 0.1, 5.9, -0.02],
          [8.0, -0.3, 3.0, 2.2],
          ...
        ]

    labels - array of N labels
      - Each label is one of "left", "keep", or "right".

      - Count the number of labels
      - Count the number of labels for each type
      - get the mean omega_d for each label
      - get the variance sum (x- x_mean)^ 2

  */
  size_t count_left  = 0;
  size_t count_keep  = 0;
  size_t count_right = 0;

  double_t sum_left  = 0;
  double_t sum_keep  = 0;
  double_t sum_right = 0;

  if(data.size() != labels.size())
  {
    /* Error, vectors do not match */
    return;
  } else {

    for(size_t i=0; i < data.size(); ++i)
    {
      if(0 == labels[i].compare(possible_labels[0]))
      {
        /*Left*/
        ++count_left;
        sum_left += fmod(data[i][1] , lane_width) + data[i][1]/lane_width;
      }
      else if(0 == labels[i].compare(possible_labels[1]))
      {
        /*keep*/
        ++count_keep;
        sum_keep += fmod(data[i][1], lane_width) + data[i][1]/lane_width;

      } else{
        /* Assuming that there are not other values than the possible_labels */
        /* right */
        ++count_right;
        sum_right += fmod(data[i][1], lane_width) + data[i][1]/lane_width;

      }
    }
    std::cout << "=========================================" << std::endl;
    std::cout << "Mean left: " << sum_left /* (double_t) count_left */ << std::endl;
    std::cout << "Mean keep: " << sum_keep /*(double_t) count_keep*/  << std::endl;
    std::cout << "Mean right: " <<sum_right /* (double_t) count_right*/ << std::endl;
    std::cout << "=========================================" << std::endl;

  }

}

string GNB::predict(vector<double>)
{
  /*
    Once trained, this method is called and expected to return
    a predicted behavior for the given observation.

    INPUTS

    observation - a 4 tuple with s, d, s_dot, d_dot.
      - Example: [3.5, 0.1, 8.5, -0.2]

    OUTPUT

    A label representing the best guess of the classifier. Can
    be one of "left", "keep" or "right".
    """
    # TODO - complete this
  */

  return this->possible_labels[1];

}