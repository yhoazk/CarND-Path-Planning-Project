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

  vector<double_t> omega_d(data.size());

  if(data.size() != labels.size())
  {
    /* Error, vectors do not match */
    return;
  } else {

    for(size_t i=0; i < data.size(); ++i)
    {
      omega_d[i] =  fmod(data[i][1] , lane_width);// + data[i][1]/lane_width;
      if(0 == labels[i].compare(possible_labels[0]))
      {
        /*Left*/
        ++count_left;
        sum_left += omega_d[i];
      }
      else if(0 == labels[i].compare(possible_labels[1]))
      {
        /*keep*/
        ++count_keep;
        sum_keep += omega_d[i];

      } else{
        /* Assuming that there are not other values than the possible_labels */
        /* right */
        ++count_right;
        sum_right += omega_d[i];

      }
    }


    mean_left = sum_left / (double_t) count_left;
    mean_keep = sum_keep / (double_t) count_keep;
    mean_right= sum_right / (double_t) count_right;

    /* Get the variance */
    for (int j = 0; j < data.size(); ++j) {
      if(0 == labels[j].compare(possible_labels[0]))
      {
        /*Left*/
        var_left += omega_d[j] - mean_left;
      }
      else if(0 == labels[j].compare(possible_labels[1]))
      {
        /*keep*/
        var_keep += omega_d[j] - mean_keep;

      } else{
        /* Assuming that there are not other values than the possible_labels */
        /* right */
        var_right += omega_d[j] - mean_right;

      }
    }
    std::cout << "=========================================" << std::endl;
    std::cout << "Mean left: " << var_left << std::endl;
    std::cout << "Mean keep: " << var_keep  << std::endl;
    std::cout << "Mean right: " <<var_right << std::endl;
    std::cout << "=========================================" << std::endl;

  }

}

double_t GNB::calculate_posterior(double d, double_t m, double_t v)
{
  double_t  n,inv_sqrt;
  n = (d - m);
  n *=n;
  n = -n/(2.0f * v);

 inv_sqrt = sqrt(2.0f * M_PI *  v);

  return exp(n)/inv_sqrt;
}

string GNB::predict(vector<double> x)
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
  unsigned int result = 0; //<- this means left as result
  double_t p_l, p_k, p_r;
  /*calculate the probability that the measurement is left*/
  p_l = calculate_posterior(x[1], mean_left, var_left);
  /*calculate the probability that the measurement is keep*/
  p_k = calculate_posterior(x[1], mean_keep, var_keep);
  /*calculate the probability that the measurement is right*/
  p_r = calculate_posterior(x[1], mean_right, var_right);

  std::cout << "pl:" << p_l << " pk: " << p_k << " pr: " << p_r << std::endl;

  if((p_l > p_k) && (p_l > p_r))
  {
    result = 0;
    std::cout << "Left" << std::endl;
  }
  if((p_l > p_k) && (p_l > p_r))
  {
    result = 1;
    std::cout << "Keep" << std::endl;
  }
  if((p_l > p_k) && (p_l > p_r))
  {
    result = 2;
    std::cout << "right" << std::endl;
  }
  return this->possible_labels[result];

}