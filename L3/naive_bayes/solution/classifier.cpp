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
  double_t var_temp = 0;
  vector<double_t> omega_d(data.size());

  if(data.size() != labels.size())
  {
    /* Error, vectors do not match */
    return;
  } else {

    for(size_t i=0; i < data.size(); ++i)
    {
      omega_d[i] = data[i][1] - (lane_width * ceil(data[i][1]/ (double_t)lane_width)); //fmod(data[i][1] , lane_width);// + data[i][1]/lane_width;

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

    p_left  = count_left/(double_t)data.size();
    p_keep  = count_keep/(double_t)data.size();
    p_right = count_right/(double_t)data.size();

    /* Get the variance */
    for (int j = 0; j < data.size(); ++j) {
      if(0 == labels[j].compare(possible_labels[0]))
      {
        /*Left*/
        var_temp = omega_d[j] - mean_left;
        var_left += var_temp * var_temp;
      }
      else if(0 == labels[j].compare(possible_labels[1]))
      {
        /*keep*/
        var_temp = omega_d[j] - mean_keep;
        var_keep += var_temp * var_temp;

      } else{
        /* Assuming that there are not other values than the possible_labels */
        /* right */
        var_temp = omega_d[j] - mean_right;
        var_right += var_temp * var_temp;

      }
    }
    std::cout << "=========================================" << std::endl;
    std::cout << "var left: " << var_left << std::endl;
    std::cout << "var keep: " << var_keep  << std::endl;
    std::cout << "var right: " <<var_right << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Mean left: " << mean_left << std::endl;
    std::cout << "Mean keep: " << mean_keep << std::endl;
    std::cout << "Mean right: " <<mean_right << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "count left: " <<count_left << std::endl;
    std::cout << "count keep: " <<count_keep << std::endl;
    std::cout << "count right: " <<count_right << std::endl;
    std::cout << "count total: " <<count_right+count_keep+count_left << std::endl;
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
  #if 0
  double_t p_l, p_k, p_r, omega_d;
  omega_d = x[1] - (lane_width * ceil(x[1]/ (double_t)lane_width)); // fmod(x[1] , lane_width);// + x[1]/lane_width;
  cout << "d: " << x[1]  << " omega_d: " << omega_d ;
  /*calculate the probability that the measurement is left*/
  p_l = p_left * calculate_posterior(omega_d, mean_left, var_left);
  /*calculate the probability that the measurement is keep*/
  p_k = p_keep * calculate_posterior(omega_d, mean_keep, var_keep);
  /*calculate the probability that the measurement is right*/
  p_r = p_right * calculate_posterior(omega_d, mean_right, var_right);

  std::cout << " - pl:" << p_l << " pk: " << p_k << " pr: " << p_r << std::endl;

  if((p_l > p_k) && (p_l > p_r))
  {
    result = 0;
    std::cout << "Left" << std::endl;
  }
  if((p_k > p_l) && (p_k > p_r))
  {
    result = 1;
    std::cout << "Keep" << std::endl;
  }
  if((p_r > p_l) && (p_r > p_k))
  {
    result = 2;
    std::cout << "right" << std::endl;
  }
  #endif
  if(x[3] < -0.15f)
  {
    result = 0;
  }
  else if((x[3] >= -0.15f) && (x[3] < 0.15f))
  {
    result = 1;
  }
  else if(x[3] >= 0.15f){
    result = 2;
  }
  return this->possible_labels[result];

}