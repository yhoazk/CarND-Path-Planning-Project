  #ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {
  double mean_left;
  double mean_right;
  double mean_keep;

  double var_left;
  double var_right;
  double var_keep;
  const double lane_width = 4.0f;
  double p_left;
  double p_keep;
  double p_right;
public:

  vector<string> possible_labels = {"left","keep","right"};


  /**
    * Constructor
    */
  GNB();

  /**
   * Destructor
   */
  virtual ~GNB();

  void train(vector<vector<double> > data, vector<string>  labels);

  double_t calculate_posterior(double_t, double_t, double_t);

  string predict(vector<double>);

};

#endif



