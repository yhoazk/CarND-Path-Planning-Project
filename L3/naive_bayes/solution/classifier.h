  #ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {
  float mean_left;
  float mean_right;
  float mean_keep;

  float var_left;
  float var_right;
  float var_keep;
  const float lane_width = 4.0f;
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

  string predict(vector<double>);

};

#endif



