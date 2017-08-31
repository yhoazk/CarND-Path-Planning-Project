#include <cstdio>
#include <cstdlib>
#include <vector>
#include "spline.h"

int main(int argc, char** argv) {

   std::vector<double> X, Y;

/*  X.push_back(2313.55); Y.push_back(2427.66);
X.push_back(2321.75); Y.push_back(2467.01);
X.push_back(2328.86); Y.push_back(2506.45);
X.push_back(2335.52); Y.push_back(2546.39);*/

Y.push_back(2337.74);  X.push_back(2570.21);
Y.push_back(2337.81);  X.push_back(2610.83);
Y.push_back(2338.1 );  X.push_back(2650.75);
Y.push_back(2338.96);  X.push_back(2690.69);

   tk::spline s;
   s.set_points(X,Y);    // currently it is required that X is already sorted

   double x=2337.0;
   for(int i=0; i < 50; ++i)
   {
     printf("%f, %f\n", x, s(x));
     x+=1.0f;
   }

   return EXIT_SUCCESS;
}
