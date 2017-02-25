#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int main(int, char**)
{
  string left_header = "/home/magiccjae/jae_stuff/classes/ee631/hw4/images/ballL";
  string right_header = "/home/magiccjae/jae_stuff/classes/ee631/hw4/images/ballR";
  string ending = ".bmp";
  Mat left_background = imread(left_header+to_string(0)+ending);
  Mat right_background = imread(right_header+to_string(0)+ending);
  cvtColor(left_background, left_background, CV_BGR2GRAY);
  cvtColor(right_background, right_background, CV_BGR2GRAY);
  Mat left_image, right_image;
  for(int i=0; i<50; i++){
    left_image

    waitKey(0);
  }


  while(waitKey(0)!=27);

  return 0;
}
