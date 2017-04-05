#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <algorithm> // sort
#include <fstream>
#include <stdlib.h>  // abs

using namespace cv;
using namespace std;

string header = "/home/magiccjae/jae_stuff/classes/ee631/hw7/images/T";
string ending = ".jpg";
int num_img = 18;

// parameters for calcOpticalFlowPyrLK
vector<Point2f> prev_features, next_features;
vector<uchar> status;
vector<float> err;

double principal_x = 331.6538103208;
double principal_y = 252.9284287373;
float focal_length = 825;
float gascan_width = 59;

float find_median(vector<float> &vec_tau);
void draw_features(Mat &src);

int main(int, char**)
{

  // to write the estimated frames to impact and frame number to a .txt file
  ofstream myfile("distance.txt");

  Mat first = imread(header+to_string(1)+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);

  int max_corners = 2;
  double qlevel = 0.01;
  double min_distance = 10;
  // obtain initial set of features
  int x_min = 275;
  int x_max = 375;
  int y_min = 165;
  int y_max = 175;
  // rectangle for masking the region of interest around the can
  Rect rec(Point(x_min, y_min), Point(x_max, y_max));
  Mat mask(first_gray.size(), CV_8UC1, Scalar::all(0));
  mask(rec).setTo(Scalar::all(255));
  goodFeaturesToTrack(first_gray, next_features, max_corners, qlevel, min_distance, mask);

  for(int i=1; i<num_img; i++){
    cout << i << endl;
    prev_features = next_features;
    Mat prev, next, prev_gray, next_gray;
    prev = imread(header+to_string(i)+ending);
    next = imread(header+to_string(i+1)+ending);
    cvtColor(prev, prev_gray, CV_RGB2GRAY);
    cvtColor(next, next_gray, CV_RGB2GRAY);
    calcOpticalFlowPyrLK(prev_gray, next_gray, prev_features, next_features, status, err, \
    Size(21,21), 5, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 0.001), 0, 1e-4);

    if(prev_features.size()==2){
      float distance_x = fabs(prev_features.at(0).x-prev_features.at(1).x);
      float distance_z = gascan_width*focal_length/distance_x;
      cout << distance_z << endl;
      myfile << i << " " << distance_z << endl;
      if(i==17){
        cout << i+1 << endl;
        distance_x = fabs(next_features.at(0).x-next_features.at(1).x);
        distance_z = gascan_width*focal_length/distance_x;
        cout << distance_z << endl;
        myfile << i+1 << " " << distance_z << endl;
      }
    }

    draw_features(prev);
    imshow("opf", prev);
    waitKey(0);
  }

  while(waitKey(0)!=27);
  return 0;
}

void draw_features(Mat &src){
  for(int i=0; i<prev_features.size(); i++){
    circle(src, prev_features.at(i), 0.3, Scalar(0,255,0), 3);
    arrowedLine(src, prev_features.at(i), next_features.at(i), Scalar(0,0,255), 1);
  }
  return;
}
