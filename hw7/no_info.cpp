#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <algorithm> // sort
#include <fstream>

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

float find_median(vector<float> &vec_tau);
void draw_features(Mat &src);

int main(int, char**)
{

  // to write the estimated frames to impact and frame number to a .txt file
  ofstream myfile("tau.txt");

  Mat first = imread(header+to_string(1)+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);

  int max_corners = 50;
  double qlevel = 0.01;
  double min_distance = 10;
  // obtain initial set of features
  int x_min = 200;
  int x_max = 400;
  int y_min = 100;
  int y_max = 380;
  // rectangle for masking the region of interest around the can
  Rect rec(Point(x_min, y_min), Point(x_max, y_max));
  int x_min2 = 280;
  int x_max2 = 360;
  int y_min2 = 200;
  int y_max2 = 280;
  // rectangle for unmasking the region that is really close to the can assuming they cause bad result due to too low parallax.
  Rect rec2(Point(x_min2, y_min2), Point(x_max2, y_max2));
  Mat mask(first_gray.size(), CV_8UC1, Scalar::all(0));
  mask(rec).setTo(Scalar::all(255));
  mask(rec2).setTo(Scalar::all(0));
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

    vector<float> vec_tau;
    // find expansion ratio 'a' - see lecture slide 25 page.3
    for(int j=0; j<prev_features.size(); j++){
      if(status.at(j)==1){
        float ax = (next_features.at(j).x-principal_x) / (prev_features.at(j).x-principal_x);
        float ay = (next_features.at(j).y-principal_y) / (prev_features.at(j).y-principal_y);
        float tau_x = ax / (ax-1);
        float tau_y = ay / (ay-1);
        vec_tau.push_back(tau_y);
      }
    }
    float final_tau = find_median(vec_tau);
    cout << final_tau << endl;
    myfile << i << " " << final_tau << endl;
    draw_features(prev);
    imshow("opf", prev);
    waitKey(0);
  }

  while(waitKey(0)!=27);
  return 0;
}

float find_median(vector<float> &vec_tau){
  sort(vec_tau.begin(), vec_tau.end());
  return vec_tau.at(vec_tau.size()/2);
}

void draw_features(Mat &src){
  for(int i=0; i<prev_features.size(); i++){
    circle(src, prev_features.at(i), 0.3, Scalar(0,255,0), 3);
    arrowedLine(src, prev_features.at(i), next_features.at(i), Scalar(0,0,255), 1);
  }
  return;
}
