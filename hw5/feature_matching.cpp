#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int num_images = 17;
string header = "/home/magiccjae/jae_stuff/classes/ee631/hw5/images/optical_flow/O";
string ending = ".jpg";

// parameters for calcOpticalFlowPyrLK
vector<Point2f> features_prev, features_next;
vector<uchar> status;
vector<float> err;

void draw_features(Mat &src);
void template_matching(int frame_jump);

int main(int, char**)
{
  int num = 1;
  Mat first = imread(header+to_string(num)+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);

  int max_corners = 500;
  double qlevel = 0.01;
  double min_distance = 10;
  // obtain initial set of features
  goodFeaturesToTrack(first_gray, features_next, max_corners, qlevel, min_distance);

  // how many frame is skipped in between frames
  int frame_jump = 1;
  // template_matching(frame_jump);
  int x_window = 20;
  int y_window = 20;
  for(int i=0; i<features_next.size(); i++){
    Rect rec(features_next.at(i).x-x_window, features_next.at(i).y-y_window, x_window*2, y_window*2);
    Mat roi = first_gray(rec);
    imshow("roi", roi);
    waitKey(0);
  }

  while(waitKey(0)!=27);
  return 0;
}

void draw_features(Mat &src){
  for(int i=0; i<features_next.size(); i++){
    circle(src, features_next.at(i), 0.3, Scalar(0,255,0), 3);
    // arrowedLine(src, features_prev.at(i), features_next.at(i), Scalar(0,0,255), 1);
  }
  return;
}

void template_matching(int frame_jump){
  return;
}
