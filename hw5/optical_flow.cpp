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
void calculate_optf(int frame_jump);

int x_size;
int y_size;

VideoWriter vout;

int main(int, char**)
{
  // video out
  int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
  double fps = 25.0;                          // framerate of the created video stream
  string filename = "./optial_flow.avi";             // name of the output video file

  int num = 1;
  Mat first = imread(header+to_string(num)+ending);

  bool isColor = (first.type() == CV_8UC3);
  vout.open(filename, codec, fps, first.size(), isColor);

  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);
  Size image_size =  first_gray.size();
  x_size = image_size.width;
  y_size = image_size.height;

  int max_corners = 500;
  double qlevel = 0.01;
  double min_distance = 10;
  // obtain initial set of features
  int mask_size = 50;
  Rect rec2(Point(mask_size, mask_size), Point(x_size-mask_size, y_size-mask_size));
  Mat mask(first_gray.size(), CV_8UC1, Scalar::all(0));
  mask(rec2).setTo(Scalar::all(255));
  goodFeaturesToTrack(first_gray, features_next, max_corners, qlevel, min_distance, mask);
  // how many frame is skipped in between frames
  int frame_jump = 1;
  calculate_optf(frame_jump);

  goodFeaturesToTrack(first_gray, features_next, max_corners, qlevel, min_distance, mask);
  frame_jump = 2;
  calculate_optf(frame_jump);

  goodFeaturesToTrack(first_gray, features_next, max_corners, qlevel, min_distance, mask);
  frame_jump = 3;
  calculate_optf(frame_jump);

  while(waitKey(0)!=27);
  return 0;
}

void draw_features(Mat &src){
  for(int i=0; i<features_next.size(); i++){
    circle(src, features_prev.at(i), 0.3, Scalar(0,255,0), 3);
    arrowedLine(src, features_prev.at(i), features_next.at(i), Scalar(0,0,255), 1);
  }
  return;
}

void calculate_optf(int frame_jump){
  Mat prev, prev_gray, next, next_gray;
  for(int i=1; i<=num_images-frame_jump; i++){
    cout << i << endl;
    features_prev = features_next;
    prev = imread(header+to_string(i)+ending);
    next = imread(header+to_string(i+frame_jump)+ending);
    cvtColor(prev, prev_gray, CV_RGB2GRAY);
    cvtColor(next, next_gray, CV_RGB2GRAY);
    calcOpticalFlowPyrLK(prev_gray, next_gray, features_prev, features_next, status, err, \
    Size(21,21), 5, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 0.001), 0, 1e-4);
    draw_features(prev);
    imshow("opf", prev);

    vout.write(prev);
    waitKey(0);
  }
}
