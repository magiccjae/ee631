#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int num_images = 17;
string header = "/home/magiccjae/jae_stuff/classes/ee631/hw5/images/optical_flow/O";
string ending = ".jpg";

vector<Point> features_prev, features_next;

void draw_features(Mat &src);
void template_matching(int frame_jump);

int x_size;
int y_size;

int main(int, char**)
{
  int num = 1;
  Mat first = imread(header+to_string(num)+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);
  Size image_size =  first_gray.size();
  x_size = image_size.width;
  y_size = image_size.height;

  int max_corners = 500;
  double qlevel = 0.01;
  double min_distance = 10;
  // obtain initial set of features
  goodFeaturesToTrack(first_gray, features_prev, max_corners, qlevel, min_distance);
  // draw_features(first);

  // how many frame is skipped in between frames
  int frame_jump = 1;
  template_matching(frame_jump);

  goodFeaturesToTrack(first_gray, features_prev, max_corners, qlevel, min_distance);
  frame_jump = 2;
  template_matching(frame_jump);

  goodFeaturesToTrack(first_gray, features_prev, max_corners, qlevel, min_distance);
  frame_jump = 3;
  template_matching(frame_jump);

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

void template_matching(int frame_jump){
  // template size
  int x_template = 10;
  int y_template = 10;
  Mat prev, prev_gray, next, next_gray;
  Mat result;

  // search window size
  int x_window = 30;
  int y_window = 30;

  for(int i=1; i<=num_images-frame_jump; i++){
    cout << i << endl;
    prev = imread(header+to_string(i)+ending);
    next = imread(header+to_string(i+frame_jump)+ending);
    cvtColor(prev, prev_gray, CV_RGB2GRAY);
    cvtColor(next, next_gray, CV_RGB2GRAY);

    // make templates that is square as a feature point being center
    for(int i=0; i<features_prev.size(); i++){
      // boundary check for template
      int x_initial = features_prev.at(i).x;
      int x_last = x_initial + x_template;
      if(x_initial+x_template >= x_size){
        x_last = x_size;
      }
      int x_t_width = x_last - x_initial;

      int y_initial = features_prev.at(i).y;
      int y_last = y_initial + y_template;
      if(y_initial+y_template >= y_size){
        y_last = y_size;
      }
      int y_t_width = y_last - y_initial;
      // cout << x_initial << ", " << y_initial << endl;

      // construct template rectangle
      Rect rec(x_initial, y_initial, x_t_width, y_t_width);
      Mat my_template = prev_gray(rec);

      // construct search window
      // x, y top-left and bottom-right
      int x_tl = x_initial-x_window;
      if(x_tl <= 0){
        x_tl = 0;
      }
      int y_tl = y_initial-y_window;
      if(y_tl <= 0){
        y_tl = 0;
      }
      int x_br = x_initial+x_window;
      if(x_br >= x_size-1){
        x_br = x_size-1;
      }
      int y_br = y_initial+y_window;
      if(y_br >= y_size-1){
        y_br = y_size-1;
      }

      Rect rec1(Point(x_tl,y_tl), Point(x_br,y_br));
      Mat search_window = next_gray(rec1);
      matchTemplate(search_window, my_template, result, TM_CCORR_NORMED);
      Point max_loc;
      minMaxLoc(result, 0, 0, 0, &max_loc, Mat());
      Point real_match(max_loc.x+x_tl, max_loc.y+y_tl);
      features_next.push_back(real_match);
    }
    draw_features(prev);
    imshow("result",prev);
    waitKey(0);
    features_prev = features_next;
    features_next.clear();
  }
  return;
}
