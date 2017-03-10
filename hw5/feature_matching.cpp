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

  while(waitKey(0)!=27);
  return 0;
}

void draw_features(Mat &src){
  for(int i=0; i<features_next.size(); i++){
    circle(src, features_next.at(i), 0.3, Scalar(0,255,0), 3);
    arrowedLine(src, features_prev.at(i), features_next.at(i), Scalar(0,0,255), 1);
  }
  return;
}

void template_matching(int frame_jump){
  int x_window = 10;
  int y_window = 10;
  Mat prev, prev_gray, next, next_gray;
  Mat result;

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
      if(x_initial-x_window < 0){
        x_initial = 0;
      }
      else if(x_initial+x_window*2 >= x_size){
        int x_over = x_initial+x_window*2-x_size;
        x_initial = x_initial-x_window-x_over;
      }
      int y_initial = features_prev.at(i).y;
      if(y_initial-y_window < 0){
        y_initial = 0;
      }
      else if(y_initial+y_window*2 >= y_size){
        int y_over = y_initial+y_window*2-y_size;
        y_initial = y_initial-y_window-y_over;
      }
      // cout << x_initial << ", " << y_initial << endl;
      Rect rec(x_initial, y_initial, x_window*2, y_window*2);
      Mat roi = prev_gray(rec);
      matchTemplate(next_gray, roi, result, CV_TM_SQDIFF);
      Point min_loc;
      minMaxLoc(result, 0, 0, &min_loc, 0, Mat());
      // cout << min_loc << endl;
      features_next.push_back(min_loc);
      // arrowedLine(prev, features_prev.at(i), min_loc, Scalar(0,0,255), 1);
    }
    draw_features(prev);
    imshow("result",prev);
    waitKey(0);
    features_prev = features_next;
    features_next.clear();
  }
  return;
}
