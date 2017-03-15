#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int num_images = 6;
// string header = "/home/magiccjae/jae_stuff/classes/ee631/hw5/images/parallel_cube/ParallelCube";
// string header = "/home/magiccjae/jae_stuff/classes/ee631/hw5/images/parallel_real/ParallelReal";
// string header = "/home/magiccjae/jae_stuff/classes/ee631/hw5/images/turned_cube/TurnCube";
string header = "/home/magiccjae/jae_stuff/classes/ee631/hw5/images/turned_real/TurnReal";
string ending = ".jpg";

vector<Point> features_prev, features_next, prev_good, next_good, features_first, features_whatever, first_good;

void draw_features(Mat &src);
void template_matching(int frame_jump);

int x_size;
int y_size;
int num = 10;   // beginning image number
int last_num = 15;

int main(int, char**)
{
  Mat first = imread(header+to_string(num)+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);
  Size image_size =  first_gray.size();
  x_size = image_size.width;
  y_size = image_size.height;

  int max_corners = 300;
  double qlevel = 0.01;
  double min_distance = 10;
  // obtain initial set of features
  goodFeaturesToTrack(first_gray, features_prev, max_corners, qlevel, min_distance);
  // creating the same vector that contains the features from first image
  for(int i=0; i<features_prev.size(); i++){
    features_first.push_back(features_prev.at(i));
    first_good.push_back(features_prev.at(i));
  }

  // how many frame is skipped in between frames
  int frame_jump = 1;
  template_matching(frame_jump);

  for(int j=0; j<features_first.size(); j++){
    circle(first, features_first.at(j), 0.3, Scalar(0,255,0), 3);
  }

  for(int j=0; j<first_good.size(); j++){
    arrowedLine(first, first_good.at(j), next_good.at(j), Scalar(0,0,255), 1);
  }
  // first image with arrows pointing to corresponding features to the last image
  imshow("result", first);

  Mat last = imread(header+to_string(last_num)+ending);
  for(int j=0; j<first_good.size(); j++){
    circle(last, next_good.at(j), 0.3, Scalar(0,255,0), 3);
  }
  imshow("result1", last);

  while(waitKey(0)!=27);
  return 0;
}

void template_matching(int frame_jump){
  // template size
  int x_template = 30;
  int y_template = 30;
  Mat prev, prev_gray, next, next_gray;
  Mat result;

  // search window size
  int x_window = 50;
  int y_window = 50;

  for(int i=num; i<num+num_images-frame_jump; i++){
    cout << i << endl;
    prev = imread(header+to_string(i)+ending);
    next = imread(header+to_string(i+frame_jump)+ending);
    cvtColor(prev, prev_gray, CV_RGB2GRAY);
    cvtColor(next, next_gray, CV_RGB2GRAY);

    // make templates that is square as a feature point being center
    for(int j=0; j<features_prev.size(); j++){
      // boundary check for template
      int x_initial = features_prev.at(j).x;
      int x_last = x_initial + x_template;
      if(x_initial+x_template >= x_size){
        x_last = x_size;
      }
      int x_t_width = x_last - x_initial;

      int y_initial = features_prev.at(j).y;
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
      normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat() );
      Point max_loc;
      minMaxLoc(result, 0, 0, 0, &max_loc, Mat());
      Point real_match(max_loc.x+x_tl, max_loc.y+y_tl);
      features_next.push_back(real_match);
    }
    Mat good_features;
    findFundamentalMat(features_prev, features_next, CV_FM_RANSAC, 3, 0.99, good_features);
    // cout << good_features << endl;

    int howmany_good = 0;
    // prev_good, next_good vectors store good features in previous image and next image in order to draw arrowedLine
    prev_good.clear();
    next_good.clear();
    for(int j=0; j<features_next.size(); j++){
      if(good_features.at<bool>(j,0)==1){
        prev_good.push_back(features_prev.at(j));
        next_good.push_back(features_next.at(j));
        first_good.push_back(first_good.at(j));
        howmany_good++;
      }
    }
    first_good.erase(first_good.begin(),first_good.begin()+features_next.size());

    cout << "num of good features: " << howmany_good << endl;
    cout << "double checking: " << first_good.size() << endl;
    // for(int j=0; j<features_prev.size(); j++){
    //   circle(prev, features_prev.at(j), 0.3, Scalar(0,255,0), 3);
    // }
    // for(int j=0; j<prev_good.size(); j++){
    //   arrowedLine(prev, prev_good.at(j), next_good.at(j), Scalar(0,0,255), 1);
    // }
    //
    // imshow("result", prev);
    // waitKey(0);
    features_prev.clear();
    features_prev = next_good;
    features_next.clear();
  }
  return;
}
