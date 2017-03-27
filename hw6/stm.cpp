#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <time.h>       /* time */

using namespace cv;
using namespace std;

int num_images = 6;
// string header = "/home/magiccjae/jae_stuff/classes/ee631/hw6/images/parallel_cube/ParallelCube";
// string header = "/home/magiccjae/jae_stuff/classes/ee631/hw6/images/parallel_real/ParallelReal";
// string header = "/home/magiccjae/jae_stuff/classes/ee631/hw6/images/turned_cube/TurnCube";
string header = "/home/magiccjae/jae_stuff/classes/ee631/hw6/images/turned_real/TurnReal";
string ending = ".jpg";

vector<Point2f> features_prev, features_next, prev_good, next_good, features_first, features_whatever, first_good;

void draw_features(Mat &src);
void template_matching(int frame_jump);

int x_size;
int y_size;
int num = 10;   // beginning image number
int last_num = 15;

int main(int, char**)
{
  /* initialize random seed: */
  srand (time(NULL));

  Mat first = imread(header+to_string(num)+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);
  Size image_size =  first_gray.size();
  x_size = image_size.width;
  y_size = image_size.height;

  int max_corners = 1000;
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
  // after template_matching function is excuted, 'first_good' contains good features on the first frame that remained until the last frame
  // 'next_good' contains features on the last frame that good features from the first frame moved to
  Mat last = imread(header+to_string(last_num)+ending);

  // reject outliers one more time between the feature points on the first and the last images
  Mat good_features;
  vector<Point2f> first_refined, last_refined;
  findFundamentalMat(first_good, next_good, CV_FM_RANSAC, 1, 0.999, good_features);
  // cout << good_features << endl;
  cout << "before refine: " << first_good.size() << endl;
  for(int i=0; i<first_good.size(); i++){
    if(good_features.at<bool>(i,0)==1){
      first_refined.push_back(first_good.at(i));
      last_refined.push_back(next_good.at(i));
    }
  }
  cout << "after refine: " << first_refined.size() << endl;
  // cout << first_refined << endl;

  // calibrated intrinsic parameters and distortion coefficients
  Mat M = (Mat_<double>(3,3) << 825.0900600547, 0.0000000000, 331.6538103208, 0.0000000000, 824.2672147458, 252.9284287373, 0.0000000000, 0.0000000000, 1.0000000000);
  Mat distCoeffs = (Mat_<double>(1,5) << -0.2380769337, 0.0931325835, 0.0003242537, -0.0021901930, 0.4641735616);

  // fundamental matrix with undistorted points
  vector<Point2f> first_undistorted, last_undistorted;
  undistortPoints(first_refined, first_undistorted, M, distCoeffs, noArray(), M);
  undistortPoints(last_refined, last_undistorted, M, distCoeffs, noArray(), M);
  // At this point first_undistorted, last_undistorted are already aligned on y which means 'rectified'.
  // fundamental matrix
  Mat F = findFundamentalMat(first_undistorted, last_undistorted, CV_FM_8POINT);
  // essential matrix from fundamental matrix and intrinsic parameters
  Mat E = M.t()*F*M;

  // rotation and translation up to a scale factor.
  Mat R,t;
  recoverPose(E, first_undistorted, last_undistorted, R, t, M.at<double>(0,0), Point2d(M.at<double>(0,2), M.at<double>(1,2)));
  // cout << "===== t =====" << endl;
  // cout << t << endl;

  // parallel = 1 for parallel image set
  // parallel =2 for turned image set
  int parallel = 2;
  double scale_factor = 0;
  if(parallel==1){
    scale_factor = 2.08;
  }
  else{
    scale_factor = 2.23;
  }
  // R1, R2, P1, P2, Q are output matrices from stereoRectify
  // "t" multiplied by scale_factor to make the closest points on the cube are about 20 in Z axis.
  Mat t_scaled = t*scale_factor;
  cout << "===== t_scaled =====" << endl;
  cout << t_scaled << endl;

  Mat R1, R2, P1, P2, Q;
  stereoRectify(M, distCoeffs, M, distCoeffs, \
  image_size, R, t_scaled, R1, R2, P1, P2, Q, 0, -1, image_size, 0, 0);

  vector<Point3f> first_input;
  vector<Point3f> last_input;
  // augment disparity to make suitable inputs for perspectiveTransform function
  for(int i=0; i<first_undistorted.size(); i++){
    float disparity = first_undistorted.at(i).x - last_undistorted.at(i).x;
    Point3f a_point(first_undistorted.at(i).x, first_undistorted.at(i).y, disparity);
    Point3f another_point(last_undistorted.at(i).x, last_undistorted.at(i).y, disparity);
    first_input.push_back(a_point);
    last_input.push_back(another_point);
  }

  // find_scale = 1 to pick the closest points from the cube to find scale factor.
  // find_scale = 0 to pick random 4 points and display their 3D coordinates
  int find_scale = 0;
  // turned = 1 to grab closest points from the TurnedCube image set.
  if(find_scale==1){
    // selecting features that are on the closest point on the cube
    vector<Point3f> first_closest;
    int x_min = 280;
    int x_max = 310;
    int y_min = 75;
    int y_max = 450;
    if(parallel==2){
      x_min = 220;
      x_max = 250;
      y_min = 75;
      y_max = 450;
    }
    for(int i=0; i<first_input.size(); i++){
      if(first_input.at(i).x>x_min && first_input.at(i).x<x_max && first_input.at(i).y>y_min && first_input.at(i).y<y_max){
        first_closest.push_back(first_input.at(i));
      }
    }
    vector<Point3f> first_3d;
    perspectiveTransform(first_closest, first_3d, Q);
    cout << "===== first 3D =====" << endl;
    cout << first_3d << endl;

    for(int i=0; i<first_closest.size(); i++){
        circle(first, Point(first_closest.at(i).x, first_closest.at(i).y), 1, Scalar(0,255,0), 2);
        string text_x = to_string(first_3d.at(i).x);
        string text_y = to_string(first_3d.at(i).y);
        string text_z = to_string(first_3d.at(i).z);
        string text = "(" + text_x + ", " + text_y + ", " + text_z + ")";
        Point text_point(first_closest.at(i).x, first_closest.at(i).y);
        putText(first, text, Point(text_point.x, text_point.y), \
                FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);
    }
  }
  else{
    vector<Point3f> first_3d, last_3d;
    perspectiveTransform(first_input, first_3d, Q);
    perspectiveTransform(last_input, last_3d, Q);
    for(int i=0; i<4; i++){
      int rand_num = rand()%first_3d.size();
      // cout << "rand_num: " << rand_num << endl;
      Point3f a_point = first_3d.at(rand_num);
      circle(first, Point(first_input.at(rand_num).x, first_input.at(rand_num).y), 1, Scalar(0,255,0), 2);
      string text_x = "x: " + to_string(a_point.x);
      string text_y = "y: " + to_string(a_point.y);
      string text_z = "z: " + to_string(a_point.z);
      Point text_point(first_input.at(rand_num).x, first_input.at(rand_num).y);
      putText(first, text_x, Point(text_point.x, text_point.y), \
              FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);
      putText(first, text_y, Point(text_point.x, text_point.y+10), \
              FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);
      putText(first, text_z, Point(text_point.x, text_point.y+20), \
              FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);


      // Point3f another_point = last_3d.at(rand_num);
      // circle(last, Point(last_input.at(rand_num).x, last_input.at(rand_num).y), 1, Scalar(0,255,0), 2);
      // string text_x1 = "x: " + to_string(another_point.x);
      // string text_y1 = "y: " + to_string(another_point.y);
      // string text_z1 = "z: " + to_string(another_point.z);
      // Point text_point1(last_input.at(rand_num).x, last_input.at(rand_num).y);
      // putText(last, text_x1, Point(text_point1.x, text_point1.y), \
      //         FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);
      // putText(last, text_y1, Point(text_point1.x, text_point1.y+10), \
      //         FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);
      // putText(last, text_z1, Point(text_point1.x, text_point1.y+20), \
      //         FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);

    }
  }

  imshow("first", first);
  // imshow("last", last);

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
  int x_window = 60;
  int y_window = 60;

  for(int i=num; i<num+num_images-frame_jump; i++){
    // cout << i << endl;
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
    findFundamentalMat(features_prev, features_next, CV_FM_RANSAC, 1, 0.999, good_features);
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

    // cout << "num of good features: " << howmany_good << endl;

    features_prev.clear();
    features_prev = next_good;
    features_next.clear();
  }
  return;
}
