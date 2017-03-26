#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

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

  // calibrated intrinsic parameters and distortion coefficients
  Mat M = (Mat_<double>(3,3) << 825.0900600547, 0.0000000000, 331.6538103208, 0.0000000000, 824.2672147458, 252.9284287373, 0.0000000000, 0.0000000000, 1.0000000000);
  Mat distCoeffs = (Mat_<double>(1,5) << -0.2380769337, 0.0931325835, 0.0003242537, -0.0021901930, 0.4641735616);

  // fundamental matrix with undistorted points
  vector<Point2f> first_undistorted, last_undistorted;
  undistortPoints(first_refined, first_undistorted, M, distCoeffs, noArray(), M);
  undistortPoints(last_refined, last_undistorted, M, distCoeffs, noArray(), M);
  Mat F = findFundamentalMat(first_undistorted, last_undistorted, CV_FM_8POINT);
  // cout << "======= Fundamental Matrix =======" << endl;
  // cout << F << endl;

  // essential matrix
  Mat E = M.t()*F*M;

  // method 1 means matrix calculation. 2 means to use 'recoverPose' function
  int method = 2;
  if(method==1){
    // see slide 22, page 7
    Mat u, vt, w;
    SVD::compute(E, w, u, vt);
    // cout << "======= w =======" << endl;
    // cout << w << endl;
    normalize(w,w);
    // cout << "======= w normalized =======" << endl;
    // cout << w << endl;
    // cout << "======= u =======" << endl;
    // cout << u << endl;
    // cout << "======= vt =======" << endl;
    // cout << vt << endl << endl;

    // calculate R, T_hat(estimated translation)
    Mat R_tz1 = (Mat_<double>(3,3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
    Mat R_tz2 = (Mat_<double>(3,3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
    Mat R1 = u*R_tz1*vt;
    Mat R2 = u*R_tz2*vt;
    Mat diagonal = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 0);
    Mat T_hat1 = u*R_tz1*diagonal*u.t();
    Mat T_hat2 = u*R_tz2*diagonal*u.t();

    // print out the resulting 4 pairs
    cout << "!!!!!!!!!!!!!!! set 1 !!!!!!!!!!!!!!!" << endl;
    cout << "======= R1 =======" << endl;
    cout << R1 << endl;
    cout << "======= T1 =======" << endl;
    cout << T_hat1 << endl << endl;
    cout << "!!!!!!!!!!!!!!! set 2 !!!!!!!!!!!!!!!" << endl;
    cout << "======= R1 =======" << endl;
    cout << R1 << endl;
    cout << "======= T2 =======" << endl;
    cout << T_hat2 << endl << endl;
    cout << "!!!!!!!!!!!!!!! set 3 !!!!!!!!!!!!!!!" << endl;
    cout << "======= R2 =======" << endl;
    cout << R2 << endl;
    cout << "======= T1 =======" << endl;
    cout << T_hat1 << endl << endl;
    cout << "!!!!!!!!!!!!!!! set 4 !!!!!!!!!!!!!!!" << endl;
    cout << "======= R2 =======" << endl;
    cout << R2 << endl;
    cout << "======= T2 =======" << endl;
    cout << T_hat2 << endl << endl;
    // T = [0  -Tz  Ty
    //      Tz  0  -Tx
    //      Ty  Tx  0]
  }
  else if(method==2){
    Mat R,t;
    recoverPose(E, first_undistorted, last_undistorted, R, t, M.at<double>(0,0), Point2d(M.at<double>(0,2), M.at<double>(1,2)));
    cout << "======= R =======" << endl;
    cout << R << endl;
    cout << "======= t =======" << endl;
    cout << t << endl;
    cout << "======= E =======" << endl;
    cout << E << endl;
    cout << "======= F =======" << endl;
    cout << F << endl;
  }
  else{
    cout << "intentionally doing nothing" << endl;
  }

  for(int i=0; i<first_refined.size(); i++){
      circle(first, first_refined.at(i), 2, Scalar(0,255,0));
      circle(last, last_refined.at(i), 2, Scalar(0,255,0));
  }

  imshow("first", first);
  imshow("last", last);

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

    cout << "num of good features: " << howmany_good << endl;

    features_prev.clear();
    features_prev = next_good;
    features_next.clear();
  }
  return;
}
