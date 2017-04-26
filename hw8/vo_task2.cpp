#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>   // abs

using namespace cv;
using namespace std;

string header = "/home/magiccjae/jae_stuff/classes/ee631/hw8/BYU_Hallway_Sequence/BYU_Hallway_Sequence/";
string ending = ".png";
char buf[6];
int num_img = 2241;

// parameters for calcOpticalFlowPyrLK
vector<Point2f> prev_features, next_features;
vector<uchar> status;
vector<float> err;
vector<Point2f> prev_refined, next_refined;
vector<Point2f> prev_undistorted, next_undistorted;

// calibrated intrinsic parameters
Mat M = (Mat_<double>(3,3) << 677.41251774486568, 0, 323.12557438767283, 0, 680.73800850564749, 224.77413395670021, 0, 0, 1);
Mat distCoeffs;
Mat fourth_row = (Mat_<double>(1,4) << 0, 0, 0, 1);
Mat total_transformation = (Mat_<double>(4,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
Mat prev_transformation = (Mat_<double>(4,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
Mat current_transformation;

void draw_features(Mat &src);

int main(int, char**)
{
  // to write the estimated frames to impact and frame number to a .txt file
  ofstream myfile("VO_result2.txt");

  sprintf(buf, "%06d", 0);
  Mat first = imread(header+buf+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);

  int max_corners = 500;
  double qlevel = 0.001;
  double min_distance = 10;
  // obtain initial set of features
  int x_min = 40;
  int x_max = 600;
  int y_min = 40;
  int y_max = 320;
  // rectangle for masking the region of interest around the can
  Rect rec(Point(x_min, y_min), Point(x_max, y_max));
  Mat mask(first_gray.size(), CV_8UC1, Scalar::all(0));
  mask(rec).setTo(Scalar::all(255));

  // how many frames are skipped to get more baselines
  int frame_jump = 3;

  for(int i=0; i<num_img-frame_jump; i=i+frame_jump){
    prev_refined.clear();
    next_refined.clear();
    cout << i << endl;
    char buf1[6];
    char buf2[6];
    sprintf(buf1, "%06d", i);
    sprintf(buf2, "%06d", i+frame_jump);

    Mat prev, next, prev_gray, next_gray;
    prev = imread(header+buf1+ending);
    next = imread(header+buf2+ending);

    cvtColor(prev, prev_gray, CV_RGB2GRAY);
    cvtColor(next, next_gray, CV_RGB2GRAY);

    goodFeaturesToTrack(prev_gray, prev_features, max_corners, qlevel, min_distance, mask);
    calcOpticalFlowPyrLK(prev_gray, next_gray, prev_features, next_features, status, err, \
    Size(31,31), 5, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 0.001), 0, 1e-4);

    undistortPoints(prev_features, prev_undistorted, M, distCoeffs, noArray(), M);
    undistortPoints(next_features, next_undistorted, M, distCoeffs, noArray(), M);
    // reject outliers one more time between the feature points on the current and the next images
    Mat good_features;
    findFundamentalMat(prev_undistorted, next_undistorted, CV_FM_RANSAC, 0.1, 0.999, good_features);
    // cout << good_features << endl;

    cout << "before refine: " << prev_undistorted.size() << endl;
    int baseline_threshold = 5;  // accepting only motion vectors whose baselines are greater than this threshold
    for(int i=0; i<prev_undistorted.size(); i++){
      if(good_features.at<bool>(i,0)==1){
        // calculate distance between previous feature point and next feature point to filter out short baseline
        float diff_x = abs(prev_undistorted.at(i).x - next_undistorted.at(i).x);
        float diff_y = abs(prev_undistorted.at(i).y - next_undistorted.at(i).y);
        float distance = diff_x + diff_y;
        if(distance > baseline_threshold /*&& distance < 200*/){
          prev_refined.push_back(prev_undistorted.at(i));
          next_refined.push_back(next_undistorted.at(i));
        }
      }
    }
    cout << "after refine: " << prev_refined.size() << endl;
    // if the number of matching points is less than threshold, propagate current transformation using previous transformation
    int matching_threshold = 8;
    if(prev_refined.size() > matching_threshold){
    Mat F = findFundamentalMat(prev_refined, next_refined, CV_FM_8POINT);
    // cout << "======= Fundamental Matrix =======" << endl;
    // cout << F << endl;
    // essential matrix
    Mat E = M.t()*F*M;
    Mat R,t;
    recoverPose(E, prev_refined, next_refined, R, t, M.at<double>(0,0), Point2d(M.at<double>(0,2), M.at<double>(1,2)));
    // cout << "======= R =======" << endl;
    // cout << R << endl;
    // cout << "======= t =======" << endl;
    // cout << t << endl;

    Mat temp, T;
    hconcat(R, t, temp);
    vconcat(temp, fourth_row, T);
    // cout << "======= T =======" << endl;
    // cout << T << endl;
    // cout << "======= E =======" << endl;
    // cout << E << endl;
    // cout << "======= F =======" << endl;
    // cout << F << endl;
    current_transformation = T.inv();
    total_transformation = total_transformation*current_transformation;
    }
    else{
      total_transformation = total_transformation*prev_transformation;
    }
    prev_transformation = current_transformation;

    // writing only tx, tz components to a file
    myfile << total_transformation.at<double>(0,0) << " " << total_transformation.at<double>(0,1) << " " \
           << total_transformation.at<double>(0,2) << " " << total_transformation.at<double>(0,3) << " " \
           << total_transformation.at<double>(1,0) << " " << total_transformation.at<double>(1,1) << " " \
           << total_transformation.at<double>(1,2) << " " << total_transformation.at<double>(1,3) << " " \
           << total_transformation.at<double>(2,0) << " " << total_transformation.at<double>(2,1) << " " \
           << total_transformation.at<double>(2,2) << " " << total_transformation.at<double>(2,3) << endl;

    draw_features(prev);
    imshow("opf", prev);
    waitKey(1);
  }

  while(waitKey(0)!=27);
  return 0;
}

void draw_features(Mat &src){
  for(int i=0; i<prev_refined.size(); i++){
    circle(src, prev_refined.at(i), 0.3, Scalar(0,255,0), 3);
    arrowedLine(src, prev_refined.at(i), next_refined.at(i), Scalar(0,0,255), 1);
  }
  return;
}
