#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

string header = "/home/magiccjae/jae_stuff/classes/ee631/hw7/images/T";
string ending = ".jpg";
int num_img = 18;

// parameters for calcOpticalFlowPyrLK
vector<Point2f> prev_features, next_features;
vector<uchar> status;
vector<float> err;

void draw_features(Mat &src);

int main(int, char**)
{
  Mat first = imread(header+to_string(1)+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);

  int max_corners = 100;
  double qlevel = 0.01;
  double min_distance = 10;
  // obtain initial set of features
  // int x_min = 200;
  // int x_max = 400;
  // int y_min = 150;
  // int y_max = 365;
  // Rect rec2(Point(x_min, y_min), Point(x_max, y_max));
  // Mat mask(first_gray.size(), CV_8UC1, Scalar::all(0));
  // mask(rec2).setTo(Scalar::all(255));
  // goodFeaturesToTrack(first_gray, next_features, max_corners, qlevel, min_distance, mask);
  goodFeaturesToTrack(first_gray, next_features, max_corners, qlevel, min_distance);

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

    // find expansion ratio 'a' - see lecture slide 25 page.3
    // Mat x_prime(prev_features.size()*2,1,CV_32FC1);
    // Mat x(prev_features.size()*2,1,CV_32FC1);
    Mat x_prime;
    Mat x;
    for(int j=0; j<prev_features.size(); j++){
      x.push_back(prev_features.at(j).x);
      x.push_back(prev_features.at(j).y);
      x_prime.push_back(next_features.at(j).x);
      x_prime.push_back(next_features.at(j).y);
      // x.at<float>(j,0) = prev_features.at(j).x;
      // x.at<float>(j+1,0) = prev_features.at(j).y;
      // x_prime.at<float>(j,0) = next_features.at(j).x;
      // x_prime.at<float>(j+1,0) = next_features.at(j).y;
    }
    // cout << x_prime << endl;
    // Mat x_inv;
    // invert(x, x_inv, DECOMP_SVD);
    Mat a_mat = x.inv(DECOMP_SVD)*x_prime;
    // cout << a_mat << endl;
    float a = a_mat.at<float>(0,0);
    float tau = a / (a-1);
    cout << tau << endl;
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
