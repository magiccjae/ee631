#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

Mat left_image, right_image;
string left_window = "left";
string right_window = "right";

vector<Point2f> left_points;
vector<Point2f> right_points;
Mat undistorted_left, undistorted_right;
Mat fundamental;
Mat lines_left, lines_right;

// mouse event for picking points
void left_callback(int event, int x, int y, int flags, void* userdata){
  if(event==EVENT_LBUTTONDOWN){
    if(left_points.size()<3){
      Point2f point(x, y);
      left_points.push_back(point);
      cout << x << ", " << y << " saved in left_points" << endl;
      circle(undistorted_left, point, 1, Scalar(0,255,0), 3, 8, 0);
      imshow(left_window, undistorted_left);

      // compute epipolar line
      computeCorrespondEpilines(left_points, 1, fundamental, lines_left);
      int index = left_points.size()-1;
      cout << lines_left << endl;
      float a = lines_left.at<float>(index,0);
      float b = lines_left.at<float>(index,1);
      float c = lines_left.at<float>(index,2);
      int max_x = undistorted_right.cols;
      // draw epipolar line corresponding to the point selected
      line(undistorted_right, Point(0,-c/b), Point(max_x,-(a*max_x+c)/b), Scalar(0,0,255), 1, 8, 0);
      imshow(right_window, undistorted_right);
    }
  }
}
void right_callback(int event, int x, int y, int flags, void* userdata){
  if(event==EVENT_LBUTTONDOWN){
    if(right_points.size()<3){
      Point2f point(x, y);
      right_points.push_back(point);
      cout << x << " " << y << " saved in right_points" << endl;
      circle(undistorted_right, point, 1, Scalar(0,255,0), 3, 8, 0);
      imshow(right_window, undistorted_right);

      // compute epipolar line
      computeCorrespondEpilines(right_points, 2, fundamental, lines_right);
      int index = right_points.size()-1;
      cout << lines_right << endl;
      float a = lines_right.at<float>(index,0);
      float b = lines_right.at<float>(index,1);
      float c = lines_right.at<float>(index,2);
      int max_x = undistorted_left.cols;
      // draw epipolar line corresponding to the point selected
      line(undistorted_left, Point(0,-c/b), Point(max_x,-(a*max_x+c)/b), Scalar(0,0,255), 1, 8, 0);
      imshow(left_window, undistorted_left);
    }
  }
}

int main(int, char**)
{
  // read camera parameters and distortion coefficient from a file
  FileStorage fs_left("intrinsic_left.xml", FileStorage::READ);
  Mat intrinsic_left, dist_coeffs_left;
  fs_left["intrinsic"] >> intrinsic_left;
  fs_left["distCoeffs"] >> dist_coeffs_left;
  cout << "intrinsic_left" << endl;
  cout << intrinsic_left << endl;
  cout << "distortion coefficient left" << endl;
  cout << dist_coeffs_left << endl;

  FileStorage fs_right("intrinsic_right.xml", FileStorage::READ);
  Mat intrinsic_right, dist_coeffs_right;
  fs_right["intrinsic"] >> intrinsic_right;
  fs_right["distCoeffs"] >> dist_coeffs_right;
  cout << "intrinsic_right" << endl;
  cout << intrinsic_right << endl;
  cout << "distortion coefficient right" << endl;
  cout << dist_coeffs_right << endl;

  // read fundamental matrix from a file
  FileStorage fs("fundamental.xml", FileStorage::READ);
  fs["fundamental"] >> fundamental;
  cout << "fundamental" << endl;
  cout << fundamental << endl;

  // read images
  string left_name = "/home/magiccjae/jae_stuff/classes/ee631/hw3/stereo/StereoL0.bmp";
  string right_name = "/home/magiccjae/jae_stuff/classes/ee631/hw3/stereo/StereoR0.bmp";
  left_image = imread(left_name);
  right_image = imread(right_name);
  // correct distortion for left and right images
  undistort(left_image, undistorted_left, intrinsic_left, dist_coeffs_left);
  undistort(right_image, undistorted_right, intrinsic_right, dist_coeffs_right);

  // displays images
  namedWindow(left_window, CV_WINDOW_AUTOSIZE);
  namedWindow(right_window, CV_WINDOW_AUTOSIZE);
  setMouseCallback(left_window, left_callback, NULL);
  setMouseCallback(right_window, right_callback, NULL);
  imshow(left_window, undistorted_left);
  imshow(right_window, undistorted_right);

  while(waitKey(0)!=27);

  return 0;
}
