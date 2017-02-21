#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

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
  Mat rotation, translation;
  fs["rotation"] >> rotation;
  fs["translation"] >> translation;
  cout << "rotation" << endl;
  cout << rotation << endl;
  cout << "translation" << endl;
  cout << translation << endl;

  string left_header = "/home/magiccjae/jae_stuff/classes/ee631/hw3/images/ImageJae/stereoL";
  string ending = ".bmp";
  int num = 3;
  Mat left_image = imread(left_header+to_string(num)+ending);
  Size image_size = left_image.size();
  cout << "image size" << left_image.size() << endl;

  string right_header = "/home/magiccjae/jae_stuff/classes/ee631/hw3/images/ImageJae/stereoR";
  Mat right_image = imread(right_header+to_string(num)+ending);

  Mat R1, R2, P1, P2, Q;
  stereoRectify(intrinsic_left, dist_coeffs_left, intrinsic_right, dist_coeffs_right, \
    image_size, rotation, translation, R1, R2, P1, P2, Q, 0, -1, image_size, 0, 0);
  cout << "===== R1 =====" << endl;
  cout << R1 << endl;
  cout << "===== R2 =====" << endl;
  cout << R2 << endl;
  cout << "===== P1 =====" << endl;
  cout << P1 << endl;
  cout << "===== P2 =====" << endl;
  cout << P2 << endl;
  cout << "===== Q =====" << endl;
  cout << Q << endl;

  Mat map1, map2;
  Mat map3, map4;
  initUndistortRectifyMap(intrinsic_left, dist_coeffs_left, R1, P1, image_size, CV_32FC1, map1, map2);
  initUndistortRectifyMap(intrinsic_right, dist_coeffs_right, R2, P2, image_size, CV_32FC1, map3, map4);

  Mat left_rectified, right_rectified;
  remap(left_image, left_rectified, map1, map2, INTER_LINEAR, BORDER_CONSTANT, 0);
  remap(right_image, right_rectified, map3, map4, INTER_LINEAR, BORDER_CONSTANT, 0);

  // compute difference images
  Mat left_diff, right_diff;
  absdiff(left_image, left_rectified, left_diff);
  absdiff(right_image, right_rectified, right_diff);

  // draw horizontal lines
  int col = left_rectified.cols;
  for(int i=1; i<9; i++){
    int row = i*50;
    line(left_rectified, Point(0,row), Point(col,row), Scalar(0, 0, 255), 1, 8, 0);
    line(right_rectified, Point(0,row), Point(col,row), Scalar(0, 0, 255), 1, 8, 0);
  }
  imshow("left_rectified", left_rectified);
  imshow("right_rectified", right_rectified);

  imshow("left_diff", left_diff);
  imshow("right_diff", right_diff);


  while(waitKey(0)!=27);

  return 0;
}
