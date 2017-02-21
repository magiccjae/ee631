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

  string image_header = "/home/magiccjae/jae_stuff/classes/ee631/hw3/stereo/StereoL";
  string ending = ".bmp";
  Mat src = imread(image_header+to_string(0)+ending);
  cout << "image size" << src.size() << endl;

  Mat R1, R2, P1, P2, Q;
  stereoRectify(intrinsic_left, dist_coeffs_left, intrinsic_right, dist_coeffs_right, \
    src.size(), rotation, translation, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, src.size(), 0, 0);
  cout << "R1" << endl;
  cout << R1 << endl;
  cout << "R2" << endl;
  cout << R2 << endl;
  cout << "P1" << endl;
  cout << P1 << endl;
  cout << "P2" << endl;
  cout << P2 << endl;
  cout << "Q" << endl;
  cout << Q << endl;
  

  while(waitKey(0)!=27);

  return 0;
}
