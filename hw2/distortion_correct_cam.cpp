#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

Mat src;

int main(int, char**)
{
  // read camera parameters and distortion coefficient from a file
  FileStorage fs("intrinsic_cam.xml", FileStorage::READ);
  Mat intrinsic, distCoeffs;
  fs["intrinsic"] >> intrinsic;
  fs["distCoeffs"] >> distCoeffs;

  cout << "intrinsic" << endl;
  cout << intrinsic << endl;
  cout << "distortion coefficient" << endl;
  cout << distCoeffs << endl;

  Mat src_turned = imread("../capture1.jpg");
  Mat undistorted_turned, difference_turned;
  // imshow("src_turned", src_turned);
  undistort(src_turned, undistorted_turned, intrinsic, distCoeffs);
  // imshow("undistorted_turned", undistorted_turned);
  absdiff(src_turned, undistorted_turned, difference_turned);
  imshow("difference_turned", difference_turned);

  Mat src_close = imread("../capture2.jpg");
  Mat undistorted_close, difference_close;
  // imshow("src_close", src_close);
  undistort(src_close, undistorted_close, intrinsic, distCoeffs);
  // imshow("undistorted_close", undistorted_close);
  absdiff(src_close, undistorted_close, difference_close);
  imshow("difference_close", difference_close);

  Mat src_far = imread("../capture3.jpg");
  Mat undistorted_far, difference_far;
  // imshow("src_far", src_far);
  undistort(src_far, undistorted_far, intrinsic, distCoeffs);
  // imshow("undistorted_far", undistorted_far);
  absdiff(src_far, undistorted_far, difference_far);
  imshow("difference_far", difference_far);

  while(waitKey(0)!=27);

  return 0;
}
