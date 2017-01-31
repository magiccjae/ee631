#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int main(int, char**)
{
  // read camera parameters and distortion coefficient from a file
  FileStorage fs("intrinsic.xml", FileStorage::READ);
  Mat intrinsic, distCoeffs;
  fs["intrinsic"] >> intrinsic;
  fs["distCoeffs"] >> distCoeffs;

  cout << "intrinsic" << endl;
  cout << intrinsic << endl;
  cout << "distortion coefficient" << endl;
  cout << distCoeffs << endl;

  Mat src = imread("../Turned.jpg");
  Mat undistorted, difference;
  imshow("src", src);

  undistort(src, undistorted, intrinsic, distCoeffs);
  imshow("undistorted", undistorted);

  absdiff(src, undistorted, difference);
  imshow("difference", difference);

  while(waitKey(0)!=27);

  return 0;
}
