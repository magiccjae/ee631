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
  Mat intrinsic_right, dist_coeffis_right;
  fs_right["intrinsic"] >> intrinsic_right;
  fs_right["distCoeffs"] >> dist_coeffis_right;

  cout << "intrinsic_right" << endl;
  cout << intrinsic_right << endl;
  cout << "distortion coefficient right" << endl;
  cout << dist_coeffis_right << endl;

  



  // while(waitKey(0)!=27);

  return 0;
}
