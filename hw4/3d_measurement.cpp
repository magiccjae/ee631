#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

vector<Point2f> get_outer_points(vector<Point2f> corners, int width, int height);

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

  string left_header = "/home/magiccjae/jae_stuff/classes/ee631/hw4/stereoL";
  string ending = ".bmp";
  int num = 21;
  Mat left_image = imread(left_header+to_string(num)+ending);
  Size image_size = left_image.size();
  cout << "image size" << left_image.size() << endl;

  string right_header = "/home/magiccjae/jae_stuff/classes/ee631/hw4/stereoR";
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

  Mat left_gray, right_gray;
  Size patternsize(10,7); //interior number of corners
  vector<Point2f> left_corners;   //this will be filled by the detected corners
  vector<Point2f> right_corners;   //this will be filled by the detected corners

  cvtColor(left_image, left_gray, CV_BGR2GRAY);
  cvtColor(right_image, right_gray, CV_BGR2GRAY);

  bool patternfound = findChessboardCorners(left_gray, patternsize, left_corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
  cornerSubPix(left_gray, left_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.01));
  bool patternfound1 = findChessboardCorners(right_gray, patternsize, right_corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
  cornerSubPix(right_gray, right_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.01));

  vector<Point2f> four_left;
  vector<Point2f> four_right;
  // a function to find four outermost corners of the chess board pattern
  four_left = get_outer_points(left_corners, patternsize.width, patternsize.height);
  four_right = get_outer_points(right_corners, patternsize.width, patternsize.height);

  // draw four corners on images just to check
  circle(left_image, four_left.at(0), 3, Scalar(0,255,0), 3, 8, 0);
  circle(left_image, four_left.at(1), 3, Scalar(0,255,0), 3, 8, 0);
  circle(left_image, four_left.at(2), 3, Scalar(0,255,0), 3, 8, 0);
  circle(left_image, four_left.at(3), 3, Scalar(0,255,0), 3, 8, 0);
  circle(right_image, four_right.at(0), 3, Scalar(0,255,0), 3, 8, 0);
  circle(right_image, four_right.at(1), 3, Scalar(0,255,0), 3, 8, 0);
  circle(right_image, four_right.at(2), 3, Scalar(0,255,0), 3, 8, 0);
  circle(right_image, four_right.at(3), 3, Scalar(0,255,0), 3, 8, 0);

  imshow("left_image", left_image);
  imshow("right_image", right_image);

  // four_ideal is the four outermost points of the chess board undistorted and rectified
  vector<Point2f> four_ideal_left;
  vector<Point2f> four_ideal_right;
  undistortPoints(four_left, four_ideal_left, intrinsic_left, dist_coeffs_left, R1, P1);
  undistortPoints(four_right, four_ideal_right, intrinsic_right, dist_coeffs_right, R2, P2);

    #####  #######    #    ######  #######    #     # ####### ######  #######
   #     #    #      # #   #     #    #       #     # #       #     # #
   #          #     #   #  #     #    #       #     # #       #     # #
    #####     #    #     # ######     #       ####### #####   ######  #####
         #    #    ####### #   #      #       #     # #       #   #   #
   #     #    #    #     # #    #     #       #     # #       #    #  #
    #####     #    #     # #     #    #       #     # ####### #     # #######
   // I need to use perspectiveTransoform(). I am not sure about the inputs to the function.

  cout << "===== left =====" << endl;
  cout << four_ideal_left << endl;
  cout << "===== right =====" << endl;
  cout << four_ideal_right << endl;

  while(waitKey(0)!=27);

  return 0;
}

vector<Point2f> get_outer_points(vector<Point2f> corners, int width, int height){
  Point2f first = corners.at(0);
  Point2f second = corners.at(width-1);
  Point2f third = corners.at(width*height-1);
  Point2f fourth = corners.at(width*height-width);
  vector<Point2f> four_points;
  four_points.push_back(first);
  four_points.push_back(second);
  four_points.push_back(third);
  four_points.push_back(fourth);
  return four_points;
}
