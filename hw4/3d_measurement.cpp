#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <math.h>   // for rounding float numbers to less decimal places

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


  // four_ideal is the four outermost points of the chess board undistorted and rectified
  vector<Point2f> four_ideal_left;
  vector<Point2f> four_ideal_right;
  undistortPoints(four_left, four_ideal_left, intrinsic_left, dist_coeffs_left, R1, P1);
  undistortPoints(four_right, four_ideal_right, intrinsic_right, dist_coeffs_right, R2, P2);

  // compare Y coordinates to see if they are same.
  cout << "===== left =====" << endl;
  cout << four_ideal_left << endl;
  cout << "===== right =====" << endl;
  cout << four_ideal_right << endl;

  vector<Point3f> four_input_left;
  vector<Point3f> four_input_right;
  // augment disparity to make suitable inputs for perspectiveTransform function
  for(int i=0; i<4; i++){
    float disparity = four_ideal_left.at(i).x - four_ideal_right.at(i).x;
    Point3f a_point(four_ideal_left.at(i).x, four_ideal_left.at(i).y, disparity);
    Point3f another_point(four_ideal_right.at(i).x, four_ideal_right.at(i).y, disparity);
    four_input_left.push_back(a_point);
    four_input_right.push_back(another_point);
  }

  // these vectors will contain 3D information.
  vector<Point3f> four_3d_left;
  vector<Point3f> four_3d_right;
  perspectiveTransform(four_input_left, four_3d_left, Q);
  perspectiveTransform(four_input_right, four_3d_right, Q);
  cout << "===== left 3D=====" << endl;
  cout << four_3d_left << endl;
  cout << "===== right 3D=====" << endl;
  cout << four_3d_right << endl;

  // adding 3D coordinates to images
  for(int i=0; i<4; i++){
    float temp_x = round(four_3d_left.at(i).x*100)/100.0;
    float temp_y = round(four_3d_left.at(i).y*100)/100.0;
    float temp_z = round(four_3d_left.at(i).z*100)/100.0;
    string text_x = to_string(temp_x);
    string text_y = to_string(temp_y);
    string text_z = to_string(temp_z);
    string text = "(" + text_x + ", " + text_y + ", " + text_z + ")";
    Point text_point = four_ideal_left.at(i);
    putText(left_image, text, Point(text_point.x-200, text_point.y), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);

    float temp_x1 = round(four_3d_right.at(i).x*100)/100.0;
    float temp_y1 = round(four_3d_right.at(i).y*100)/100.0;
    float temp_z1 = round(four_3d_right.at(i).z*100)/100.0;
    string text_x1 = to_string(temp_x1);
    string text_y1 = to_string(temp_y1);
    string text_z1 = to_string(temp_z1);
    string text1 = "(" + text_x1 + ", " + text_y1 + ", " + text_z1 + ")";
    Point text_point1 = four_ideal_right.at(i);
    putText(right_image, text1, Point(text_point1.x, text_point1.y), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(0,0,255), 1.7, 8, false);
  }

  imshow("left_image", left_image);
  imshow("right_image", right_image);

  while(waitKey(0)!=27);

  // write all matrices into a .xml file
  FileStorage fs_write("everything.xml", FileStorage::WRITE);
  fs_write << "intrinsic_left" << intrinsic_left;
  fs_write << "distCoeffs_left" << dist_coeffs_left;
  fs_write << "intrinsic_right" << intrinsic_right;
  fs_write << "distCoeffs_right" << dist_coeffs_right;
  fs_write << "R1" << R1;
  fs_write << "R2" << R2;
  fs_write << "P1" << P1;
  fs_write << "P2" << P2;
  fs_write << "Q" << Q;
  fs_write.release();

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
