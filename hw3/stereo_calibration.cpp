#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

string window_name = "src";
Mat src, gray;

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

  string left_header = "/home/magiccjae/jae_stuff/classes/ee631/hw3/stereo/StereoL";
  string right_header = "/home/magiccjae/jae_stuff/classes/ee631/hw3/stereo/StereoR";
  string ending = ".bmp";
  namedWindow(window_name, CV_WINDOW_AUTOSIZE);
  src = imread(left_header+to_string(0)+ending);
  cout << "image size" << src.size() << endl;
  Size patternsize(10,7); //interior number of corners
  vector<vector<Point3f>> object_points;    // This vector contains the same number of object points as the number of images with patterns.
  vector<Point3f> object_point;   // how many points are on the chessboard. In this case 7x10 (7 rows 10 columns)
  vector<vector<Point2f>> image_points_left;   // This vector contains the same number of object points as the number of images with patterns.
  vector<vector<Point2f>> image_points_right;   // This vector contains the same number of object points as the number of images with patterns.

  // creating a object point vector. 3.88 is the size of a square on the chessboard
  for(int i=0; i<patternsize.height; i++){
    for(int j=0; j<patternsize.width; j++){
      object_point.push_back(Point3f(3.88*j,3.88*i,0.0f));
    }
  }

  for(int i=0; i<=31; i++){
    cout << i << endl;
    object_points.push_back(object_point);
    src = imread(left_header+to_string(i)+ending);
    cvtColor(src,gray,CV_BGR2GRAY);

    vector<Point2f> corners;   // This will be filled by the detected corners
    bool patternfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.01));
    drawChessboardCorners(src, patternsize, Mat(corners), patternfound);
    image_points_left.push_back(corners);

    imshow(window_name,src);
    waitKey(0);
  }

  for(int i=0; i<=31; i++){
    cout << i << endl;
    src = imread(right_header+to_string(i)+ending);
    cvtColor(src,gray,CV_BGR2GRAY);

    vector<Point2f> corners;   // This will be filled by the detected corners
    bool patternfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.01));
    drawChessboardCorners(src, patternsize, Mat(corners), patternfound);
    image_points_right.push_back(corners);

    imshow(window_name,src);
    waitKey(0);
  }

  // stereoCalibrate output place holders
  Mat rotation;
  Mat translation;
  Mat essential;
  Mat fundamental;

  stereoCalibrate(object_points, image_points_left, image_points_right, intrinsic_left, dist_coeffs_left, \
                  intrinsic_right, dist_coeffs_right, src.size(), rotation, translation, essential, fundamental, \
                  CV_CALIB_FIX_INTRINSIC, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 50, 0.01));

  cout << "===== R =====" << endl;
  cout << rotation << endl;
  cout << "===== T =====" << endl;
  cout << translation << endl;
  cout << "===== E =====" << endl;
  cout << essential << endl;
  cout << "===== F =====" << endl;
  cout << fundamental << endl;

  FileStorage fs("fundamental.xml", FileStorage::WRITE);
  fs <<"fundamental" << fundamental;
  fs << "rotation" << rotation;
  fs << "translation" << translation;
  fs << "essential" << essential;

  fs.release();
  // while(waitKey(0)!=27);

  return 0;
}
