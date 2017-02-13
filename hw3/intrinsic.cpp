#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

string window_name = "src";
Mat src, gray;

int main(int argc, char* argv[])
{
  cout << "Usage: \"./intrinsic (left) or (right)\"" << endl;
  string l_or_r = argv[1];
  string filename = "intrinsic_" + l_or_r + ".xml";
  cout << filename << endl;
  // write to a XML or YAML file
  FileStorage fs(filename, FileStorage::WRITE);
  string header = "";
  if(l_or_r == "left"){
    header = "/home/magiccjae/a_jae_stuff/classes/ee631/hw3/left_camera/CameraL";
  }
  else if(l_or_r == "right"){
    header = "/home/magiccjae/a_jae_stuff/classes/ee631/hw3/right_camera/CameraR";
  }

  string ending = ".bmp";
  namedWindow(window_name, CV_WINDOW_AUTOSIZE);
  src = imread(header+to_string(1)+ending);
  cout << "image size" << src.size() << endl;
  Size patternsize(10,7); //interior number of corners
  vector<vector<Point3f>> object_points;    // This vector contains the same number of object points as the number of images with patterns. In this case 40
  vector<Point3f> object_point;   // how many points are on the chessboard. In this case 7x10 (7 rows 10 columns)
  vector<vector<Point2f>> image_points;   // This vector contains the same number of object points as the number of images with patterns. In this case 40
  vector<Point2f> corners;   //this will be filled by the detected corners

  for(int i=0; i<patternsize.height; i++){
    for(int j=0; j<patternsize.width; j++){
      object_point.push_back(Point3f(j,i,0.0f));
    }
  }

  for(int i=0; i<=31; i++){
    cout << i << endl;
    object_points.push_back(object_point);

    src = imread(header+to_string(i)+ending);
    cvtColor(src,gray,CV_BGR2GRAY);
    bool patternfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    // cout << corners << endl;
    drawChessboardCorners(src, patternsize, Mat(corners), patternfound);

    image_points.push_back(corners);

    imshow(window_name,src);
    waitKey(0);
  }
  // These are variables that will hold the camera intrinsic matrix, distortion coefficient, rotation matrix, translation matrix.
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;

  calibrateCamera(object_points, image_points, src.size(), intrinsic, distCoeffs, rvecs, tvecs);
  cout << "intrinsic" << endl;
  cout << intrinsic << endl;
  cout << "distortion coefficient" << endl;
  cout << distCoeffs << endl;

  // write data into a file
  fs << "intrinsic" << intrinsic;
  fs << "distCoeffs" << distCoeffs;
  fs.release();

  return 0;
}
