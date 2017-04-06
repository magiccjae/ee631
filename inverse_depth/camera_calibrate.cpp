#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

Mat src, gray;

int main(int, char**)
{
  // write to a XML or YAML file
  FileStorage fs("intrinsic_cam.xml",FileStorage::WRITE);

  VideoCapture video(0); // open the default camera
  video >> src;
  cout << "image size" << src.size() << endl;

  vector<vector<Point2f>> image_points;   // This vector contains the same number of object points as the number of images with patterns.
  vector<Point2f> corners;   //this will be filled by the detected corners
  Size patternsize(9,7); //interior number of corners
  vector<vector<Point3f>> object_points;    // This vector contains the same number of object points as the number of images with patterns.
  vector<Point3f> object_point;   // how many points are on the chessboard.
  for(int i=0; i<patternsize.height; i++){
    for(int j=0; j<patternsize.width; j++){
      object_point.push_back(Point3f(j,i,0.0f));
    }
  }

  int image_collected = 0;
  int image_total = 50;
  while(image_collected < image_total){
    video >> src;
    imshow("src", src);
    int key = waitKey(10);
    // press 'q' to capture image
    if(key==113){
      cvtColor(src,gray,CV_BGR2GRAY);
      bool patternfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
      if(patternfound){
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        drawChessboardCorners(src, patternsize, Mat(corners), patternfound);
        image_points.push_back(corners);

        object_points.push_back(object_point);
        imshow("corners",src);
        waitKey(0);
        image_collected++;
        cout << image_collected << " images collected so far" << endl;
      }
    }
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
