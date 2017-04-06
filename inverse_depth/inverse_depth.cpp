#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

Mat src, gray, prev, prev_gray;
Mat intrinsic = Mat(3, 3, CV_32FC1);
Mat distCoeffs;

// parameters for goodFeaturesToTrack
int max_corners = 20;
double qlevel = 0.01;
double min_distance = 10;

// parameters for calcOpticalFlowPyrLK
vector<Point2f> features_prev, features_next;
vector<uchar> status;
vector<float> err;

void draw_features(Mat &src);

int main(int, char**)
{
  // write to a XML or YAML file
  FileStorage fs("intrinsic_cam.xml",FileStorage::READ);
  // read camera intrinsic
  fs["intrinsic"] >> intrinsic;
  fs["distCoeffs"] >> distCoeffs;
  fs.release();
  cout << intrinsic << endl;
  cout << distCoeffs << endl;

  VideoCapture video(0); // open the default camera
  video >> src;
  cout << "image size" << src.size() << endl;
  int x_size = src.size().width;
  int y_size = src.size().height;

  bool initialized = false;

  while(1){
    video >> src;
    cvtColor(src, gray, CV_RGB2GRAY);
    // press 'q' to initialize features to estimate depth
    // press 'w' to discard the initial features
    int key = waitKey(10);
    if(key==113){
      // obtain initial set of features
      int mask_size = 100;
      Rect rec(Point(mask_size, mask_size), Point(x_size-mask_size, y_size-mask_size));
      Mat mask(gray.size(), CV_8UC1, Scalar::all(0));
      mask(rec).setTo(Scalar::all(255));
      goodFeaturesToTrack(gray, features_prev, max_corners, qlevel, min_distance, mask);
      prev_gray = gray.clone();
      initialized = true;
    }
    else if(key==119){
      initialized = false;
    }

    // if initialized, find motion field.
    if(initialized){
      calcOpticalFlowPyrLK(prev_gray, gray, features_prev, features_next, status, err, \
      Size(21,21), 5, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 0.001), 0, 1e-4);

      // undistort points


      draw_features(src);
      imshow("src", src);
      features_prev = features_next;
      prev_gray = gray.clone();
    }
    else{
      imshow("src", src);
    }

    if(waitKey(10)==27) break;
  }

  return 0;
}

void draw_features(Mat &src){
  for(int i=0; i<features_next.size(); i++){
    circle(src, features_next.at(i), 0.3, Scalar(0,255,0), 3);
    arrowedLine(src, features_prev.at(i), features_next.at(i), Scalar(0,0,255), 1);
  }
  return;
}
