#include "opencv2/opencv.hpp"
#include <string>

using namespace cv;
using namespace std;

string window = "src";
Mat src, gray;

int main(int, char**)
{
  namedWindow(window,CV_WINDOW_AUTOSIZE);
  src = imread("../AR40.jpg");
  cvtColor(src,gray,CV_BGR2GRAY);

  Size patternsize(10,7); //interior number of corners
  vector<Point2f> corners; //this will be filled by the detected corners

  bool patternfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
  if(patternfound){
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  }
  drawChessboardCorners(src, patternsize, Mat(corners), patternfound);

  imshow(window,src);

  while(waitKey(0)!=27);

  return 0;
}
