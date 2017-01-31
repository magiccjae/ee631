#include "opencv2/opencv.hpp"
#include <string>

using namespace cv;
using namespace std;

string window_name = "src";
Mat src, gray;

int main(int, char**)
{
  string header = "/home/magiccjae/jae_stuff/classes/ee631/hw2/calibration_image/AR";
  string ending = ".jpg";
  namedWindow(window_name, CV_WINDOW_AUTOSIZE);
  Size patternsize(10,7); //interior number of corners
  vector<Point2f> corners; //this will be filled by the detected corners

  for(int i=1; i<=40; i++){
    cout << i << endl;
    src = imread(header+to_string(i)+ending);
    cvtColor(src,gray,CV_BGR2GRAY);
    bool patternfound = findChessboardCorners(gray, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    imshow(window_name,src);
    waitKey(0);
  }
  // while(waitKey(0)!=27);

  return 0;
}
