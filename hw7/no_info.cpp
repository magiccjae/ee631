#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

string header = "/home/magiccjae/jae_stuff/classes/ee631/hw7/images/T";
string ending = ".jpg";
int thresh = 150;

int main(int, char**)
{
  Mat first = imread(header+to_string(1)+ending);
  Mat first_gray;
  cvtColor(first, first_gray, CV_RGB2GRAY);

  

  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( first.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( first_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ ){
     for( int i = 0; i < dst_norm.cols; i++ ){
        if( (int) dst_norm.at<float>(j,i) > thresh ){
           circle( first, Point( i, j ), 5,  Scalar(0,0,255), 2, 8, 0 );
        }
      }
  }

  imshow("first", first);
  // imshow("dst", dst);

  while(waitKey(0)!=27);
  return 0;

}
