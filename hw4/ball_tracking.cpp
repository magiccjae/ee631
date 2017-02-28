#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

// for thresholding
int threshold_type = 0;
int threshold_value = 10;
int const max_BINARY_value = 255;

// for morphological operation
int morph_elem = 0;
int morph_size = 2;
int morph_operation = 2;  // morpholotical opening
Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

int main(int, char**)
{
  string left_header = "/home/magiccjae/jae_stuff/classes/ee631/hw4/images/Ball_testL";
  string right_header = "/home/magiccjae/jae_stuff/classes/ee631/hw4/images/Ball_testR";
  string ending = ".bmp";
  Mat left_background = imread(left_header+to_string(1)+ending);
  Mat right_background = imread(right_header+to_string(1)+ending);
  cvtColor(left_background, left_background, CV_BGR2GRAY);
  cvtColor(right_background, right_background, CV_BGR2GRAY);

  // image process only in this region
  int left_x = 320;
  int left_y = 0;
  Rect left_rec(left_x, left_y, 320, 480);
  Mat left_roi_background = left_background(left_rec);

  // parameters for SimpleBlobDetector`
  SimpleBlobDetector::Params params;
  // Change thresholds
  params.minThreshold = 100;
  params.maxThreshold = 120;
  params.filterByColor = true;
  params.blobColor = 255;
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 100;
  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.3;
  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.1;
  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.1;
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

  Mat left_image, left_gray, left_result, left_roi;
  for(int i=1; i<50; i++){
    cout << i << endl;
    left_image = imread(left_header+to_string(i)+ending);
    cvtColor(left_image, left_gray, CV_BGR2GRAY);
    left_roi = left_gray(left_rec);
    absdiff(left_roi_background, left_roi, left_result);
    threshold(left_result, left_result, threshold_value, max_BINARY_value, threshold_type);
    morphologyEx(left_result, left_result, morph_operation, element);
    vector<KeyPoint> keypoints;
    detector->detect(left_result, keypoints);
    for(int i=0; i<keypoints.size(); i++){
      keypoints.at(i).pt.x = keypoints.at(i).pt.x + left_x;
    }
    drawKeypoints(left_image, keypoints, left_image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imshow("left", left_image);
    waitKey(0);
  }

  // image process only in this region
  int right_x = 0;
  int right_y = 0;
  Rect right_rec(right_x, right_y, 320, 480);
  Mat right_roi_background = right_background(right_rec);\

  Mat right_image, right_gray, right_result, right_roi;
  for(int i=1; i<50; i++){
    cout << i << endl;
    right_image = imread(right_header+to_string(i)+ending);
    cvtColor(right_image, right_gray, CV_BGR2GRAY);
    right_roi = right_gray(right_rec);
    absdiff(right_roi_background, right_roi, right_result);
    threshold(right_result, right_result, threshold_value, max_BINARY_value, threshold_type);
    morphologyEx(right_result, right_result, morph_operation, element);

    vector<KeyPoint> keypoints;
    detector->detect(right_result, keypoints);
    drawKeypoints(right_image, keypoints, right_image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imshow("right", right_image);
    waitKey(0);
  }

  while(waitKey(0)!=27);

  return 0;
}
