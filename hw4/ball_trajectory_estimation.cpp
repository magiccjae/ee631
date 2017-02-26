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
  // read camera parameters and distortion coefficient from a file
  FileStorage fs("everything.xml", FileStorage::READ);
  Mat intrinsic_left, dist_coeffs_left, intrinsic_right, dist_coeffs_right, R1, R2, P1, P2, Q;
  fs["intrinsic_left"] >> intrinsic_left;
  fs["distCoeffs_left"] >> dist_coeffs_left;
  fs["intrinsic_right"] >> intrinsic_right;
  fs["distCoeffs_right"] >> dist_coeffs_right;
  fs["R1"] >> R1;
  fs["R2"] >> R2;
  fs["P1"] >> P1;
  fs["P2"] >> P2;
  fs["Q"] >> Q;

  cout << "intrinsic_left" << endl;
  cout << intrinsic_left << endl;
  cout << "distortion coefficient left" << endl;
  cout << dist_coeffs_left << endl;

  string left_header = "/home/magiccjae/jae_stuff/classes/ee631/hw4/images/ballL";
  string right_header = "/home/magiccjae/jae_stuff/classes/ee631/hw4/images/ballR";
  string ending = ".bmp";
  Mat left_background = imread(left_header+to_string(1)+ending);
  Mat right_background = imread(right_header+to_string(1)+ending);
  cvtColor(left_background, left_background, CV_BGR2GRAY);
  cvtColor(right_background, right_background, CV_BGR2GRAY);

  // image process only in this region
  int left_x = 270;
  int left_y = 0;
  Rect left_rec(left_x, left_y, 230, 480);
  Mat left_roi_background = left_background(left_rec);

  // image process only in this region
  int right_x = 0;
  int right_y = 0;
  Rect right_rec(right_x, right_y, 320, 480);
  Mat right_roi_background = right_background(right_rec);\

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
  params.minCircularity = 0.5;
  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.3;
  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.3;
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

  Mat left_image, left_gray, left_result, left_roi;
  Mat right_image, right_gray, right_result, right_roi;
  for(int i=1; i<50; i++){
    left_image = imread(left_header+to_string(i)+ending);
    cvtColor(left_image, left_gray, CV_BGR2GRAY);
    left_roi = left_gray(left_rec);
    absdiff(left_roi_background, left_roi, left_result);
    threshold(left_result, left_result, threshold_value, max_BINARY_value, threshold_type);
    morphologyEx(left_result, left_result, morph_operation, element);

    right_image = imread(right_header+to_string(i)+ending);
    cvtColor(right_image, right_gray, CV_BGR2GRAY);
    right_roi = right_gray(right_rec);
    absdiff(right_roi_background, right_roi, right_result);
    threshold(right_result, right_result, threshold_value, max_BINARY_value, threshold_type);
    morphologyEx(right_result, right_result, morph_operation, element);

    vector<KeyPoint> keypoints_left;
    detector->detect(left_result, keypoints_left);
    // For left camera image, add left_x because ROI started from left_x in x coordinate.
    // Thus keypoints x coordinate is computed left_x in x coordinate as its 0 x coordinate.
    // So the code below compensates for that by adding left_x to keypoint's x coordinate.
    for(int i=0; i<keypoints_left.size(); i++){
      keypoints_left.at(i).pt.x = keypoints_left.at(i).pt.x + left_x;
    }
    vector<KeyPoint> keypoints_right;
    detector->detect(right_result, keypoints_right);

    drawKeypoints(left_image, keypoints_left, left_image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints(right_image, keypoints_right, right_image, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    if(keypoints_left.size()==1 && keypoints_right.size()==1){
      Point2f left_point(keypoints_left.at(0).pt.x, keypoints_left.at(0).pt.y);
      Point2f right_point(keypoints_right.at(0).pt.x, keypoints_right.at(0).pt.y);

      vector<Point2f> left_vector;
      left_vector.push_back(left_point);
      vector<Point2f> right_vector;
      right_vector.push_back(right_point);

      vector<Point2f> left_undistorted;
      vector<Point2f> right_undistorted;
      undistortPoints(left_vector, left_undistorted, intrinsic_left, dist_coeffs_left, R1, P1);
      undistortPoints(right_vector, right_undistorted, intrinsic_right, dist_coeffs_right, R2, P2);

      // compare Y coordinates to see if they are same.
      cout << "===== left =====" << endl;
      cout << left_undistorted << endl;
      cout << "===== right =====" << endl;
      cout << right_undistorted << endl;

      vector<Point3f> input_left;
      vector<Point3f> input_right;
      // augment disparity to make suitable inputs for perspectiveTransform function
      float disparity = left_undistorted.at(0).x - right_undistorted.at(0).x;
      Point3f a_point(left_undistorted.at(0).x, left_undistorted.at(0).y, disparity);
      Point3f another_point(right_undistorted.at(0).x, right_undistorted.at(0).y, disparity);
      input_left.push_back(a_point);
      input_right.push_back(another_point);

      // these vectors will contain 3D information.
      vector<Point3f> left_3d;
      vector<Point3f> right_3d;
      perspectiveTransform(input_left, left_3d, Q);
      perspectiveTransform(input_right, right_3d, Q);
      cout << "===== left 3D=====" << endl;
      cout << left_3d << endl;
      cout << "===== right 3D=====" << endl;
      cout << right_3d << endl;
    }

    imshow("left", left_image);
    imshow("right", right_image);
    waitKey(0);
  }

  while(waitKey(0)!=27);


  return 0;
}
