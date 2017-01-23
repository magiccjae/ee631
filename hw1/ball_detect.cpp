#include "opencv2/opencv.hpp"
#include <string>

using namespace cv;
using namespace std;

int threshold_type = 0;
int threshold_value = 3;
string window_name = "Lee";
string trackbar_value = "Value";
int const max_BINARY_value = 255;
Mat image1, image2, dst, result, src;

int morph_elem = 0;
int morph_size = 1;
int morph_operation = 2;  // morpholotical opening
Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

// void binarize(int, void*){
//   threshold(dst, result, threshold_value, max_BINARY_value, threshold_type);
//   imshow("result", result);
// }

int main(int, char**)
{
  string header = "/home/magiccjae/jae_stuff/classes/ee631/hw1/baseball/1";
  string ending = ".jpg";
  src = imread(header+"L"+to_string(5)+ending);
  namedWindow(window_name, CV_WINDOW_AUTOSIZE);
  // createTrackbar(trackbar_value, window_name, &threshold_value, max_BINARY_value, binarize);

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
  params.minCircularity = 0.4;
  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.3;
  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.1;
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

  // video write
  VideoWriter vout;
  int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
  double fps = 25.0;                          // framerate of the created video stream
  string filename = "./Lee.avi";             // name of the output video file
  bool isColor = (src.type() == CV_8UC3);
  vout.open(filename, codec, fps, src.size(), isColor);
  string path = "";
  for(int j=0; j<2; j++){
    if(j==0){
      path = header+"L";
    }
    else{
      path = header+"R";
    }
    for(int i=6; i<=40; i++){
      cout << i << endl;
      image1 = imread(path+to_string(5)+ending);
      image2 = imread(path+to_string(i)+ending);
      cvtColor(image1, image1, CV_BGR2GRAY);
      cvtColor(image2, image2, CV_BGR2GRAY);
      absdiff(image1, image2, dst);
      threshold(dst, result, threshold_value, max_BINARY_value, threshold_type);
      morphologyEx(result, result, morph_operation, element);

      vector<KeyPoint> keypoints;
      detector->detect(result,keypoints);
      cout << keypoints.size() << endl;

      // cvtColor(result,result,CV_GRAY2BGR);
      drawKeypoints(image2, keypoints, image2, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

      imshow("result",result);
      imshow("Lee",image2);
      vout.write(image2);
      waitKey(0);

    }
  }
  return 0;
}
