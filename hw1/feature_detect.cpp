#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int main(int, char**)
{
  VideoCapture video(0); // open the default camera
  namedWindow("Lee", CV_WINDOW_AUTOSIZE);
  cout << "Press the following number to choose operation:" << endl;
  cout << "  1. Binarize" << endl;
  cout << "  2. Canny Edges" << endl;
  cout << "  3. Detect Corners" << endl;
  cout << "  4. Detect Lines" << endl;
  cout << "  5. Difference Image" << endl;
  cout << "Press 'Q' to Quit" << endl;
  int choice = 0;
  Mat src, src_gray, dst, dst_norm, dst_norm_scaled, cdst, prev_frame, temp;
  // for binarize
  int threshold_value = 128;
  int max_binary_value = 255;
  int threshold_type = 0;
  // for canny
  int lowThreshold = 30;
  int ratio = 3;
  int kernel_size = 3;
  // for corner
  int max_corners = 100;
  vector<Point2f> corners;
  double quality_level = 0.01;
  double min_distance = 10;
  int block_size = 3;
  bool use_harris_detector = false;
  double k = 0.04;
  Size win_size = Size(5,5);
  Size zero_zone = Size(-1,-1);
  TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
  // for line
  int threshold_line = 170;
  // for difference
  bool initial_flag = true;

  // video write
  video >> src;
  VideoWriter vout;
  int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
  double fps = 25.0;                          // framerate of the created video stream
  string filename = "./Lee_feature.avi";             // name of the output video file
  bool isColor = (src.type() == CV_8UC3);
  vout.open(filename, codec, fps, src.size(), isColor);

  while(1){
    video >> src;
    int key = waitKey(10);
    if(key != -1){
      choice = key;
    }
    if(choice==49){
      cvtColor(src, src_gray, CV_BGR2GRAY);
      threshold(src_gray, dst, threshold_value, max_binary_value, threshold_type);
      cvtColor(dst, dst, CV_GRAY2BGR);
      imshow("Lee", dst);
    }
    else if(choice==50){
      cvtColor(src, src_gray, CV_BGR2GRAY);
      blur(src_gray, src_gray, Size(3,3));
      Canny(src_gray, dst, lowThreshold, lowThreshold*ratio, kernel_size);
      cvtColor(dst, dst, CV_GRAY2BGR);
      imshow("Lee", dst);
    }
    else if(choice==51){
      cvtColor(src, src_gray, CV_BGR2GRAY);
      dst=src.clone();
      goodFeaturesToTrack(src_gray, corners, max_corners, quality_level, min_distance, Mat(), block_size, use_harris_detector, k);
      cornerSubPix( src_gray, corners, win_size, zero_zone, criteria );
      int r = 4;
      for(int i=0; i<corners.size(); i++){
        circle( dst, corners[i], r, Scalar(0,0,255), -1, 8, 0 );
      }
      imshow("Lee", dst);
    }
    else if(choice==52){
      src.copyTo(dst);
      cvtColor(dst, src_gray, CV_BGR2GRAY);
      blur(src_gray, src_gray, Size(3,3));
      Canny(src_gray, temp, lowThreshold, lowThreshold*ratio, kernel_size);
      // cvtColor(dst, cdst, CV_GRAY2BGR);
      vector<Vec2f> lines;
      HoughLines(temp, lines, 1, CV_PI/180, threshold_line, 0, 0);
      for( size_t i = 0; i < lines.size(); i++ ){
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line( dst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
      }
      imshow("Lee", dst);
    }
    else if(choice==53){
      if(initial_flag == true){
        src.copyTo(prev_frame);
        initial_flag = false;
      }
      absdiff(prev_frame, src, dst);
      src.copyTo(prev_frame);
      imshow("Lee", dst);
    }
    else if(choice==113){
      cout << "Terminating..." << endl;
      break;
    }
    else{
      src.copyTo(dst);
      imshow("Lee", dst);
    }
    vout.write(dst);
  }
  return 0;
}
