#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main(int, char**)
{
  // read camera parameters and distortion coefficient from a file
  FileStorage fs("intrinsic.xml", FileStorage::READ);
  Mat intrinsic, distCoeffs;
  fs["intrinsic"] >> intrinsic;
  fs["distCoeffs"] >> distCoeffs;

  cout << "intrinsic" << endl;
  cout << intrinsic << endl;
  cout << "distortion coefficient" << endl;
  cout << distCoeffs << endl;

  string line;
  ifstream myfile("../Data Points.txt");

  vector<Point3f> object_points;   // how many points are on the chessboard. In this case 7x10 (7 rows 10 columns)
  vector<Point2f> corners;   //this will be filled by the detected corners
  Mat rvecs;
  Mat tvecs;

  for(int i=0; i<2; i++){
    if(i==0){
      cout << "image points" << endl;
      for(int j=0; j<20; j++){
        getline(myfile,line);
        istringstream iss(line);
        float x;
        float y;
        iss >> x;
        iss >> y;
        cout << x << " " << y << endl;
        corners.push_back(Point2f(x, y));
      }
    }
    else if(i==1){
      cout << "object points" << endl;
      for(int j=0; j<20; j++){
        getline(myfile,line);
        istringstream iss(line);
        float x;
        float y;
        float z;
        iss >> x;
        iss >> y;
        iss >> z;
        cout << x << " " << y << " " << z <<endl;
        object_points.push_back(Point3f(x, y, z));
      }
    }
  }
  myfile.close();

  solvePnP(object_points, corners, intrinsic, distCoeffs, rvecs, tvecs);
  Mat rvecs_euler;
  Rodrigues(rvecs, rvecs_euler);
  cout << "rotation" << endl;
  cout << rvecs_euler << endl;
  cout << "translation" << endl;
  cout << tvecs << endl;

  return 0;
}
