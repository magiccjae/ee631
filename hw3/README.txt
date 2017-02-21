This project includes left, right camera calibration, stereo calibration, drawing epipolar lines, and image rectification.

intrinsic.cpp
Set file path to the path that your images are stored.
For execution, you have to set left or right camera.
./intrinsic left
or
./intrinsic right
This program generates "intrinsic_left.xml" or "intrinsic_right.xml" files
that include intrinsic parameters and distortion coefficients of the camera.


stereo_calibration.cpp
This program reads in intrinsic parameters and distortion coefficients from .xml files
and also reads in pairs of images taken from stereo camera. Set file path to your stereo images are stored.

epipolar_lines.cpp
This program reads in fundamental matrix from stereo calibration and draw epipolar lines on a pair of stereo images.

rectification.cpp
This program rectifies images taken from a stereo camera and align images taken from each camera horizontally. This makes general configuration for a stereo camera become canonical configuration.
