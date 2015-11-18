
#ifndef VIDEO_STAB_H
#define VIDEO_STAB_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs_c.h"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/features2d.hpp"

#include <iostream>
#include <stdio.h>

#include "opencv2/videostab/global_motion.hpp"
#include "opencv2/videostab.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/videostab/optical_flow.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"

using namespace cv;
using namespace std;
using namespace cv::videostab;

const int GRID_WIDTH = 5;
const int GRID_HEIGHT = 5;

extern "C" {
    Mat get_match_parallel(Mat &img1, Mat &img2, int index);
    int set_first_img(Mat &img, int index);
}

#endif
