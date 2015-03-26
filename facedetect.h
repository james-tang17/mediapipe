#ifndef __FACE_DETECT_H__
#define __FACE_DETECT_H__

#include <vector>
#include <string>

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>

#include <opencv2/videoio/videoio_c.h>
#include <opencv2/highgui/highgui_c.h>

int fd_init(const std::string& cascadename);
int fd_detect_cpu(cv::Mat& img, std::vector<cv::Rect>& faces);
int fd_detect_gpu(cv::Mat& img, std::vector<cv::Rect>& faces);
int fd_detect_withskin(cv::Mat& img, std::vector<cv::Rect>& faces);

#endif
