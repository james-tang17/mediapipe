#include <iostream>
#include <deque>
#include "facedetect.h"
#include <opencv2/core/ocl.hpp>

using namespace std;
using namespace cv;

static CascadeClassifier cascade;

static void mergeBoxes(Mat& img, vector<Rect> &target) {
    deque<Rect> mergedBoxes;
    deque<Rect> unmergedBoxes(target.begin(), target.end());
    Rect mergedBox;

    const char* drawskinbox = getenv("FD_DRAW_SKIN_BOX_BEFORE_MERGE");
    if (drawskinbox != NULL)
    {
        if (strcmp(drawskinbox, "true") == 0)
        {
            for (size_t i = 0; i < target.size(); i++) {
                Rect bRect = target[i];
                ellipse(img, cvPoint(cvRound(bRect.x+bRect.width/2), cvRound(bRect.y+bRect.height/2)),
                           cvSize(cvRound(bRect.width/2), cvRound(bRect.height/2)),
                           0, 0, 360,
                           CV_RGB(255,0,0), 2, 8, 0);
            }            
        }
    }
    
    while (unmergedBoxes.size()) {
        mergedBox = unmergedBoxes.front();
        unmergedBoxes.pop_front();

        size_t index = 0;
        while (index < mergedBoxes.size()) {
            Rect box = mergedBoxes.front();
            mergedBoxes.pop_front();
            if (max(box.x, mergedBox.x) > min(box.x + box.width, mergedBox.x + mergedBox.width) ||
                max(box.y, mergedBox.y) > min(box.y + box.height, mergedBox.y + mergedBox.height)) {
                mergedBoxes.push_back(box);
                index++;
            } else {
                Rect newMergedBox;
                
                newMergedBox.x = min(box.x, mergedBox.x);
                newMergedBox.width = max(box.x + box.width, mergedBox.x + mergedBox.width) - newMergedBox.x;
                newMergedBox.y = min(box.y, mergedBox.y);
                newMergedBox.height = max(box.y + box.height, mergedBox.y + mergedBox.height) - newMergedBox.y;
                mergedBox = newMergedBox;
                index = 0;
            }
        }
        mergedBoxes.push_back(mergedBox);
    }
    target.clear();
    target.insert(target.end(), mergedBoxes.begin(), mergedBoxes.end());

    drawskinbox = getenv("FD_DRAW_SKIN_BOX_AFTER_MERGE");
    if (drawskinbox != NULL)
    {
        if (strcmp(drawskinbox, "true") == 0)
        {
            for (size_t i = 0; i < target.size(); i++) {
                Rect bRect = target[i];
                rectangle(img, cvPoint(cvRound(bRect.x), cvRound(bRect.y)),
                           cvPoint(cvRound(bRect.x + bRect.width), cvRound(bRect.y + bRect.height)),
                           CV_RGB(0,255,0), 2, 8, 0);
            }            
        }
    }
    
}

static void skindetect(Mat& img, vector<Rect>& boxes)
{
    Mat blurred;
    blur(img, blurred, Size(50,50));

    Mat hsv;
    cvtColor(blurred, hsv, CV_BGR2HSV);

    Mat bw;
    inRange(hsv, Scalar(0, 10, 0), Scalar(20, 200, 255), bw);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(bw, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    for (size_t i = 0; i < contours.size(); i++) 
    {
        Rect rect = boundingRect(contours[i]);
        if (rect.width * rect.height > 1000) 
        {
            int x2 = min(rect.x + rect.width * 1.2, img.cols - 0.0);
            int y2 = min(rect.y + rect.height * 1.2, img.rows - 0.0);

            rect.x = max(rect.x-cvRound(rect.width*0.2), 0);
            rect.y = max(rect.y-cvRound(rect.height*0.2), 0);
            rect.width = x2 - rect.x;
            rect.height = y2 - rect.y;
            
            boxes.push_back(rect);
        }
    }

    mergeBoxes(img, boxes);
}


int fd_init(const string& cascadename)
{
    if( !cascade.load(cascadename))
    {
        cout << "ERROR: Could not load classifier cascade: " << cascadename << endl;
        return -1;
    }
    
    return 0;
}


int fd_detect_gpu(Mat& img, vector<Rect>& faces)
{
    ocl::setUseOpenCL(true);
    
    UMat image, gray;
    img.copyTo(image);

    if (image.channels() == 1)
    {
        gray = image;
    }
    else
    {
        cvtColor(image, gray, COLOR_BGR2GRAY );
    }

    equalizeHist(gray, gray );

    cascade.detectMultiScale(gray, faces,
        1.1, 3, 0
        //|CASCADE_FIND_BIGGEST_OBJECT
        //|CASCADE_DO_ROUGH_SEARCH
        |CASCADE_SCALE_IMAGE
        ,
        Size(30, 30) );
    
    return 0;
}


int fd_detect_cpu(Mat& img, vector<Rect>& faces)
{
    ocl::setUseOpenCL(false);
    
    Mat gray;

    if (img.channels() == 1)
    {
        gray = img;
    }
    else
    {
        cvtColor(img, gray, COLOR_BGR2GRAY );
    }

    equalizeHist(gray, gray );

    cascade.detectMultiScale(gray, faces,
        1.1, 3, 0
        //|CASCADE_FIND_BIGGEST_OBJECT
        //|CASCADE_DO_ROUGH_SEARCH
        |CASCADE_SCALE_IMAGE
        ,
        Size(30, 30) );
    
    return 0;
}


int fd_detect_withskin(Mat& image, vector<Rect>& faces)
{   
    ocl::setUseOpenCL(true);
    
    Mat gray;

    cvtColor(image, gray, COLOR_BGR2GRAY);
    equalizeHist(gray, gray);

    vector<Rect> boxes;
    skindetect(image, boxes);
    
    for (size_t i = 0; i < boxes.size(); i++) 
    {
        Rect rect = boxes[i];
        //cout << "orig: " << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << endl;
        //rect.x = max(rect.x-cvRound(rect.width*0.25), 0);
        //rect.y = max(rect.y-cvRound(rect.height*0.25), 0);
        //rect.width = min(cvRound(rect.width*1.5), img.cols - rect.x);
        //rect.height = min(cvRound(rect.height*1.5), img.rows - rect.y);
        //cout << "large: " << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << endl;
        
        Mat cropped = gray(rect);

        int minSize = cvRound(min(cropped.rows, cropped.cols)/1.5);
        minSize = min(minSize, 30);

        vector<Rect> pfaces;
        cascade.detectMultiScale(cropped, pfaces, 
                                  1.1, 3, 
                                  CASCADE_SCALE_IMAGE, 
                                  Size(minSize,minSize), Size(0, 0));

        for (size_t i = 0; i < pfaces.size(); ++i)
        {
            Rect face = pfaces[i];
            face.x += rect.x;
            face.y += rect.y;
            faces.push_back(face);
        }        
    }
    return 0;
}

