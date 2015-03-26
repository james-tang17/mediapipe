#include "../../facedetect.h"

#include <iostream>
using namespace std;
using namespace cv;

const static Scalar colors[] =  { CV_RGB(0,0,255),
                                  CV_RGB(0,128,255),
                                  CV_RGB(0,255,255),
                                  CV_RGB(0,255,0),
                                  CV_RGB(255,128,0),
                                  CV_RGB(255,255,0),
                                  CV_RGB(255,0,0),
                                  CV_RGB(255,0,255)
                                } ;

int main(int argc, char **argv)
{
    string cascadename = "";
    string nestcascadename = "";
    string infile = "";
    string outfile = "";
    bool skin = false;
    bool cpu = false;
    int loop = 0;
    for( int i = 1; i < argc; i++ )
    {
        if (strcmp(argv[i], "-cascade") == 0)
        {
            ++i;
            cascadename = argv[i];
        }
        if (strcmp(argv[i], "-nestcascade") == 0)
        {
            ++i;
            nestcascadename = argv[i];
        }        
        else if (strcmp(argv[i], "-i") == 0)
        {
            ++i;
            infile = argv[i];
        }
        else if (strcmp(argv[i], "-o") == 0)
        {
            ++i;
            outfile = argv[i];
        }
        else if (strcmp(argv[i], "-skin") == 0)
        {
            ++i;
            skin = (strcmp(argv[i], "true") == 0);
        }   
        else if (strcmp(argv[i], "-cpu") == 0)
        {
            ++i;
            cpu = (strcmp(argv[i], "true") == 0);
        }         
        else if (strcmp(argv[i], "-loop") == 0)
        {
            ++i;
            loop = atoi(argv[i]);
        }         
    }
    
    if (cascadename.length() == 0 || infile.length() == 0)
    {
        cout << "usage: " << argv[0];
        cout << " -cascade haarcascade_frontalface_alt.xml";
        cout << " -i in.jpg";
        cout << " [-o out.jpg -skin true -loop 10 -cpu true]";
        cout << endl;
        return 0;
    }

    if (skin && cpu)
    {
        cout << "do not support cpu version with skin detect" << endl;
        return 0;
    }

    fd_init(cascadename);

    Mat image = imread(infile, CV_LOAD_IMAGE_COLOR);
    if(image.empty())
    {
        cout << "unable to open file " << infile << " for read" << endl;
        return 0;
    }

    vector<Rect> faces;
    if (skin)
    {
        fd_detect_withskin(image, faces);
    }
    else
    {
        if (cpu)
        {
            fd_detect_cpu(image, faces);
        }
        else
        {
            fd_detect_gpu(image, faces);
        }
    }

    if (loop > 0)
    {
        double time = 0;
        for (int i = 0; i < loop; ++i)
        {
            faces.clear();
            double t = (double)cvGetTickCount();
            if (skin)
            {
                fd_detect_withskin(image, faces);
            }
            else
            {
                if (cpu)
                {
                    fd_detect_cpu(image, faces);
                }
                else
                {
                    fd_detect_gpu(image, faces);
                }
            }
            time += ((double)cvGetTickCount() - t);
        }
        cout << "avg detection time = " << time/((double)cvGetTickFrequency()*1000.)/loop << " ms" << endl;
    }

    int i = 0;
    for (vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        Point center;
        Scalar color = colors[i%8];
        int radius;
        center.x = cvRound(r->x + r->width*0.5);
        center.y = cvRound(r->y + r->height*0.5);
        radius = cvRound((r->width + r->height)*0.25);
        circle(image, center, radius, color, 8, 8, 0 );
    }

    if (outfile.length() > 0)
    {
        imwrite(outfile, image);
    }
    
    return 0;
}
