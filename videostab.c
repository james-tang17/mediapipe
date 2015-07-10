
#include "videostab.h"

#define USE_OMP 1

const int N_BLOCKS = 5;
const int N_THREADS = N_BLOCKS * N_BLOCKS;
const int N_FEATURES = 30;

const int BIN_HEIGHT = 2;
const int BIN_WIDTH = 2;

void detect_keypt(Mat &image, vector<KeyPoint> &keypoints, int nfeatures)
{
	Ptr<GFTTDetector> detector = GFTTDetector::create(nfeatures);

	detector->detect(image, keypoints);
}


vector<Mat> first_patches[N_THREADS];
vector<Point2f> first_points[N_THREADS];

extern "C" int set_first_img(Mat &img, int index)
{
    int rows = img.rows;
    int cols = img.cols;

    vector<Point2f> last_points[GRID_HEIGHT][GRID_WIDTH];
    vector<KeyPoint> kpoints[GRID_HEIGHT][GRID_WIDTH];

	#pragma omp parallel for
    for (int i = 0; i < GRID_HEIGHT; i++) {
        for (int j = 0; j < GRID_WIDTH; j++) {
            int xl = j * cols / GRID_WIDTH;
            int xr = (j + 1) * cols / GRID_WIDTH;

            int yl = i * rows / GRID_HEIGHT;
            int yr = (i + 1) * rows / GRID_HEIGHT;

            Mat subimg = img(Range(yl, yr), Range(xl, xr));

            detect_keypt(subimg, kpoints[i][j], N_FEATURES);

            for (int k = 0; k < kpoints[i][j].size(); k++) {
                last_points[i][j].push_back(kpoints[i][j][k].pt);
            }
        }
    }

    return 0;
}

extern "C" Mat get_match_parallel(Mat &img1, Mat &img2, int index)
{
	int64 t_start;
	int64 t_end;

    vector<Point2f> last_points[GRID_HEIGHT][GRID_WIDTH];
	vector<KeyPoint> lpoints[GRID_HEIGHT][GRID_WIDTH];
	vector<Point2f> pointsPre[GRID_HEIGHT][GRID_WIDTH], pointsNext[GRID_HEIGHT][GRID_WIDTH];
	vector<Point2f> inlierPointsPre[GRID_HEIGHT][GRID_WIDTH], inlierPointsNext[GRID_HEIGHT][GRID_WIDTH];

    vector<float> track_error[GRID_HEIGHT][GRID_WIDTH];
    
    Mat_<float> M = Mat::eye(3, 3, CV_32F);

	int rows = img1.rows;
	int cols = img1.cols;
    
	t_start = getTickCount();
    
	#pragma omp parallel for
	for (int i = 0; i < GRID_HEIGHT; i++) {
        for (int j = 0; j < GRID_WIDTH; j++) {
            int xl = j * cols / GRID_WIDTH;
            int xr = (j + 1) * cols / GRID_WIDTH;

            int yl = i * rows / GRID_HEIGHT;
            int yr = (i + 1) * rows / GRID_HEIGHT;

            Mat subimg = img1(Range(yl, yr), Range(xl, xr));
    		Mat subdst = img2(Range(yl, yr), Range(xl, xr));

            detect_keypt(subimg, lpoints[i][j], N_FEATURES);
            
            if (lpoints[i][j].size() < 3) {
                continue;
            }

            for (int k = 0; k < lpoints[i][j].size(); k++) {
                last_points[i][j].push_back(lpoints[i][j][k].pt);
            }

		    Size winSize(9, 9);
 
	        vector<Point2f> pointsGoodPre, pointsGoodNext;
        
            int ninliers;
            Mat first_motion;
       
            //track from last image to this image
	        vector<uchar> status;
            calcOpticalFlowPyrLK(subimg, subdst, last_points[i][j], pointsNext[i][j], status, noArray(), winSize, 2);
 
            //inverse track from current image to last image
	        vector<uchar> status_back;
            calcOpticalFlowPyrLK(subdst, subimg, pointsNext[i][j], pointsPre[i][j], status_back, noArray(), winSize, 2);
            
            for (int k = 0; k < pointsPre[i][j].size(); k++) {
                if (!status_back[k]) {
                    continue;
                }

                float err = (pointsPre[i][j][k].x - last_points[i][j][k].x) * (pointsPre[i][j][k].x - last_points[i][j][k].x)
                            + (pointsPre[i][j][k].y - last_points[i][j][k].y) * (pointsPre[i][j][k].y - last_points[i][j][k].y);
                err = sqrt(err);

                if (err < 0.2) {
                    track_error[i][j].push_back(sqrt(err));
                    #if 0
                    pointsGoodPre.push_back(last_points[i][j][k]);
                    pointsGoodNext.push_back(pointsNext[i][j][k]);
                    #endif
                    Point2f p0, p1;
                    p0 = last_points[i][j][k];
                    p1 = pointsNext[i][j][k];
                    
                    p0.x += xl;
                    p1.x += xl;
                    p0.y += yl;
                    p1.y += yl;

                    //cout << p0.y << ", " << p0.x << " : " << p1.y << ", " << p1.x << endl;
                    
                    inlierPointsPre[i][j].push_back(p0);
                    inlierPointsNext[i][j].push_back(p1);
                    
                    //cout << "after: " << inlierPointsPre[i][j][inlierPointsPre[i][j].size() - 1].y << ", " << inlierPointsPre[i][j][inlierPointsPre[i][j].size() - 1].x 
                        //<< " : " << inlierPointsNext[i][j][inlierPointsNext[i][j].size() - 1].y << ", " << inlierPointsNext[i][j][inlierPointsNext[i][j].size() - 1].x << endl;
                }
            }
        }
    }
    
    

    int total_points = 0;

    vector<Point2f> globalPre[BIN_HEIGHT][BIN_WIDTH], globalNext[BIN_HEIGHT][BIN_WIDTH];
    
	
    for (int i = 0; i < GRID_HEIGHT; i++) {
        for (int j = 0; j < GRID_WIDTH; j++) {
            for (int k = 0; k < inlierPointsPre[i][j].size(); k++) {
                Point2f p0, p1;
                p0 = inlierPointsPre[i][j][k];
                p1 = inlierPointsNext[i][j][k];

                int bin_index_y = p0.y * BIN_HEIGHT / rows;
                int bin_index_x = p0.x * BIN_WIDTH / cols;
                

                globalPre[bin_index_y][bin_index_x].push_back(p0);
                globalNext[bin_index_y][bin_index_x].push_back(p1);

                total_points++;
            }
            //cout << "(" << motions[i][j].at<float>(0, 2) << ", " << motions[i][j].at<float>(1, 2) << ") ";
        }
        //cout << endl;
    }
    
    vector<Point2f> filterPre[BIN_HEIGHT][BIN_WIDTH], filterNext[BIN_HEIGHT][BIN_WIDTH];

    #pragma omp parallel for
    for (int i = 0; i < BIN_HEIGHT; i++) {
        for (int j = 0; j < BIN_WIDTH; j++) {
            MotionModel motion_model = MM_TRANSLATION;

            RansacParams param = RansacParams::default2dMotion(motion_model);
            param.thresh = 1.5;
            int inliers = 0;

            Mat_<float> m = estimateGlobalMotionRansac(globalPre[i][j], globalNext[i][j], motion_model, param, 0, &inliers);
            
            for (int k = 0; k < globalPre[i][j].size(); k++) {
                Point2f p0, p1;
                p0 = globalPre[i][j][k];
                p1 = globalNext[i][j][k];

                float x = m(0,0) * p0.x + m(0,1) * p0.y + m(0,2);
                float y = m(1,0) * p0.x + m(1,1) * p0.y + m(1,2);

                float err = (x - p1.x) * (x - p1.x) + (y - p1.y) * (y - p1.y);
                if (err < param.thresh * param.thresh) {
                    filterPre[i][j].push_back(p0);
                    filterNext[i][j].push_back(p1);
                }
            }
        }
    }
    
    vector<Point2f> inlierPre, inlierNext; 
    for (int i = 0; i < BIN_HEIGHT; i++) {
        for (int j = 0; j < BIN_WIDTH; j++) {
            for (int k = 0; k < filterPre[i][j].size(); k++) {
                inlierPre.push_back(filterPre[i][j][k]);
                inlierNext.push_back(filterNext[i][j][k]);
            }
        }
    }
    

    MotionModel motion_model = MM_AFFINE;
 
    RansacParams param = RansacParams::default2dMotion(motion_model);
    param.thresh = 1.0;
    int inliers = 0;
    
    //Mat_<float> m_affine = estimateGlobalMotionRansac(inlierPre, inlierNext, motion_model, param, 0, &inliers);
    Mat_<float> m_affine = estimateGlobalMotionLeastSquares(inlierPre, inlierNext, motion_model, 0);

    t_end = getTickCount();
    
    cout << "frame: " << index << " total points: " << total_points
            << " total inliers: " << inlierPre.size() 
            << " time: " << (t_end - t_start) / getTickFrequency()
            << endl;

    return m_affine;
}

