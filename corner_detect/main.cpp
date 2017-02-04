/*
 *      Author: alexanderb
 */

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "Harris.h"

using namespace cv;
using namespace std;

//Harris algorithm parameters
// Specifies the sensitivity factor of the Harris algorithm (0 < k < 0.25)
float k = 0.25;
// Size of the box filter that is applied to the integral images
int boxFilterSize = 6;

int markDimension = 5;
bool gauss = true;
Mat m_img;

void doHarris() {
    // compute harris
    Harris harris(m_img, k, boxFilterSize, gauss);

    // get vector of points wanted
    vector<pointData> resPts = harris.getMaximaPoints(0.000519, boxFilterSize, 10);
    // cout << resPts.size() << " Points" << endl;

    Mat _img = Harris::MarkInImage(m_img, resPts, markDimension);
    imshow("HarrisCornerDetector", _img);
}


//-----------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
    // read image from file + error handling
    Mat img;

    if (argc == 1) {
        cout << "No image provided! Usage: ./Ex1 [path to image]" << endl << "Using default image: haus.jpg" << endl;

        img = imread("haus.jpg");
    } else {
        img = imread(argv[1]);
    }

    // if(img.rows > 100 || img.cols > 100) {
    //     int newrows = 600;
    //     int newcols = img.cols * newrows / img.rows;

    //     resize(img, img, Size(newcols, newrows), 0, 0, INTER_CUBIC);
    // }
    img.copyTo(m_img);

    // create UI and show the image
    //namedWindow("HarrisCornerDetector", 1);

    doHarris();
    //imshow("HarrisCornerDetector", img);
    waitKey(0);

    return 0;

}
