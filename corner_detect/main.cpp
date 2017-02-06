/*
 *      Author: alexanderb
 */

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "Harris.h"

using namespace cv;
using namespace std;



//-----------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
    // read image from file + error handling
    Mat img;
    // Specifies the sensitivity factor of the Harris algorithm (0 < k < 0.25)
    float k = 0.25;
    // Size of the box filter that is applied to the integral images
    int boxFilterSize = 3;

    int markDimension = 5;

    if (argc == 1) {
        cout << "No image provided! Usage: ./Ex1 [path to image]" << endl << "Using default image: haus.jpg" << endl;

    } else {
        img = imread(argv[1], IMREAD_COLOR);
    }

    //GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );

    // compute harris
    Harris harris(img, k, boxFilterSize);

    // get vector of points wanted
    vector<pointData> resPts = harris.getMaximaPoints(0.000219, boxFilterSize, 10);
    // cout << resPts.size() << " Points" << endl;

    Mat _img = Harris::MarkInImage(img, resPts, markDimension);
    imshow("HarrisCornerDetector", _img);
    waitKey(0);

    return 0;

}
