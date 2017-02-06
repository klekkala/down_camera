#include "Harris.h"

Harris::Harris(Mat img, float k, int filterRange) {
    // (1) Convert to greyscalescale image
    Mat greyscaleImg = convertRgbToGrayscale(img);

    // (2) Compute Derivatives
    Derivatives derivatives = computeDerivatives(greyscaleImg);

    // (3) Gaussian Filtering
    Derivatives mDerivatives;
    GaussianBlur(derivatives.Ix, mDerivatives.Ix, Size(filterRange,filterRange), 0, 0, BORDER_DEFAULT );
    GaussianBlur(derivatives.Iy, mDerivatives.Iy, Size(filterRange,filterRange), 0, 0, BORDER_DEFAULT );
    GaussianBlur(derivatives.Ixy, mDerivatives.Ixy, Size(filterRange,filterRange), 0, 0, BORDER_DEFAULT );

    printf("22\n");

    // (4) Compute Harris Responses
    Mat harrisResponses = computeHarrisResponses(k, mDerivatives);
    m_harrisResponses = harrisResponses;
    printf("2\n");
}

//-----------------------------------------------------------------------------------------------
vector<pointData> Harris::getMaximaPoints(float percentage, int filterRange, int suppressionRadius) {
    // Declare a max suppression matrix
    Mat maximaSuppressionMat(m_harrisResponses.rows, m_harrisResponses.cols, CV_32FC1, Scalar::all(0));

    // Create a vector of all Points
    std::vector<pointData> points;
    for (int r = 0; r < m_harrisResponses.rows; r++) {
        for (int c = 0; c < m_harrisResponses.cols; c++) {
            Point p(r,c); 

            pointData d;
            d.cornerResponse = m_harrisResponses.at<float>(r,c);
            d.point = p;

            points.push_back(d);
        }
    }

    // Sort points by corner Response
    sort(points.begin(), points.end(), by_cornerResponse());

    // Get top points, given by the percentage
    int numberTopPoints = m_harrisResponses.cols * m_harrisResponses.rows * percentage;
    std::vector<pointData> topPoints;

    int i=0;
    while(topPoints.size() < numberTopPoints) {
        if(i == points.size())
            break;

        int supRows = maximaSuppressionMat.rows;
        int supCols = maximaSuppressionMat.cols;

        // Check if point marked in maximaSuppression matrix
        if(maximaSuppressionMat.at<int>(points[i].point.x,points[i].point.y) == 0) {
            for (int r = -suppressionRadius; r <= suppressionRadius; r++) {
                for (int c = -suppressionRadius; c <= suppressionRadius; c++) {
                    int sx = points[i].point.x+c;
                    int sy = points[i].point.y+r;

                    // bound checking
                    if(sx > supRows)
                        sx = supRows;
                    if(sx < 0)
                        sx = 0;
                    if(sy > supCols)
                        sy = supCols;
                    if(sy < 0)
                        sy = 0;

                    maximaSuppressionMat.at<int>(points[i].point.x+c, points[i].point.y+r) = 1;
                }
            }

            // Convert back to original image coordinate system 
            points[i].point.x += 1 + filterRange;
            points[i].point.y += 1 + filterRange;
            topPoints.push_back(points[i]);
        }

        i++;
    }

    return topPoints;
}

//-----------------------------------------------------------------------------------------------
Mat Harris::convertRgbToGrayscale(Mat& img) {
    Mat greyscaleImg(img.rows, img.cols, CV_32F);

    for (int c = 0; c < img.cols; c++) {
        for (int r = 0; r < img.rows; r++) {
            greyscaleImg.at<float>(r,c) = 
                0.2126 * img.at<cv::Vec3b>(r,c)[0] +
                0.7152 * img.at<cv::Vec3b>(r,c)[1] +
                0.0722 * img.at<cv::Vec3b>(r,c)[2];
        }
    }

    return greyscaleImg;
}

//-----------------------------------------------------------------------------------------------
Derivatives Harris::computeDerivatives(Mat& greyscaleImg) {
    // Helper Mats for better time complexity
    Mat sobelHelperV(greyscaleImg.rows-2, greyscaleImg.cols, CV_32F);
    for(int r=1; r<greyscaleImg.rows-1; r++) {
        for(int c=0; c<greyscaleImg.cols; c++) {

            float a1 = greyscaleImg.at<float>(r-1,c);
            float a2 = greyscaleImg.at<float>(r,c);
            float a3 = greyscaleImg.at<float>(r+1,c);

            sobelHelperV.at<float>(r-1,c) = a1 + a2 + a2 + a3;
        }
    }

    Mat sobelHelperH(greyscaleImg.rows, greyscaleImg.cols-2, CV_32F);
    for(int r=0; r<greyscaleImg.rows; r++) {
        for(int c=1; c<greyscaleImg.cols-1; c++) {

            float a1 = greyscaleImg.at<float>(r,c-1);
            float a2 = greyscaleImg.at<float>(r,c);
            float a3 = greyscaleImg.at<float>(r,c+1);

            sobelHelperH.at<float>(r,c-1) = a1 + a2 + a2 + a3;
        }
    }

    // Apply Sobel filter to compute 1st derivatives
    Mat Ix(greyscaleImg.rows-2, greyscaleImg.cols-2, CV_32F);
    Mat Iy(greyscaleImg.rows-2, greyscaleImg.cols-2, CV_32F);
    Mat Ixy(greyscaleImg.rows-2, greyscaleImg.cols-2, CV_32F);

    for(int r=0; r<greyscaleImg.rows-2; r++) {
        for(int c=0; c<greyscaleImg.cols-2; c++) {
            Ix.at<float>(r,c) = sobelHelperH.at<float>(r,c) - sobelHelperH.at<float>(r+2,c);
            Iy.at<float>(r,c) = - sobelHelperV.at<float>(r,c) + sobelHelperV.at<float>(r,c+2);
            Ixy.at<float>(r,c) = Ix.at<float>(r,c) * Iy.at<float>(r,c);
        }
    }

    Derivatives d;
    d.Ix = Ix;
    d.Iy = Iy;
    d.Ixy = Iy;

    return d;
}
//-----------------------------------------------------------------------------------------------
Mat Harris::computeHarrisResponses(float k, Derivatives& d) {
    Mat M(d.Iy.rows, d.Ix.cols, CV_32FC1);

    for(int r=0; r<d.Iy.rows; r++) {  
        for(int c=0; c<d.Iy.cols; c++) {
            float   a11, a12,
                    a21, a22;

            a11 = d.Ix.at<float>(r,c) * d.Ix.at<float>(r,c);
            a22 = d.Iy.at<float>(r,c) * d.Iy.at<float>(r,c);
            a21 = d.Ix.at<float>(r,c) * d.Iy.at<float>(r,c);
            a12 = d.Ix.at<float>(r,c) * d.Iy.at<float>(r,c);

            float det = a11*a22 - a12*a21;
            float trace = a11 + a22;

            M.at<float>(r,c) = abs(det - k * trace*trace);
        }
    }

    return M;
}



Mat Harris::MarkInImage(Mat& img, vector<pointData> points, int radius) {
    Mat retImg;
    img.copyTo(retImg);

    for(vector<pointData>::iterator it = points.begin(); it != points.end(); ++it) {
        Point center = (*it).point;

        // down
        for(int r=-radius; r<radius; r++) {
            retImg.at<Vec3b>(Point(center.y+r,center.x+radius)) = Vec3b(0, 0, 255);
        }

        // up
        for(int r=-radius; r<radius; r++) {
            retImg.at<Vec3b>(Point(center.y+r,center.x-radius)) = Vec3b(0, 0, 255);
        }

        // left
        for(int c=-radius; c<radius; c++) {
            retImg.at<Vec3b>(Point(center.y-radius,center.x+c)) = Vec3b(0, 0, 255);
        }

        // right
        for(int c=-radius; c<radius; c++) {
            retImg.at<Vec3b>(Point(center.y+radius,center.x+c)) = Vec3b(0, 0, 255);
        }

        retImg.at<Vec3b>(Point(center.y,center.x)) = Vec3b(0, 255, 0);
    }

    return retImg;
}