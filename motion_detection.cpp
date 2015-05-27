//libraies used
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <time.h>
#include <unistd.h>
 
//define operating system use
#define LINUX 1
#define WINDOWS 2
//#define OPERATINGSYSTEM WINDOWS
#define OPERATINGSYSTEM LINUX
 
//define delays used
#define DECLAREWAIT 20
#if OPERATINGSYSTEM == WINDOWS
    #define WAIT DECLAREWAIT
#else
    #define WAIT (DECLAREWAIT *1000)
#endif
 
main( int argc, char* argv[] )
{
    //declare varibles used
    double fps = 0, sec = 0;
    int counter=0, dfps=0;
    time_t startTime, endTime;
    int i,j;
    CvPoint minVal, maxVal;
    IplImage *frame, *img, *proImg, *proImgGray;
    CvCapture* capture = NULL;
    capture = cvCreateCameraCapture(0);
    assert( capture != NULL );
    //set image definition
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 320 );
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 240 );
 
//-------------------------------start---------------------------------------------------------
    //load first frame to help extract information
    frame = cvQueryFrame(capture);
    cvSmooth(frame, frame, CV_BLUR, 3,3, 3,3);
 
    //allocate memory required
    #if OPERATINGSYSTEM == WINDOWS
        cvNamedWindow( "motion", CV_WINDOW_AUTOSIZE );
    #endif
    //images
    proImg = cvCreateImage(cvSize(frame->width,frame->height), frame->depth, frame->nChannels);
    img  = cvCreateImage(cvSize(frame->width,frame->height), frame->depth, frame->nChannels);
    proImgGray  = cvCreateImage(cvSize(frame->width,frame->height), IPL_DEPTH_8U, 1);
 
    //save first image
    for(i=0;i<frame->width;++i)for(j=0;j<frame->height;++j)
    {
        ((uchar*)(img->imageData + img->widthStep*j))[i*3] =   ((uchar*)(frame->imageData + frame->widthStep*j))[i*3];
        ((uchar*)(img->imageData + img->widthStep*j))[i*3+1] = ((uchar*)(frame->imageData + frame->widthStep*j))[i*3+1];
        ((uchar*)(img->imageData + img->widthStep*j))[i*3+2] = ((uchar*)(frame->imageData + frame->widthStep*j))[i*3+2];
    }
    #if OPERATINGSYSTEM == WINDOWS
        cvWaitKey(WAIT);
    #else
        usleep(WAIT);
    #endif
 
    //start timmer
    time(&startTime);
//-------------------------------start non stop loop---------------------------------------------------------
    while(1)
    {
        //load frame
        frame = cvQueryFrame(capture);
 
        //only required on video file
        if(!frame) break;
 
        //smooth frame to elimiate noise
        cvSmooth(frame, frame, CV_BLUR, 3,3, 3,3);
 
        //level one motion detection
        minVal.x = frame->width;
        minVal.y = frame->height;
        maxVal.x = 0;
        maxVal.y = 0;
        for(i=0;i<frame->width;++i)for(j=0;j<frame->height;++j)
        {
           //abs diff-> convert to gray ->check for motion------------------------------------------------------------------------------------------------
            if((( abs(((uchar*)(frame->imageData + frame->widthStep*j))[i*3] - ((uchar*)(img->imageData + img->widthStep*j))[i*3]) +
                  abs(((uchar*)(frame->imageData + frame->widthStep*j))[i*3+1] - ((uchar*)(img->imageData + img->widthStep*j))[i*3+1]) +
                  abs(((uchar*)(frame->imageData + frame->widthStep*j))[i*3+2] - ((uchar*)(img->imageData + img->widthStep*j))[i*3+2]) )/3) > 35)  //adaptive number depenind on noise??
            {
                //motion active
                ((uchar*)(proImgGray->imageData + proImgGray->widthStep*j))[i] = 255;
                //find min and max
                if(j<minVal.y)
                    minVal.y = j;
                if(i<minVal.x)
                    minVal.x = i;
                if(j>maxVal.y)
                    maxVal.y = j;
                if(i>maxVal.x)
                    maxVal.x = i;
            }
            else
            {
                //no motion
                ((uchar*)(proImgGray->imageData + proImgGray->widthStep*j))[i] = 0;
            }
 
            //Clone image!!!---------------------------------------------------------------------------
            ((uchar*)(img->imageData + img->widthStep*j))[i*3] =   ((uchar*)(frame->imageData + frame->widthStep*j))[i*3];
            ((uchar*)(img->imageData + img->widthStep*j))[i*3+1] = ((uchar*)(frame->imageData + frame->widthStep*j))[i*3+1];
            ((uchar*)(img->imageData + img->widthStep*j))[i*3+2] = ((uchar*)(frame->imageData + frame->widthStep*j))[i*3+2];
        }
 
        //take time
        ++counter;
        if(counter%100==0)
        {
            time(&endTime);
            sec=difftime(endTime,startTime);
            fps=counter/sec;
            printf("%f\n",fps);
        }
        #if OPERATINGSYSTEM == WINDOWS
            cvRectangle(proImgGray, minVal, maxVal, cvScalar(255,255,255,0), 1, 8, 0 );
            cvShowImage("motion", proImgGray);
            /*ASCII 27 = esc*/
            char c = cvWaitKey(WAIT);
            if( c == 27 ) break;
        #else
            usleep(WAIT);
        #endif
    }
//-------------------------------end non stop loop---------------------------------------------------------
 
    //dealocate memory
    cvReleaseCapture(&capture);
    cvReleaseImage(&proImg);
    cvReleaseImage(&img);
    cvReleaseImage(&proImgGray);
    #if OPERATINGSYSTEM == WINDOWS
        cvDestroyWindow("motion");
    #endif
 
    return(0);
}
