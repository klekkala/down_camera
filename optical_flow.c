#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#define pi 3.14
#define MAXNUMFEATURES 400

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

static IplImage *frame = NULL, *frame1_1C = NULL, *frame2_1C = NULL, *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;

//pre allocation of memory for faster access
static int number_of_features = MAXNUMFEATURES;
//features found in frame 1
static CvPoint2D32f frame1_features[MAXNUMFEATURES];
//the locations of the points from frame 1 in frame 2
static CvPoint2D32f frame2_features[MAXNUMFEATURES];
//The i-th element of this array will be non-zero if the i-th feature of frame 1 was found in frame 2
static char optical_flow_found_feature[MAXNUMFEATURES];
//The i-th element of this array is the error in the optical flow for the i-th feature of frame1 as found in frame 2
static float optical_flow_feature_error[MAXNUMFEATURES];

//This is the window size to use to avoid the aperture problem
static CvSize optical_flow_window;

static CvTermCriteria optical_flow_termination_criteria;

//squares a number
inline static double square(int a)
{
    return a * a;
}

//allocates image memory
inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels )
{
    if ( *img != NULL ) return;

    *img = cvCreateImage( size, depth, channels );
    if ( *img == NULL )
    {
        fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
        exit(-1);
    }
}

int main(void)
{
    double fps = 0, sec = 0;
    int counter=0, dfps=0;
    time_t startTime, endTime;
    int i;
    optical_flow_window = cvSize(3,3);
    CvPoint p,q;

    //terminates the algorithm either after 20 iterations or when the epsilon is better than .3. Adjusting these parameters for speed vs. accuracy.
    optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

    //creates capture
    CvCapture *input_video = cvCreateCameraCapture(0);
    if (input_video == NULL)
    {
        fprintf(stderr, "Error: Can't open video.\n");
        return -1;
    }

    //gets frame data
    CvSize frame_size;
    frame_size.height = (int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_HEIGHT );
    frame_size.width =  (int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_WIDTH );

    //grab first frame
    frame = cvQueryFrame( input_video );

    //Allocate memory
    allocateOnDemand( &frame1_1C, frame_size, IPL_DEPTH_8U, 1 );
    allocateOnDemand( &frame2_1C, frame_size, IPL_DEPTH_8U, 1 );
    allocateOnDemand( &eig_image, frame_size, IPL_DEPTH_32F, 1 );
    allocateOnDemand( &temp_image, frame_size, IPL_DEPTH_32F, 1 );
    allocateOnDemand( &pyramid1, frame_size, IPL_DEPTH_8U, 1 );
    allocateOnDemand( &pyramid2, frame_size, IPL_DEPTH_8U, 1 );

    cvConvertImage(frame, frame1_1C, 0);

    #if OPERATINGSYSTEM == WINDOWS
        cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
    #endif
    //start timmer
    time(&startTime);
//===========================Main Loop=================================================
    while(1)
    {
        //interchange current frame to last frame
        cvConvertImage(frame1_1C, frame2_1C, 0);

        frame = cvQueryFrame( input_video );
        //checks if frame is avalible
        if (frame == NULL)
        {
            return -1;
        }

        cvConvertImage(frame, frame1_1C, 0);

        //grabs features, in this case corners
        cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &number_of_features, .01, 10, NULL, /*blockSize*/3,0,0);
        //uses features to perform Optical Flow
        cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features, optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0 );

//--------------------------------------------optical flow analysis--------------------------------
        //iterate through the flow feild
        for(i = 0; i < number_of_features; i++)
        {
            #if OPERATINGSYSTEM == WINDOWS
                float avgMag=0, avgAngle=0;
                //check if any features
                if ( optical_flow_found_feature[i] == 0 )   continue;

                int line_thickness = 1;
                CvScalar line_color = CV_RGB(255,0,0);

                //point on frame one
                p.x = (int) frame1_features[i].x;
                p.y = (int) frame1_features[i].y;
                //same point on frame two
                q.x = (int) frame2_features[i].x;
                q.y = (int) frame2_features[i].y;

                double angle;       angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
                double hypotenuse;  hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

                avgMag = avgMag + hypotenuse;
                avgAngle = avgAngle + angle;

                //Here we lengthen the arrow by a factor of three.
                q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
                q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

                //drawing the base line of the arrow
                cvLine( frame, p, q, line_color, line_thickness, CV_AA, 0 );
                //only draw arrow head if greater then 1
                if (hypotenuse>1)
                {
                    p.x = (int) (q.x + 9 * cos(angle + pi / 4));
                    p.y = (int) (q.y + 9 * sin(angle + pi / 4));
                    cvLine( frame, p, q, line_color, line_thickness, CV_AA, 0 );
                    p.x = (int) (q.x + 9 * cos(angle - pi / 4));
                    p.y = (int) (q.y + 9 * sin(angle - pi / 4));
                    cvLine( frame, p, q, line_color, line_thickness, CV_AA, 0 );
                }
            #endif
        }
//--------------------------------------------end of optical flow analysis--------------------------------

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
            //show image
            cvShowImage("Optical Flow", frame);
            char c = cvWaitKey(WAIT);
            if( c == 27 ) break;
        #else
            usleep(WAIT);
        #endif
    }
//===========================End of Main Loop=================================================
}
