#include <iostream>
#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <wiringPi.h>  

using namespace std; 
using namespace cv;
using namespace raspicam;


RaspiCam_Cv cam; 
const int width_ = 360;
const int height_ = 240;
const int roiHeight_ = 100; 


/****************************************************************************
 *  Sets the related pins as outputs since Raspberry Pi is the master device         
*****************************************************************************/
void pinConfigurations()
{
    pinMode(21, OUTPUT); 
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT); 
}

/*********************************
 *  Sets up the camera properties 
**********************************/
void cameraSetup(int argc, char **argv)
{
    cam.set(CAP_PROP_FRAME_WIDTH, ("-w", argc, argv, 360));          
    cam.set(CAP_PROP_FRAME_HEIGHT, ("-h", argc, argv, 240));
    cam.set(CAP_PROP_BRIGHTNESS, ("-br", argc, argv, 50));
    cam.set(CAP_PROP_CONTRAST, ("-co", argc, argv, 50));
    cam.set(CAP_PROP_SATURATION, ("-sa", argc, argv, 50));
    cam.set(CAP_PROP_GAIN, ("-g", argc, argv, 50));
    cam.set(CAP_PROP_FPS, ("-fps", argc, argv, 0));  // set the camera to capture as much frames as it can 
}

/*****************************************************************
 *  Captures an image and converts its colorspace from BGR to RGB         
******************************************************************/ 
void captureAndConvert(Mat &img, Mat &obsImg, Mat &trfImg)
{
    cam.grab(); 
    cam.retrieve(img);
    cvtColor(img, obsImg, COLOR_BGR2RGB); 
    cvtColor(img, trfImg, COLOR_BGR2RGB); 
    cvtColor(img, img, COLOR_BGR2RGB);
}

/*************************************
 *  Displays the video frame-by-frame        
**************************************/ 
void display(Mat img)
{
    namedWindow("Camera", WINDOW_KEEPRATIO); 
    moveWindow("Camera", 0, 100);
    resizeWindow("Camera", 640, 480);         // zoom-in
    imshow("Camera", img);
    waitKey(1);                               // display the frame for 1 ms 
}

/************************************************************************************ 
 * Applies perspective transformation to an image considering the region of interest              
*************************************************************************************/   
void perspectiveTransform(Mat &img, Mat &warpedImg, Point2f src[], Point2f dst[]) 
{
    // draw lines around the region of interest
    line(img, src[2], src[0], Scalar(255, 255, 0), 2);
    line(img, src[0], src[1], Scalar(255, 255, 0), 2);
    line(img, src[1], src[3], Scalar(255, 255, 0), 2);
    line(img, src[2], src[3], Scalar(255, 255, 0), 2); 

    // get the bird-eye view
    Mat matrix = getPerspectiveTransform(src, dst); 
    warpPerspective(img, warpedImg, matrix, Size(360, 240)); 
}

/*******************************
 *  Finds the edges in an image          
********************************/
void edgeDetection(Mat warpedImg, Mat &grayImg, Mat &filteredImg, Mat &edgedImg, Mat &mergedImg)
{
    cvtColor(warpedImg, grayImg, COLOR_RGB2GRAY);
    inRange(grayImg, 150, 255, filteredImg);        // apply threshold filter to extract lane lines
    Canny(grayImg, edgedImg, 600, 700, 3, false);   // canny edge detection
    add(filteredImg, edgedImg, mergedImg);          // merge them
    cvtColor(mergedImg, mergedImg, COLOR_GRAY2RGB);              
}

/*******************************************************************************
 *  Creates a region of interest around the lane lines with 1-pixel-wide strips              
********************************************************************************/ 
void histogram(Mat mergedImg, Mat &roiStrip, vector<int> &strips)
{
    strips.resize(width_);
    strips.clear();
    for(int i = 0; i < width_; i++)
    {
        roiStrip = mergedImg(Rect(i, height_ - roiHeight_, 1, roiHeight_));   // create a 1-pixel-wide strip 
        divide(255, roiStrip, roiStrip);                                      // normalize pixel values in a single strip
        strips.push_back((int)(sum(roiStrip)[0]));                            // put sum of the all pixel values in a single strip into the strips vector  
    }
}

/**************************
 *  Locates the lane lines       
***************************/
void locateLines(int &leftLinePos, int &rightLinePos, vector<int> strips)
{
    vector<int>::iterator leftLinePtr; 
    leftLinePtr = max_element(strips.begin(), strips.begin()+130);
    leftLinePos = distance(strips.begin(), leftLinePtr); 
    vector<int>::iterator rightLinePtr; 
    rightLinePtr = max_element(strips.begin()+230, strips.end());
    rightLinePos = distance(strips.begin(), rightLinePtr);  
}

/****************************************************************************************************************
 * Calculates and returns the difference between the frame center and the lane center to determine the direction      
 * return value > 0 indicates it steered right 
 * return value < 0 indicates it steered left
 * return value = 0 indicates it's at the center position 
*****************************************************************************************************************/ 
int navigate(const int frameCenter, int &laneCenter, int leftLinePos, int rightLinePos) 
{
    laneCenter = leftLinePos + (rightLinePos - leftLinePos) / 2; 
    return frameCenter - laneCenter;    
}

/********************************************
 *  Sends signals to centralize the position        
*********************************************/ 
void centralize(int direction)
{
    if(direction == 0)
    {
        digitalWrite(21, 0);
        digitalWrite(22, 0);
        digitalWrite(23, 0);
        digitalWrite(24, 0);
    }
    else if(direction < 0 && direction > -10)
    {
        digitalWrite(21, 1);
        digitalWrite(22, 0);
        digitalWrite(23, 0);
        digitalWrite(24, 0);
    }
    else if(direction <= -10 && direction > -20)
    {
        digitalWrite(21, 0);
        digitalWrite(22, 1);
        digitalWrite(23, 0);
        digitalWrite(24, 0);
    }
    else if(direction <= -20)
    {
        digitalWrite(21, 1);
        digitalWrite(22, 1);
        digitalWrite(23, 0);
        digitalWrite(24, 0);
    }
    else if(direction > 0 && direction < 10)
    {
        digitalWrite(21, 0);
        digitalWrite(22, 0);
        digitalWrite(23, 1);
        digitalWrite(24, 0);
    }
    else if(direction >= 10 && direction < 20)
    {
        digitalWrite(21, 1);
        digitalWrite(22, 0);
        digitalWrite(23, 1);
        digitalWrite(24, 0);
    }
    else if(direction >= 20)
    {
        digitalWrite(21, 0);
        digitalWrite(22, 1);
        digitalWrite(23, 1);
        digitalWrite(24, 0);
    }
}

/*************************************************
 *  Detects an obstacle and returns its distance        
 *  If no object is detected, returns -1.0 
**************************************************/ 
float detectObstacle(CascadeClassifier obstacle, Mat obsImg, Mat &gray_obsImg, Mat &roiObs, 
                     vector<Rect> &obsVec, const float slopeObs, const float interceptObs)
{
    if(!obstacle.load("//home//pi//Desktop//Self-Driving-Robot-Car//obstacle_cascade.xml"))
    {
        cerr << "ERROR: Could not load obstacle_cascade.xml file!" << endl;
        exit(1);
    } 
    float distObs = -1.0f;
    roiObs = obsImg(Rect(width_/4, roiHeight_, width_/2, height_ - roiHeight_));        // set a proper region of interest     
    cvtColor(roiObs, gray_obsImg, COLOR_RGB2GRAY);    
    equalizeHist(gray_obsImg, gray_obsImg);                                             // increase the contrast of the image
    obstacle.detectMultiScale(gray_obsImg, obsVec);                                     // each rectangle in the vector contains the detected object  
    for(int i = 0; i < obsVec.size(); i++)
    {
        Point P1 = (obsVec[i].x, obsVec[i].y);                                                                                      
        Point P2 = (obsVec[i].x + obsVec[i].width, obsVec[i].y + obsVec[i].height);                                                 
        rectangle(roiObs, P1, P2, Scalar(0,0,255), 2);                                  // draw a rectangle around the detected object 
        putText(roiObs, "Obstacle", P1, FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 2);     // write the name of the detected object
        distObs = slopeObs * (P2.x - P1.x) + interceptObs;                              // calculate the distance of the object based on the size of the rectangle 
    }
    return distObs; 
}

/************************************************************************
 *  Detects a traffic light with the red light and returns its distance       
 *  If no object is detected, returns -1.0 
*************************************************************************/ 
float detectTrafficLight(CascadeClassifier trafficLight, Mat trfImg, Mat &gray_trfImg, Mat &roiTrf, 
                         vector<Rect> &trfVec, const float slopeTrf, const float interceptTrf)
{
    if(!trafficLight.load("//home//pi//Desktop//Self-Driving-Robot-Car//trafficLight_cascade.xml"))
    {
        cerr << "ERROR: Could not load trafficLight_cascade.xml file!" << endl;
        exit(1);
    }
    float distTrf = -1.0f; 
    roiTrf = trfImg(Rect(width_/2, 0, width_/2, height_ - roiHeight_));                 
    cvtColor(roiTrf, gray_trfImg, COLOR_RGB2GRAY);    
    equalizeHist(gray_trfImg, gray_trfImg);                                             
    trafficLight.detectMultiScale(gray_trfImg, trfVec);                                     
    for(int i = 0; i < trfVec.size(); i++)
    {
        Point P1 = (trfVec[i].x, trfVec[i].y);                                                                                      
        Point P2 = (trfVec[i].x + trfVec[i].width, trfVec[i].y + trfVec[i].height);                                                 
        rectangle(roiTrf, P1, P2, Scalar(0,0,255), 2);                                  
        putText(roiTrf, "Traffic Light", P1, FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 2);     
        distTrf = slopeTrf * (P2.x - P1.x) + interceptTrf;                                     
    }
    return distTrf; 
}

/********************************
 *  Sends signals to change lane       
*********************************/   
void changeLane()
{
    digitalWrite(21, 1);
    digitalWrite(22, 1);
    digitalWrite(23, 1);
    digitalWrite(24, 0);
}

/*************************
 * Sends signals to stop              
**************************/  
void stop()
{
    digitalWrite(21, 0);
    digitalWrite(22, 0);
    digitalWrite(23, 0);
    digitalWrite(24, 1);
}

int main(int argc, char **argv)
{
    /*********************************
     * Variables for image processing 
    **********************************/
    Mat img, warpedImg, grayImg, filteredImg, edgedImg, mergedImg, roiStrip;    
    Point2f src[] = {Point2f(25,155), Point2f(330,155), Point2f(0,215), Point2f(360,215)}; 
    Point2f dst[] = {Point2f(80,0), Point2f(280,0), Point2f(80,240), Point2f(280,240)};  
    vector<int> strips;
    int leftLinePos = -1; 
    int rightLinePos = -1; 
    int laneCenter = -1; 
    const int frameCenter =  169;  // calibrated by overlapping it with the lane center at the central position   

    /*********************************
     * Variables for object detection 
    **********************************/ 
    CascadeClassifier obstacle, trafficLight;  
    Mat obsImg, gray_obsImg, roiObs, trfImg, gray_trfImg, roiTrf;  
    vector<Rect> obsVec, trfVec; 
    float detectedDist = -1.0f; 
    // slope and intercept will be used in the linear equation while calculating the distance of the detected object 
    const float slopeObs = -0.64f;        
    const float interceptObs = 53.48f; 
    const float slopeTrf = -2.5f;
    const float interceptTrf = 120.2f; 

    cameraSetup(argc, argv); 
    cout << "Connecting to the camera..." << endl; 
    if(!cam.open())
    {
        cerr << "ERROR: Failed to connect!" << endl;
        return -1;
    }
    wiringPiSetup();
    pinConfigurations();    
    while(1)
    {
        captureAndConvert(img, obsImg, trfImg); 
        perspectiveTransform(img, warpedImg, src, dst);  
        edgeDetection(warpedImg, grayImg, filteredImg, edgedImg, mergedImg);
        histogram(mergedImg, roiStrip, strips);
        locateLines(leftLinePos, rightLinePos, strips); 
        int direction = navigate(frameCenter, laneCenter, leftLinePos, rightLinePos); 
        centralize(direction);   

        detectedDist = detectObstacle(obstacle, obsImg, gray_obsImg, roiObs, obsVec, slopeObs, interceptObs); 
        // If an obstacle is closer than 20 cm, change lane (In practice it's slightly less than 20 cm because of the inertia of the motors)  
        if(detectedDist > 0.0f && detectedDist < 20.0f) 
        {
            changeLane();
        }

        detectedDist = detectTrafficLight(trafficLight, trfImg, gray_trfImg, roiTrf, trfVec, slopeTrf, interceptTrf); 
        // If a traffic light is closer than 20 cm and the red light is on, stop 
        if(detectedDist > 0.0f && detectedDist < 20.0f) 
        {
            stop(); 
        }

        display(img);  
    }
    return 0; 
}
