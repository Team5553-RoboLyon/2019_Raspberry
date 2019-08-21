

#include "PrecompilationDefine.h"

#include <vector>
#include <memory>
#include <math.h>
#include "opencv2/opencv.hpp"

#include "Hud.h"
#include "Tools.h"

/*

#include "Values.h"

#include "Vision.h"
#include "VisionTarget.h"
#include "VisionTargetPair.h"

#include "Spline.h"
*/

#include "Editor.h"

using namespace cv;


double _Error(double trueNumber, double approxNumber) {
    return approxNumber / trueNumber * 100;
}


double estimate_VisionTargetPairAngle(double angleAOZ, double angleBOZ, double angleCOZ) {
    double angleAOB;
    double angleBOC;
//	double approx;

    double pi = 3.14159265359;
    double x = pi / 2;
    int i = 0;
    double k = 0;

    angleAOB = angleAOZ + angleBOZ;
    angleBOC = angleCOZ - angleBOZ;

    /*for (x=0; x<=pi; x+=0.01)
    {
        //y=2*pi-(x+c1+c2)
        approx = Error(sin(angleBOC)/sin(angleAOB), sin(pi-(x+angleAOB+angleBOC))/sin(x));

        if(approx <100.3 && approx > 99.7)
        {
           // cout << approx<< ":"<<sin(pi-(x+angleAOB+angleBOC))/sin(x)<<"   "<<sin(angleBOC)/sin(angleAOB)<< endl;
            return x;
        }

    }*/
    double test = sin(angleBOC) / sin(angleAOB);
    for (i = 0; i < 15; i++) {
        if (sin(pi - (x + angleAOB + angleBOC)) / sin(x) > test) {
            x = (x + k) / 2;
        }
        if (sin(pi - (x + angleAOB + angleBOC)) / sin(x) < test) {
            k = x;
            x += (x - k) / 2;
        }

    }
    return (x + angleAOZ);
}


// all Trackbar call back 
// camera
void on_trackbar_camheight(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->setCameraYWorld((float) value / 10.0f);
}

void on_trackbar_cambrightness(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->setCameraBrigthness(value - 100);
}

void on_trackbar_camexposure(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->setCameraExposure(value);
}

void on_trackbar_camgain(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->setCameraGain(value);
}

void on_trackbar_camcontrast(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->setCameraContrast(value);
}


#ifdef PRECOMPIL_IMAGEFILTER_HLS

// Low
void on_trackbar_HLS_Hlow(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_Hlow = value;
}

void on_trackbar_HLS_Llow(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_Llow = value;
}

void on_trackbar_HLS_Slow(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_Slow = value;
}

//High
void on_trackbar_HLS_Hhigh(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_Hhigh = value;
}

void on_trackbar_HLS_Lhigh(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_Lhigh = value;
}

void on_trackbar_HLS_Shigh(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_Shigh = value;
}

#endif


//Canny
#ifdef PRECOMPIL_IMAGEFILTER_CANNY

void on_trackbar_cannyUpperThres(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_cannyThreshMax = (double) value;
}

void on_trackbar_cannyLowerThres(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_cannyThreshMin = pvision->m_cannyThreshMax * (double) value / 100;
}

#endif
//Threshold
#ifdef PRECOMPIL_IMAGEFILTER_2GRAY_THRESHOLD
void on_trackbar_thresholdMin(int value, void *ptr)
{
    Vision *pvision = (Vision*)ptr;
    pvision->m_threshMin = value;
}
void on_trackbar_thresholdMax(int value, void *ptr)
{
    Vision *pvision = (Vision*)ptr;
    pvision->m_threshMax = value;
}
#endif

void on_trackbar_horizon(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_horizonRatio = value / 1000.0f;
    pvision->m_horizon = pvision->m_screenHeight * pvision->m_horizonRatio;
}

void on_trackbar_areaMin(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_areaMinRatio = (float) value / 1000.0f;
    pvision->m_areaMin =
            (pvision->m_screenHeight * pvision->m_areaMinRatio) * (pvision->m_screenWidth * pvision->m_areaMinRatio);
}

void on_trackbar_areaMax(int value, void *ptr) {
    Vision *pvision = (Vision *) ptr;
    pvision->m_areaMaxRatio = (float) value / 1000.0f;
    pvision->m_areaMax =
            (pvision->m_screenHeight * pvision->m_areaMaxRatio) * (pvision->m_screenWidth * pvision->m_areaMaxRatio);
}

Editor::Editor(Vision *pvision) {
    m_cursorCameraY = (int) (pvision->getCameraYWorld() * 10.0f);

    m_cursorBrigthness = (int) (pvision->getCameraBrigthness() + 100);
    m_cursorContrast = (int) (pvision->getCameraContrast());
    m_exposure = (int) pvision->getCameraExposure();
    m_cursorGain = (int) pvision->getCameraGain();

    m_cursorAreaMinRatio = (int) (pvision->m_areaMinRatio * 1000.0f);
    m_cursorAreaMaxRatio = (int) (pvision->m_areaMaxRatio * 1000.0f);
    m_cursorHorizonRatio = (int) (pvision->m_horizonRatio * 1000.0f);

#ifdef PRECOMPIL_IMAGEFILTER_HLS
    m_cursorHlow = pvision->m_Hlow;
    m_cursorLlow = pvision->m_Llow;
    m_cursorSlow = pvision->m_Slow;
    m_cursorHhigh = pvision->m_Hhigh;
    m_cursorLhigh = pvision->m_Lhigh;
    m_cursorShigh = pvision->m_Shigh;
#endif

#ifdef PRECOMPIL_IMAGEFILTER_2GRAY_THRESHOLD
    m_cursorThreshMin = pvision->m_threshMin;
    m_cursorThreshMax = pvision->m_threshMax;
#endif

#ifdef PRECOMPIL_IMAGEFILTER_CANNY
    m_cursorCannyThreshMax = (int) pvision->m_cannyThreshMax;
    m_cursorCannyThreshMinRatio = (int) ((pvision->m_cannyThreshMin / pvision->m_cannyThreshMax) * 100);
#endif

    // BASIC FILTER PARAM (toujours actif et compil�, quelquesoit le filtrage actif)
    namedWindow("CAMERA", WINDOW_AUTOSIZE);
    resizeWindow("CAMERA", 1000, 200);
    //createTrackbar("CamLight", "CAMERA", &camera_bright, 100, NULL);
    createTrackbar("Ycam(mm)", "CAMERA", &m_cursorCameraY, 1000, on_trackbar_camheight,
                   (void *) pvision);            // max value = 1000.>>	camera height is a value from 0 to 100.0cm (from 0/10 to 1000/10)
    createTrackbar("Brigthness", "CAMERA", &m_cursorBrigthness, 200, on_trackbar_cambrightness,
                   (void *) pvision);    // -100 < brightness < 100
    createTrackbar("Contrast", "CAMERA", &m_cursorContrast, 100, on_trackbar_camcontrast,
                   (void *) pvision);            // 0 < contrast < 100
    createTrackbar("Exposure", "CAMERA", &m_exposure, 100, on_trackbar_camexposure,
                   (void *) pvision);                // max value = 1000.>>	camera brightness is a value from 0 to
    createTrackbar("Gain", "CAMERA", &m_cursorGain, 100, on_trackbar_camgain,
                   (void *) pvision);                        // max value = 1000.>>	camera brightness is a value from 0 to
    // DETECTION PARAM
    namedWindow("CONTOUR-DETECT", WINDOW_AUTOSIZE);
    resizeWindow("CONTOUR-DETECT", 1000, 200);
    createTrackbar("Horizon", "CONTOUR-DETECT", &m_cursorHorizonRatio, 1000, on_trackbar_horizon,
                   (void *) pvision);    // max value = 1000.	>>	horizonRatio is a ratio from 0 to 1 (from 0/1000	 to 1000/1000)
    createTrackbar("AreaMin", "CONTOUR-DETECT", &m_cursorAreaMinRatio, 1000, on_trackbar_areaMin,
                   (void *) pvision);    // max value = 1000.>>	areaMinRatio is a ratio from 0 to 1 (from 0/1000 to 1000/1000)
    createTrackbar("AreaMax", "CONTOUR-DETECT", &m_cursorAreaMaxRatio, 1000, on_trackbar_areaMax,
                   (void *) pvision);    // max value = 1000.>>	areaMaxRatio is a ratio from 0 to 1 (from 0/1000 to 1000/1000)

#ifdef PRECOMPIL_IMAGEFILTER_HLS
    namedWindow("FILTERING-HLS", WINDOW_AUTOSIZE);
    resizeWindow("FILTERING-HLS", 500, 300);
    createTrackbar("H Low", "FILTERING-HLS", &m_cursorHlow, 255, on_trackbar_HLS_Hlow,
                   (void *) pvision);        // max value = 255. >>	HLS params are from 0 to 255
    createTrackbar("L Low", "FILTERING-HLS", &m_cursorLlow, 255, on_trackbar_HLS_Llow,
                   (void *) pvision);        // max value = 255. >>	HLS params are from 0 to 255
    createTrackbar("S Low", "FILTERING-HLS", &m_cursorSlow, 255, on_trackbar_HLS_Slow,
                   (void *) pvision);        // max value = 255. >>	HLS params are from 0 to 255
    createTrackbar("H High", "FILTERING-HLS", &m_cursorHhigh, 255, on_trackbar_HLS_Hhigh,
                   (void *) pvision);        // max value = 255. >>	HLS params are from 0 to 255
    createTrackbar("L High", "FILTERING-HLS", &m_cursorLhigh, 255, on_trackbar_HLS_Lhigh,
                   (void *) pvision);        // max value = 255. >>	HLS params are from 0 to 255
    createTrackbar("S High", "FILTERING-HLS", &m_cursorShigh, 255, on_trackbar_HLS_Shigh,
                   (void *) pvision);        // max value = 255. >>	HLS params are from 0 to 255

#endif

#ifdef PRECOMPIL_IMAGEFILTER_2GRAY_THRESHOLD
    namedWindow("FILTERING-THRESHOLD", WINDOW_AUTOSIZE);
    resizeWindow("FILTERING-THRESHOLD", 1000, 200);

    createTrackbar("ThresMin", "FILTERING-THRESHOLD", nullptr, 255, on_trackbar_thresholdMin, (void*)pvision);	// max value = 255. >>	thresholdMin is a value from 0 to 255
    createTrackbar("ThresMax", "FILTERING-THRESHOLD", nullptr, 255, on_trackbar_thresholdMax, (void*)pvision);	// max value = 255. >>	thresholdMax is a value from 0 to 255
#endif

#ifdef PRECOMPIL_IMAGEFILTER_CANNY
    namedWindow("FILTERING-CANNY", WINDOW_AUTOSIZE);
    resizeWindow("FILTERING-CANNY", 1000, 200);
    createTrackbar("Thres High", "FILTERING-CANNY", &m_cursorCannyThreshMax, 500, on_trackbar_cannyUpperThres,
                   (void *) pvision);    // max value = 500. >>	cannyThresMax is a value ... don't know exactly the range ...
    createTrackbar("Thres low Ratio", "FILTERING-CANNY", &m_cursorCannyThreshMinRatio, 100, on_trackbar_cannyLowerThres,
                   (void *) pvision);    // max value = 100. >>	cannyThresMin is a value given by the ratio with cannyUpperThres
#endif


}
