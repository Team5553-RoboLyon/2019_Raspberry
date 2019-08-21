#pragma once

#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include "PrecompilationDefine.h"

#include "opencv2/opencv.hpp"

#include "Values.h"
#include "VisionTarget.h"
#include "VisionTargetPair.h"

using namespace std;
using namespace cv;

// Constantes
#define TOLERANCE_3DCOORD_f32 \
  0.5f  // On parle ici en cm !! et il s'agit de la marge d'erreur acceptable dans l'estimation des distances
// 3D
#define EPSILON_2DCOORD_f32 0.0001f              // tres (trop) petite valeur pour une coordonn�es 2D
#define EPSILON_VECTOR_CROSSPRODUCT_f32 0.0001f  // tres petite valeur pour un Cross produt (float)

#define COORD3DMAX (-1000000)  // Just a big number
#define HUD_3D_ALIGNMENT_RECT_SIZE_Z (-7.0f)
#define HUD_3D_ALIGNMENT_RECT_TOP_Y_SHIFT (0.0f)
#define HUD_3D_ALIGNMENT_RECT_BOTTOM_Y_SHIFT (-5.0f)

class VisionTarget;

class VisionTargetPair;

class Vision {
public:
    enum enumFlags {
        FLAG_CAPTURE_PAUSE = 0
    };

    Vision(const float screenwidth = 320.0f, const float screenheight = 240.0f);

    ~Vision();

    // METHODES
    // contr�le capture video
    void capturePause();

    void captureResume();

    void capturePauseToggle();

    // Capture video Process
    const bool imageCapture();

    const bool imageProcess();

    // retour video ( pilote)
    void videoBackRender();

    void videoBackRender(const cv::Mat &img);

    // Camera
    void setCurrentCamera(const unsigned long id);

    void setCameraYWorld(const float y);

    void setCameraBrigthness(const double b);

    void setCameraContrast(const double c);

    void setCameraExposure(const double e);

    void setCameraGain(const double g);

    float getCameraYWorld();

    double getCameraBrigthness();

    double getCameraContrast();

    double getCameraExposure();

    double getCameraGain();

    // Calibration camera
    const bool loadCameraIntrinsic(const unsigned long camid, const char *filename);

    const bool saveCameraIntrinsic(const unsigned long camid, const char *filename);

    // 3D
    const bool estimate3DPoint(const cv::Point2f &point, const float realYFromTheGround, Point3f &resultat3D);

    const float estimate3DVerticalBipoint(const cv::Point2f (&point)[2], const float (&realYFromTheGround)[2],
                                          Point3f (&resultat3D)[2]);

    const float estimateIntrinsicHorizontalAngleRad(const cv::Point2f &p);

    const float estimateIntrinsicVerticalAngleRad(const cv::Point2f &p);

    const float estimateIntrinsicDistanceZ(const cv::Point2f &p, const float realHeightFromTheGround);

    const float getCurrentCameraFocalDistX();

    const float getCurrentCameraFocalDistY();

    const float getCurrentScreenCenterX();

    const float getCurrentScreenCenterY();

    const bool loadVisionTargetModel(const char *filename);

    const bool loadCameraParam(const char *filename);

    // Vision Target Detection Methods
    const unsigned long findVisionTargetPairs();

    const unsigned long extractVisionTargetCandidates();

    const unsigned long buildVisionTargetsChainGraph();

    const unsigned long extractBestVisualTargetPairChain();

    VisionTargetPair *getCenteredTargetPair();

    void buildRegressionLines();

    // Render FeedBack
    void renderVisionContours(cv::InputOutputArray image);

    void renderVisionTargetCandidates(cv::InputOutputArray image);

    void renderVisionTargetPairs(cv::InputOutputArray image);

    void renderHorizon(cv::InputOutputArray image, const unsigned long mode = 0);

    void render3DTopView(cv::InputOutputArray image);

    void renderHud(cv::InputOutputArray image, const unsigned long mode = 0);

    void render3DVisionTargets(cv::InputOutputArray image, const unsigned long mode = 0);

    void renderMax(cv::InputOutputArray image);

    // IMAGE PROCESSING PARAM
    float m_areaMinRatio;  // = ADVPARAM_VISIONTARGET_SIZEMIN;	// means a/1000 of each screen dimension >>>
    // (a/1000 of screen width) * (a/1000 of screen height)
    float m_areaMaxRatio;  // = ADVPARAM_VISIONTARGET_SIZEMAX;	// means a/1000 of each screen dimension >>>
    // (a/1000 of screen width) * (a/1000 of screen height)
    float m_horizonRatio;  // = ADVPARAM_HORIZON;					// means a/1000 of screen height

    float m_areaMin;
    float m_areaMax;
    float m_horizon;

#ifdef PRECOMPIL_IMAGEFILTER_HLS
    int m_Hlow;   // = FILTRE_HLS_HLOW;
    int m_Llow;   // = FILTRE_HLS_LLOW;
    int m_Slow;   // = FILTRE_HLS_SLOW;
    int m_Hhigh;  // = FILTRE_HLS_HHIGH;
    int m_Lhigh;  // = FILTRE_HLS_LHIGH;
    int m_Shigh;  // = FILTRE_HLS_SHIGH;
#endif

#ifdef PRECOMPIL_IMAGEFILTER_2GRAY_THRESHOLD
    int m_threshMin;  // = FILTRE_THRESHOLD_MIN;
    int m_threshMax;  // = FILTRE_THRESHOLD_MAX;
#endif

#ifdef PRECOMPIL_IMAGEFILTER_CANNY
    double m_cannyThreshMin;  // = FILTRE_CANNY_THRESH1;
    double m_cannyThreshMax;  // = FILTRE_CANNY_THRESH2;
#endif

    // Systeme � deux cameras:
    float m_screenWidth;   // Les deux cameras capture des images de m�me taille
    float m_screenHeight;  // Les deux cameras capture des images de m�me taille
    double m_brightness;
    double m_contrast;
    double m_exposure;
    double m_gain;
    double m_awb_red;
    double m_awb_blue;
    double m_saturation;

    float m_currentCameraYworld;  // Une seule camera active en m�me temps.
    unsigned long m_currentCameraID;      // 0 (camera avant) ou 1 (camera arriere)
    //
    // La matrice m_intrinsic:
    //
    // | focalDistX		0		scrCenterX	|
    // | 	0		focalDistY	scrCenterY	|
    // | 	0			0			 1		|
    //
    // 2 definitions intrinsic compl�tes, une par camera:
    Mat m_intrinsic[2];
    Mat m_distCoeffs[2];
    vector<Mat> m_rvecs[2];
    vector<Mat> m_tvecs[2];

    VideoCapture m_captureDevice;

    Mat m_imageSrc;
    Mat m_imageUndistorded;
    Mat m_imageFiltered;
    Mat m_imageHLS;  // check if its possible to work directly with m_imageFiltered

    // Vision Targets Pairs detection
    vector<vector<Point> > m_contours;
    vector<VisionTarget> m_visionTargets;
    vector<VisionTargetPair> m_visionTargetPairs;
    vector<Point3f> m_visionTarget3DModel;

    VisionTargetPair *m_pSelectedVisionTargetPair;

    unsigned long m_flags;

    // *********************************************************************************************************************
    // OLD ONES
    /*
    void setFov(const double camerafov_deg);

    cv::Point3f	m_Xaxis;
    cv::Point3f	m_Yaxis;
    cv::Point3f	m_Zaxis;

    double		m_fovY;						// Camera FOVY (DEG). Specifies the field of view angle, in degrees, in the y
    direction. double		m_fovDist;
    double		m_aspectRatio;				// Specifies the aspect ratio that determines the field of view in the x
    direction. The aspect ratio is the ratio of x (width) to y (height).
    double		m_tangent;					// Pre-computed Data used for precise FRUSTRUM Culling and other things ...
    double		m_inverseTangent;				// Pre-computed Data
    double		m_widthoutof2;
    double		m_heightoutof2;
    */
};
