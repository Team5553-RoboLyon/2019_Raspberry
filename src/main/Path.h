#pragma once

#include <math.h>
#include "opencv2/opencv.hpp"
#include "Spline.h"

using namespace std;
using namespace cv;

#define CIRCLE_TANGENT_COEF                0.552f        // longueur de la tangente par rapport au rayon pour dessiner un cercle avec une spline.
#define ROBOT_BOUNDING_CIRCLE_RADIUS    0.6f        // metre ( = 60 cm de rayon, soit un cercle de 1.2 m�tres de diam�tre )

class Path {
    Path();
    //~Path();
public:
    bool buildPath(cv::Point2f start, cv::Point2f startdir, cv::Point2f end, cv::Point2f enddir);

    float m_endRadiusMin; // Rayon de courbure minimum � l'arriv�e
    Spline m_spline;
};