#pragma once

#include "opencv2/opencv.hpp"
#include "VisionTarget.h"
#include "VisionTargetPair.h"

using namespace cv;


namespace Hud {
    // Dessine un viseur au milieu de l'image
    void drawSight(cv::InputOutputArray image, const Point2f &center, const float extend);

    void drawSight2(cv::InputOutputArray image, const Point2f &center, const float extend);

    // Dessine le rectangle orient� associ� � une VisionTarget
    void
    drawVisionTargetRotatedRectangle(cv::InputOutputArray image, const VisionTarget &visiontarget, const int mode = 0);

    // Dessine de quadrilatere associ� au couple de vision target
    void drawVisionTargetPair(cv::InputOutputArray image, VisionTargetPair &vtcouple);

    // (proj�te) et Dessine un rep�re 3D
    //void draw3DBase(cv::InputOutputArray image, const cv::Mat rot_vector, const cv::Mat trans_vector, const float size);
    void drawPoint(cv::InputOutputArray image, const cv::Point2f &p, const float extend, const cv::Scalar &color);
}