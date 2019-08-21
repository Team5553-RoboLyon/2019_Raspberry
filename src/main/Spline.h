#pragma once

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class SplineKnot {
public:
    SplineKnot();

    SplineKnot(const cv::Point2f &pos, const cv::Point2f &ta, const cv::Point2f &tb);

    ~SplineKnot();

    void setKnot(const cv::Point2f &pos, const cv::Point2f &ta, const cv::Point2f &tb);

    void setKnotPosition(const cv::Point2f &pos);

    void setKnotPosition(const float x, const float y);

    void setKnotTangenteA(const cv::Point2f &a);

    void setKnotTangenteA(const float x, const float y);

    void setKnotTangenteB(const cv::Point2f &b);

    void setKnotTangenteB(const float x, const float y);


    cv::Point2f m_position;
    cv::Point2f m_ta;
    cv::Point2f m_tb;
};

class Spline {
public:

    Spline(const int capacity = 2, const unsigned int accuracy = 10);

    ~Spline();

    void insertKnot();

    void insertKnot(const unsigned int nb);

    void insertKnot(const cv::Point2f &pos, const cv::Point2f &ta, const cv::Point2f &tb);

    void qLerp(Point2f &result, const unsigned int kaid, const unsigned int kbid, const float t);

    void drawSpline(cv::Mat image, cv::Scalar &color);

    unsigned int m_accuracy;                            // Precision
    std::vector<SplineKnot> m_splineKnots;    // liste des noeuds de la spline.( A priori seulement deux devraient suffire pour le Robot.)
};