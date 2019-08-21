#include "Spline.h"
#include "Tools.h"

// SplineKnot
SplineKnot::SplineKnot() {
    m_position = Point2f(0, 0);
    m_ta = Point2f(0, 0);
    m_tb = Point2f(0, 0);
}

SplineKnot::SplineKnot(const cv::Point2f &pos, const cv::Point2f &ta, const cv::Point2f &tb) {
    m_position = pos;
    m_ta = ta;
    m_tb = tb;
}

SplineKnot::~SplineKnot() {}

void SplineKnot::setKnot(const cv::Point2f &pos, const cv::Point2f &ta, const cv::Point2f &tb) {
    m_position = pos;
    m_ta = ta;
    m_tb = tb;
}

void SplineKnot::setKnotPosition(const cv::Point2f &pos) {
    m_position = pos;
}

void SplineKnot::setKnotPosition(const float x, const float y) {
    m_position.x = x;
    m_position.y = y;
}

void SplineKnot::setKnotTangenteA(const cv::Point2f &a) {
    m_ta = a;
}

void SplineKnot::setKnotTangenteA(const float x, const float y) {
    m_ta.x = x;
    m_ta.y = y;
}

void SplineKnot::setKnotTangenteB(const cv::Point2f &b) {
    m_tb = b;
}

void SplineKnot::setKnotTangenteB(const float x, const float y) {
    m_tb.x = x;
    m_tb.y = y;
}

// Spline
Spline::Spline(const int capacity, const unsigned int accuracy) {
    m_accuracy = accuracy;
    m_splineKnots.reserve(capacity);
}

Spline::~Spline() {
    m_splineKnots.clear();
}

void Spline::insertKnot() {
    SplineKnot knot;
    m_splineKnots.push_back(knot);
}

void Spline::insertKnot(const unsigned int nb) {
    m_splineKnots.resize(m_splineKnots.size() + nb);
}

void Spline::insertKnot(const cv::Point2f &pos, const cv::Point2f &ta, const cv::Point2f &tb) {
    SplineKnot knot = SplineKnot(pos, ta, tb);
    m_splineKnots.push_back(knot);
}

void Spline::qLerp(Point2f &result, const unsigned int kaid, const unsigned int kbid, const float t) {
    Point2f R, L1, L2, L3, L4, L5;
    Tools::point2fLerp(L1, m_splineKnots[kaid].m_position, m_splineKnots[kaid].m_tb, t);
    Tools::point2fLerp(L2, m_splineKnots[kaid].m_tb, m_splineKnots[kbid].m_ta, t);
    Tools::point2fLerp(L3, m_splineKnots[kbid].m_ta, m_splineKnots[kbid].m_position, t);
    Tools::point2fLerp(L4, L1, L2, t);
    Tools::point2fLerp(L5, L2, L3, t);
    Tools::point2fLerp(result, L4, L5, t);
}

void Spline::drawSpline(cv::Mat image, cv::Scalar &color) {
    Point2f a, b;
    float fr, fac;
    fr = fac = 1.0f / (float) m_accuracy;

    for (unsigned int i = 1; i < m_splineKnots.size(); i++) {
        a = m_splineKnots[i - 1].m_position;
        for (unsigned int j = 1; j <= m_accuracy; j++, fr += fac) {
            qLerp(b, i - 1, i, fr);
            cv::line(image, a, b, color);
            a = b;
        }
    }
}


