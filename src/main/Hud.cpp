#include "Hud.h"
#include "Vision.h"

void Hud::drawSight(cv::InputOutputArray image, const Point2f &center, const float extend) {
    Point2f v0, v1;

    // Horizontal
    v0.x = center.x - extend;
    v0.y = center.y;
    v1.x = center.x + extend;
    v1.y = v0.y;
    line(image, v0, v1, Scalar(0, 0, 255));
    // Vertical
    v0.x = center.x;
    v0.y = center.y - extend;
    v1.x = v0.x;
    v1.y = center.y + extend;
    line(image, v0, v1, Scalar(0, 0, 255));
}

void Hud::drawSight2(cv::InputOutputArray image, const Point2f &center, const float extend) {
    Point2f v0, v1;


    // Horizontal centre
    v0.x = center.x - extend * 2;
    v0.y = center.y;
    v1.x = center.x - 5;
    v1.y = v0.y;
    line(image, v0, v1, Scalar(192, 192, 0));

    v0.x = center.x + extend * 2;
    v0.y = center.y;
    v1.x = center.x + 5;
    v1.y = v0.y;
    line(image, v0, v1, Scalar(192, 192, 0));

    // Vertical centre
//	v0.x = center.x;			v0.y = center.y - extend/4;
//	v1.x = v0.x;				v1.y = center.y + extend/4;
//	line(image, v0, v1, Scalar(128, 128,0));

    // Verticales Droite et gauche
    v0.x = center.x - 5;
    v0.y = center.y - extend / 8;
    v1.x = v0.x;
    v1.y = center.y + extend / 8;
    line(image, v0, v1, Scalar(192, 192, 0));

    v0.x = center.x + 5;
    v0.y = center.y - extend / 8;
    v1.x = v0.x;
    v1.y = center.y + extend / 8;
    line(image, v0, v1, Scalar(192, 192, 0));

    // Verticales Droite et gauche �loign�es
    v0.x = center.x - (5 + extend);
    v0.y = center.y - extend / 16;
    v1.x = v0.x;
    v1.y = center.y;
    line(image, v0, v1, Scalar(216, 216, 0));

    v0.x = center.x + (5 + extend);
    v0.y = center.y - extend / 16;
    v1.x = v0.x;
    v1.y = center.y;
    line(image, v0, v1, Scalar(216, 216, 0));

    // Vertical Droite et gauche �loign�es ++
    v0.x = center.x - (5 + 2 * extend);
    v0.y = center.y - extend / 8;
    v1.x = v0.x;
    v1.y = center.y;
    line(image, v0, v1, Scalar(255, 255, 0));
    v0.x = center.x + (5 + 2 * extend);
    v0.y = center.y - extend / 8;
    v1.x = v0.x;
    v1.y = center.y;
    line(image, v0, v1, Scalar(255, 255, 0));

}

void
Hud::drawVisionTargetRotatedRectangle(cv::InputOutputArray image, const VisionTarget &visiontarget, const int mode) {
    // 01: Vertical axis
    cv::line(image, visiontarget.m_point[0], visiontarget.m_point[(1)], Scalar(50, 128, 50), 1);
    // 12
    cv::line(image, visiontarget.m_point[1], visiontarget.m_point[(2)], Scalar(100, 100, 100), 1);
    // 23
    cv::line(image, visiontarget.m_point[2], visiontarget.m_point[(3)], Scalar(100, 100, 100), 1);
    // 30: Horizontal axis
    cv::line(image, visiontarget.m_point[3], visiontarget.m_point[(0)], Scalar(50, 50, 128), 1);

    if (mode == 1) {
        cv::putText(image, "0", visiontarget.m_point[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        cv::putText(image, "1", visiontarget.m_point[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
        cv::putText(image, "2", visiontarget.m_point[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 128, 128));
        cv::putText(image, "3", visiontarget.m_point[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
    }
}

void Hud::drawVisionTargetPair(cv::InputOutputArray image, VisionTargetPair &vtcouple) {
    // Version standard
    cv::line(image, vtcouple.m_pvisionTargetA->m_higher2D, vtcouple.m_pvisionTargetB->m_lower2D, Scalar(0, 0, 255), 1);
    cv::line(image, vtcouple.m_pvisionTargetA->m_lower2D, vtcouple.m_pvisionTargetB->m_higher2D, Scalar(0, 0, 255), 1);


    // Diagonales
    cv::line(image, vtcouple.m_pvisionTargetA->m_higher2D, vtcouple.m_pvisionTargetB->m_lower2D, Scalar(128, 255, 128),
             1);
    cv::line(image, vtcouple.m_pvisionTargetA->m_lower2D, vtcouple.m_pvisionTargetB->m_higher2D, Scalar(128, 255, 128),
             1);

    Point2f v0 = vtcouple.m_center[VisionTargetPair::TRAPEZOID];
    v0.x -= 5;
    v0.y += 50 + vtcouple.m_quality * 10;
    cv::putText(image, to_string(vtcouple.m_quality), v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
}

void Hud::drawPoint(cv::InputOutputArray image, const cv::Point2f &p, const float extend, const cv::Scalar &color) {
    // Trace une  croix blanche ed taille "extend" � la position p.
    cv::Point2f v0 = p;
    cv::Point2f v1 = p;
    // |
    v0.y -= extend;
    v1.y += extend;
    line(image, v0, v1, color);
    // _
    v0.y = p.y;
    v1.y = p.y;
    v0.x -= extend;
    v1.x += extend;
    line(image, v0, v1, color);
}
