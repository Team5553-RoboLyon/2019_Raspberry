#pragma once
//#include <iostream>

#include "opencv2/opencv.hpp"

using namespace cv;

// Constantes Maths et autres
#define EPSILON_LINEXLINE_f32 \
  0.000001f  // Utilis� par les fonctions "lineXline" et "segXseg" pour d�finir si deux droites/segments sont
// parall�les ( produit vectoriel NULL )
#define PI_f64 3.14159265358979323846   // PI
#define PI_f32 3.14159265358979323846f  // PI

#define EPSILON_VECTOR_DOTPRODUCT_f64 0.000001  // tres petite valeur pour un dot produt
#define EPSILON_VECTOR_DOTPRODUCT_f32 0.0001f   // tres petite valeur pour un dot produt (float)
#define EPSILON_NORM_f32 0.0001f                // tres petite valeur pour une norme (loat)

// MACROS
#define DEGtoRAD(deg) ((PI_f64 / 180) * (deg))  // Deg -> Radian
#define RADtoDEG(rad) ((180 / PI_f64) * (rad))  // Radian -> Deg

#define DEGtoRADf32(deg) ((PI_f32 / 180.0f) * (deg))  // Deg -> Radian
#define RADtoDEGf32(rad) ((180.0f / PI_f32) * (rad))  // Radian -> Deg

#define NABS(a) (((a) < 0) ? -(a) : (a))  // Absolute value ( valeur sans le signe )
#define IS_ODD(a) ((a)&1)  // Test if a number is a Odd number (2xn + 1 ) if not is Even (2xn) !

// BIT Manipulation
#define BITSET(val, bit_id) ((val) |= (1 << (bit_id)))
#define BITCLEAR(val, bit_id) ((val) &= ~(1 << (bit_id)))
#define BITGET(val, bit_id) ((val) & (1 << (bit_id)))
#define BITTOGGLE(val, bit_id) ((val) ^= (1 << (bit_id)))

namespace Tools {
    bool lineXLine(const cv::Point2f &A, const cv::Point2f &B, const cv::Point2f &C, const cv::Point2f &D,
                   cv::Point2f &pxrx);

    bool segXseg(const Point2f &A, const Point2f &B, const Point2f &C, const Point2f &D, Point2f &pxrx);

    float triangleSurface(const Point2f &A, const Point2f &B, const Point2f &C);

    inline void point2fLerp(Point2f &vr, const cv::Point2f vorigin, const cv::Point2f v, const float t) {
        vr.x = vorigin.x + (v.x - vorigin.x) * t;
        vr.y = vorigin.y + (v.y - vorigin.y) * t;
    }

    inline void point3fLerp(Point3f &vr, const cv::Point3f vorigin, const cv::Point3f v, const float t) {
        vr.x = vorigin.x + (v.x - vorigin.x) * t;
        vr.y = vorigin.y + (v.y - vorigin.y) * t;
        vr.z = vorigin.z + (v.z - vorigin.z) * t;
    }
}  // namespace Tools
