#pragma once

#include "PrecompilationDefine.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>
#include <math.h>
#include "opencv2/opencv.hpp"

#include "VisionTarget.h"

#define PAIR_NORM3DTOP            2.5f // distance r�elle en m�tre s�parant les deux shapes d'une m�me paire.
#define PAIR_NORM3DBOTTOM        3.5f // distance r�elle en m�tre s�parant les deux shapes d'une m�me paire.
#define PAIR_NORM3DRATIO        (PAIR_NORM3DTOP/PAIR_NORM3DBOTTOM)


class VisionTarget;

class VisionTargetPair {
public:
    enum enumCenter {
        TRAPEZOID = 0,
        TRAPEZOID_BOTTOM_SIDE = 1
    };

    const bool assemble(VisionTarget *pshapeA, VisionTarget *pshapeB, const bool type);

    VisionTarget *m_pvisionTargetA;
    VisionTarget *m_pvisionTargetB;


    float m_norm3DBottom;
    float m_norm3DTop;

    float m_quality;                // note de qualit� du couple en fonction des r�sultats aux tests

    cv::Point2f m_center[2];            // m_center[0] = Centre du Trapezoid: point d'intersection des deux diagonales du Trapezoid
    // m_center[1] = milieu du segment bas du Trapezoid

    cv::Point3f m_dirVect;
    bool m_bType;                // les 2 vision targets reli�es par ce lien forme un angle vers le bas ,ouvert (m_bType = 1) ou ferm� (m_btype = 0)

    // 3D Output: rotation and translation for Pnp:
    //bool			b3Dsolved;				// Pnp return value
    //cv::Mat			m_rotation_vector;		// Rotation in axis-angle form
    //cv::Mat			m_translation_vector;	// Translation from camera(0,0,0)
};
