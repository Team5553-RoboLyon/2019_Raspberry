#pragma once

#include "PrecompilationDefine.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>
#include <math.h>
#include "opencv2/opencv.hpp"

#include "Statistics.h"
#include "Vision.h"
#include "VisionTarget.h"

using namespace cv;
using namespace std;

class VisionTarget;

typedef struct {
    VisionTarget *m_pto;            // Pointeur sur la vision Target associ�e( par ce lien ) avec celle poss�dant ce lien.
    cv::Point2f m_nXZ;            // Vecteur normalis� des composantes X et Z du vecteur vtA,vtB
    float m_angleQuality;    // note de qualit� de l'angle avec l'horizontale de la paire "from-to"
    float m_sizeQuality;    // note de qualit� la "longueur" de la paire "from-to" par rapoprt � la longeur �talon
    bool m_bType;        // les 2 vision targets reli�es par ce lien forme un angle vers le bas ,ouvert (m_bType = 1) ou ferm� (m_btype = 0)
} VisionTargetLink;

class Vision;
//class VisionTargetPair;

#define VISIONTARGET_LINKSIZE    4

class VisionTarget {
public:

    //VisionTarget();
    //~VisionTarget();

    //VisionTarget & operator = (const VisionTarget & shape);

    bool estimate3Dposition(Vision *pcontext3D);


    //cv::Rect			boundingRect;
    unsigned int m_contourId;        // contour associ� � la visionTarget. Ce contour est dans le vector "m_contours" de la classe vision "m�re"
    cv::Point2f m_center;        // Vision Target Center of rotated rectangle ( il ne s'agit pas du centre projet� !!! )

    cv::Point2f m_point[4];
    unsigned int m_lowerId;        // Index du "lower point", le point le plus bas (parmis les 4 contenu dans point[])
    unsigned int m_higherId;        // Index du "high point", le point le plus haut (parmis les 4 contenu dans point[])

    cv::Point2f m_lower2D;        // copie du "lower point" ( = point[m_lowerId] )
    cv::Point2f m_higher2D;        // copie du "higher point"( = point[m_higherId] )
    cv::Point3f m_lower3D;        // point3D associ� au point m_lower2D
    cv::Point3f m_higher3D;        // point3D associ� au point m_higher2D

    cv::Point2f m_n01;            // supposed to be the ROTATED RECT Y axis ( The vertical ONE, oriented from bottom to top )
    cv::Point2f m_n03;            // supposed to be the ROTATED RECT X axis ( The horizontal ONE, oriented from left to right )
    float m_norm01;            // Norme du vecteur n01
    float m_norm03;            // Norme du vecteur n03
    float m_area;            // surface du recangle orient� ( compos� par les 4 points de point[] )

    // Pour la construction du chain graph
    VisionTargetLink m_links[VISIONTARGET_LINKSIZE];
    unsigned long m_linkSize;

    float m_chainQuality;
    VisionTarget *m_pChainPrev;
    VisionTargetLink *m_pLinkPrev;
    bool m_isPiledUp;
};

