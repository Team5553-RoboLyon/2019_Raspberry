
#include "PrecompilationDefine.h"

#include <math.h>
#include "Hud.h"
#include "Tools.h"
#include "Vision.h"

void Vision::renderVisionContours(cv::InputOutputArray image) {
    cv::drawContours(image, m_contours, -1, cv::Scalar(0, 0, 255), 2);
}

void Vision::renderVisionTargetCandidates(cv::InputOutputArray image) {
    for (unsigned int i = 0; i < m_visionTargets.size(); i++) {
        Hud::drawVisionTargetRotatedRectangle(image, m_visionTargets[i], 0);
    }
}

void Vision::renderVisionTargetPairs(cv::InputOutputArray image) {
    //	float f;
    //	cv::Point2f v0;

    for (unsigned int i = 0; i < m_visionTargetPairs.size(); i++) {
        Hud::drawVisionTargetRotatedRectangle(image, *m_visionTargetPairs[i].m_pvisionTargetA, 1);
        Hud::drawVisionTargetRotatedRectangle(image, *m_visionTargetPairs[i].m_pvisionTargetB, 1);
        /*
        // Affichage angle horizontal estim� en Degr�
        f =
        RADtoDEGf32(estimateIntrinsicHorizontalAngleRad(m_visionTargetPairs[i].m_center[VisionTargetPair::TRAPEZOID]));
        v0 = m_visionTargetPairs[i].m_center[VisionTargetPair::TRAPEZOID];
        v0.x -= 10;
        v0.y += 50;
        cv::putText(image, to_string(f), v0, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        // Affichage Ditance Z par rapport � la camera en cm !!! Distance non corrig�e en cas de non-alignement
        avec l'horizontal ... f =
        estimateIntrinsicDistanceZ(m_visionTargetPairs[i].m_center[VisionTargetPair::TRAPEZOID],
        REAL_Y_VISIONTARGETPAIR_CENTER); v0.y += 30; cv::putText(image, to_string(f), v0,
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
        */
    }
}

void Vision::renderHorizon(cv::InputOutputArray image, const unsigned long mode) {
    cv::Point2f v0, v1;

    v0.x = 0.0f;
    v0.y = m_horizon;
    v1.x = (float) image.size().width;
    v1.y = v0.y;
    line(image, v0, v1, Scalar(255, 255, 0));

    // Infos sous la ligne d'Horizon
    if (mode == 1) {
#ifdef __DESKTOP__
        char str[256];

        // HORIZON INFOS
        sprintf(str, "Horizon:%.2f pix ( r=%.2f )", m_horizon, m_horizonRatio);
        v0.y -= 5;
        cv::putText(image, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 0));
        // CAMERA INFOS:
        v0.x = m_screenWidth - 150;
        sprintf(str, "Current Camera ID:%d", m_currentCameraID);
        cv::putText(image, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 128));
        v0.y += 10;
        sprintf(str, "Camera Y/sol:%.3f", m_currentCameraYworld);
        cv::putText(image, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 128));
        // VISION TARGETS SIZE MIN & MAX INFOS
        // Infos area Min / Max
        v0.y = m_horizon + 10;
        sprintf(str, "Area Min: %.3f pix ( r=%.3f )\tArea Max: %.3f pix ( r=%.3f ) ", m_areaMin, m_areaMinRatio,
                m_areaMax, m_areaMaxRatio);
        cv::putText(image, str, v0, cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 0));
#endif

        cv::Rect rect;
        // Rectangle repr�sentant la taille mini.
        rect.x = 5;
        rect.y = (int) (v0.y + 5.0f);  // On veut dessinner des rectangles repr�sentatifs aux proportions exactes
        // d'une Vision Target
        // Surface = w*h,  w = h*REAL_VISIONTARGET_WIDTHHEIGHT_RATIO
        // Surface = h*h*REAL_VISIONTARGET_WIDTHHEIGHT_RATIO
        // h = sqrt(Surface/REAL_VISIONTARGET_WIDTHHEIGHT_RATIO);
        rect.height = (int) sqrtf(m_areaMin / REAL_VISIONTARGET_WIDTHHEIGHT_RATIO);
        rect.width = (int) (REAL_VISIONTARGET_WIDTHHEIGHT_RATIO * rect.height);
        // cv::rectangle(image, rect, cv::Scalar(255, 255, 0));

        // Rectangle repr�sentant la taille maxi.
        rect.x += rect.width + 50;
        rect.height = (int) sqrtf(m_areaMax / REAL_VISIONTARGET_WIDTHHEIGHT_RATIO);
        rect.width = (int) (REAL_VISIONTARGET_WIDTHHEIGHT_RATIO * rect.height);
        // cv::rectangle(image, rect, cv::Scalar(255, 255, 0));
    }
}

void Vision::renderHud(cv::InputOutputArray image, const unsigned long mode) {
    Point2f center;
    Point3f camdir(0.0f, 0.0f, -1.0f);
    Scalar color;
    int radius;

    if (mode == 1)  // undistorded center
    {
        center.x = getCurrentScreenCenterX();
        center.y = getCurrentScreenCenterY();
    } else {
        center.x = (float) image.size().width / 2.0f;
        center.y = (float) image.size().height / 2.0f;
    }
    /*
    if (m_pSelectedVisionTargetPair)
    {
      //Hud::drawVisionTargetPair(image, *m_pSelectedVisionTargetPair);

      // Calcul d'un facteur de qualit� de l'orientation de la paire de vision target par rapport � la cam�ra.
      // ( produit scalaire du vecteur perpendiculaire au plan de la paire de vision target et de l'axe de vis�e
  de la cam�ra ) float fd = camdir.dot(m_pSelectedVisionTargetPair->m_dirVect);
      // On affecte un exposant 4 au facteur obtenu pour obtenir une pente plus grande au voisinage de 1 et donc
  "�tirer" les valeurs sur une plage plus grande. fd = fd * fd*fd*fd;

      // Calcul d'un facteur de qualit� de la distance horizontale de la Paire de vision target avec l'axe de
  vis�e de la cam�ra
      // ( plus la paire est centr�e � l'�cran mieux c'est !)
      float ft = 1.0f - fabs(m_pSelectedVisionTargetPair->m_center[VisionTargetPair::TRAPEZOID_BOTTOM_SIDE].x -
  center.x)/((float)CAPTURED_IMAGE_WIDTH*0.5f);


      fd = fd * ft;

      fd = MIN(1.0f,MAX(0.0f, fd));

      // on utilise le r�sultat pour d�finir la couleur de tracage
      if (fd < 0.8f)
        color = Scalar(0, 0, 200);
      else if (fd < 0.9f)
        color = Scalar(0, 128, 255);
      else if (fd < 0.95f)
        color = Scalar(0, 255, 255);
      else
        color = Scalar(0, 255, 0);

      // ... et pour f�finir le rayon du cercle du viseur
      fd = 1.0f - (fd - 0.75f) / 0.25f;
      radius = (int)(fd * 24.0f) + 8;


      // 3D Alignment Rectangle
      vector<Point3f> p3d;
      vector<Point2f> p2d;
      Mat rvec = Mat(3, 1, CV_64FC1);
      Mat tvec = Mat(3, 1, CV_64FC1);
      Point2f	v0;

      rvec.at<double>(0) = 0;
      rvec.at<double>(1) = 0;
      rvec.at<double>(2) = 0;
      tvec.at<double>(0) = 0;
      tvec.at<double>(1) = 0;
      tvec.at<double>(2) = 0;

      p3d.reserve(12);
      p3d.resize(12);

      p2d.reserve(12);

      // rectangle de gauche
      p3d[0] = m_pSelectedVisionTargetPair->m_pvisionTargetA->m_lower3D; p3d[0].y +=
  HUD_3D_ALIGNMENT_RECT_TOP_Y_SHIFT; p3d[3].x = p3d[0].x +
  m_pSelectedVisionTargetPair->m_dirVect.x*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[3].y = p3d[0].y +
  m_pSelectedVisionTargetPair->m_dirVect.y*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[3].z = p3d[0].z +
  m_pSelectedVisionTargetPair->m_dirVect.z*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[1] = p3d[0]; p3d[1].y +=
  HUD_3D_ALIGNMENT_RECT_BOTTOM_Y_SHIFT; p3d[2].x = p3d[1].x +
  m_pSelectedVisionTargetPair->m_dirVect.x*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[2].y = p3d[1].y +
  m_pSelectedVisionTargetPair->m_dirVect.y*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[2].z = p3d[1].z +
  m_pSelectedVisionTargetPair->m_dirVect.z*HUD_3D_ALIGNMENT_RECT_SIZE_Z;
      // rectangle de droite
      p3d[4] = m_pSelectedVisionTargetPair->m_pvisionTargetB->m_lower3D; p3d[4].y +=
  HUD_3D_ALIGNMENT_RECT_TOP_Y_SHIFT; p3d[7].x = p3d[4].x +
  m_pSelectedVisionTargetPair->m_dirVect.x*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[7].y = p3d[4].y +
  m_pSelectedVisionTargetPair->m_dirVect.y*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[7].z = p3d[4].z +
  m_pSelectedVisionTargetPair->m_dirVect.z*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[5] = p3d[4]; p3d[5].y +=
  HUD_3D_ALIGNMENT_RECT_BOTTOM_Y_SHIFT; p3d[6].x = p3d[5].x +
  m_pSelectedVisionTargetPair->m_dirVect.x*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[6].y = p3d[5].y +
  m_pSelectedVisionTargetPair->m_dirVect.y*HUD_3D_ALIGNMENT_RECT_SIZE_Z; p3d[6].z = p3d[5].z +
  m_pSelectedVisionTargetPair->m_dirVect.z*HUD_3D_ALIGNMENT_RECT_SIZE_Z;

      p3d[8] = m_pSelectedVisionTargetPair->m_pvisionTargetA->m_lower3D;
      p3d[9] = m_pSelectedVisionTargetPair->m_pvisionTargetB->m_lower3D;

      // Axe vertical
      p3d[10] = m_pSelectedVisionTargetPair->m_pvisionTargetB->m_lower3D +
  m_pSelectedVisionTargetPair->m_pvisionTargetA->m_lower3D;



      p3d[10].x *= 0.5f;
      p3d[10].y *= 0.5f;
      p3d[10].z *= 0.5f;



      p3d[11] = p3d[10];
      p3d[11].y -= (REAL_Y_VISIONTARGETPAIR_BOTTOM- REAL_Y_VISIONTARGETPAIR_HATCH);


      projectPoints(p3d, rvec, tvec, m_intrinsic[m_currentCameraID], m_distCoeffs[m_currentCameraID], p2d);
      cv::line(image, p2d[0], p2d[1], color, 1, 4);
      cv::line(image, p2d[1], p2d[2], color, 1, 4);
      cv::line(image, p2d[2], p2d[3], color, 1, 4);
      cv::line(image, p2d[3], p2d[0], color, 1, 4);

      cv::line(image, p2d[4], p2d[5], color, 1, 4);
      cv::line(image, p2d[5], p2d[6], color, 1, 4);
      cv::line(image, p2d[6], p2d[7], color, 1, 4);
      cv::line(image, p2d[7], p2d[4], color, 1, 4);

      cv::line(image, p2d[8], p2d[9], color, 2, 4);
      cv::line(image, p2d[10], p2d[11], color, 2, 4);
  /*
    // calcul et affichage de distance
      cv::Point2f bip[2];
      cv::Point3f r[2];
      float h[2];
      float dist;

      // m�thode avec correction ... � debugger
      bip[0] = m_pSelectedVisionTargetPair->m_pvisionTargetA->m_lower2D;
      bip[1] = m_pSelectedVisionTargetPair->m_pvisionTargetA->m_higher2D;
      h[0] = REAL_Y_VISIONTARGETPAIR_BOTTOM;
      h[1] = REAL_Y_VISIONTARGETPAIR_TOP;
      dist = estimate3DVerticalBipoint(bip, h, r);

      v0.x = center.x;  v0.y = center.y - 5;
      cv::putText(image, to_string(dist), v0, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255));

    }*/
    // else
    //{
    color = Scalar(0, 0, 255);
    radius = 32;
    //}*/

    circle(image, center, radius, color, 2);
    Hud::drawSight2(image, center, 48);
}

void Vision::render3DVisionTargets(cv::InputOutputArray image, const unsigned long mode) {
    vector<Point3f> points3d;
    vector<Point2f> points2d;
    Mat rvec = Mat(3, 1, CV_64FC1);
    Mat tvec = Mat(3, 1, CV_64FC1);
    Point2f v0;

    rvec.at<double>(0) = 0;
    rvec.at<double>(1) = 0;
    rvec.at<double>(2) = 0;

    tvec.at<double>(0) = 0;
    tvec.at<double>(1) = 0;
    tvec.at<double>(2) = 0;

    points3d.reserve(m_visionTargets.size() * 2);
    points2d.reserve(m_visionTargets.size() * 2);
    for (unsigned int i = 0; i < m_visionTargets.size(); i++) {
        points3d.push_back(m_visionTargets[i].m_lower3D);
        points3d.push_back(m_visionTargets[i].m_higher3D);
    }
    if (points3d.size()) {
        projectPoints(points3d, rvec, tvec, m_intrinsic[m_currentCameraID], m_distCoeffs[m_currentCameraID],
                      points2d);
        for (unsigned int i = 1; i < points2d.size(); i += 2) {
            v0 = points2d[i];
            v0.x += 5;
            v0.y -= 5;
            cv::putText(image, to_string(i), v0, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255));

            // points2d[i - 1].y = points2d[i - 1].y - getCurrentScreenCenterY();
            // points2d[i].y = points2d[i].y - getCurrentScreenCenterY();

            Hud::drawPoint(image, points2d[i - 1], 5, Scalar(255, 255, 255));
            Hud::drawPoint(image, points2d[i], 5, Scalar(255, 255, 255));
            cv::line(image, points2d[i - 1], points2d[i], Scalar(255, 0, 255));
        }
    }
}

void Vision::render3DTopView(cv::InputOutputArray image) {
    Point2f topcam;
    Point2f v0, v1;
    Point3f v3d;
    Point2f vtxt;

    // D'abord on efface tout !
    image.setTo(cv::Scalar(0, 0, 0));

    // FENETRE VUE DE TOP 3D

    // -----------------------------------------------------------------------------------------------------------------------------------------------------
    // -----------------------------------------------------------------------------------------------------------------------------------------------------
    // 1) CONE CAMERA:
    // Position de la camera en vue de Haut
    topcam.x = (float) (image.size().width / 2);
    topcam.y = (float) (image.size().height - image.size().height / 4);

    // Pour dessiner la camera et son cone de vision en vue de Top
    // nous allons d�projeter les coins de l'�cran...
    // Axe camera ( = axe de vis�e = axe Z )
    v0.x = (float) (image.size().width / 2);
    v0.y = (float) (image.size().height / 4);
    estimate3DPoint(v0, REAL_Y_CAMERA + 10.0f, v3d);
    v1.x = v3d.x * SCALEFACTOR3D + topcam.x;
    // v1.y = topcam.y - v3d.z*SCALEFACTOR3D;
    v1.y = v3d.z * SCALEFACTOR3D + topcam.y;  // Inversion due � l'inversion de l'axe Y en coordonn�es ecran.
    arrowedLine(image, topcam, v1, Scalar(0, 0, 255));

    // Frustum Camera plan de droite ( pour le dessiner nous deprojetons le coin superieur droit de l'�cran)
    // le resultat est le point 3D v3d, pour le tracer nous ne nous interressons qu'� ses coordonn�es X et Z.
    v0.x = m_screenWidth;
    v0.y = 0;
    estimate3DPoint(v0, REAL_Y_CAMERA + 10.0f, v3d);
    v1.x = v3d.x * SCALEFACTOR3D + topcam.x;
    // v1.y = topcam.y - v3d.z*SCALEFACTOR3D;
    v1.y = v3d.z * SCALEFACTOR3D + topcam.y;  // Inversion due � l'inversion de l'axe Y en coordonn�es ecran.
    cv::line(image, topcam, v1, Scalar(0, 0, 255));

    // Frustum Camera plan de gauche
    // M�me d�marche que pour le plan de droite avec cette fois ci l'angle sup�rieur gauche.
    v0.x = 0;
    v0.y = 0;
    estimate3DPoint(v0, REAL_Y_CAMERA + 10.0f, v3d);
    v1.x = v3d.x * SCALEFACTOR3D + topcam.x;
    // v1.y = topcam.y - v3d.z*SCALEFACTOR3D;
    v1.y = v3d.z * SCALEFACTOR3D + topcam.y;  // Inversion due � l'inversion de l'axe Y en coordonn�es ecran.
    cv::line(image, topcam, v1, Scalar(0, 0, 255));

    // Calcul et representation de l'angle du mur...
    // -----------------------------------------------------------------------------------------------------------------------------------------------------
    // -----------------------------------------------------------------------------------------------------------------------------------------------------
    //
    // 2) ALL VISIONTARGET CANDIDATES POINTS
    vtxt.x = 10;
    vtxt.y = 10;
    for (unsigned int i = 0; i < m_visionTargets.size(); i++) {
        // Points Bas
        v0.x = -m_visionTargets[i].m_lower3D.x * SCALEFACTOR3D + topcam.x;
        // v0.y = topcam.y - m_visionTargets[i].m_lower3D.z*SCALEFACTOR3D;
        v0.y = topcam.y + m_visionTargets[i].m_lower3D.z * SCALEFACTOR3D;
        Hud::drawPoint(image, v0, 5, Scalar(0, 128, 128));

        // Points Haut
        v0.x = -m_visionTargets[i].m_higher3D.x * SCALEFACTOR3D + topcam.x;
        // v0.y = topcam.y - m_visionTargets[i].m_higher3D.z*SCALEFACTOR3D;
        v0.y = topcam.y + m_visionTargets[i].m_higher3D.z * SCALEFACTOR3D;
        Hud::drawPoint(image, v0, 5, Scalar(128, 128, 0));

        v0.x += 5;
        v0.y -= 5;
        cv::putText(image, to_string(i), v0, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255));
    }
}

void Vision::renderMax(cv::InputOutputArray image) {
    cv::Point2f v0, v1;

    v0.x = image.size().width / 6;
    v0.y = image.size().height;

    v1.x = 6 * image.size().width / 20;
    v1.y = 4 * image.size().height / 5;

    cv::line(image, v0, v1, Scalar(150, 255, 0), 2);

    cv::Point2f v2, v3;

    v2.x = 5 * image.size().width / 6;
    v2.y = image.size().height;

    v3.x = 14 * image.size().width / 20;
    v3.y = 4 * image.size().height / 5;

    cv::line(image, v2, v3, Scalar(150, 255, 0), 2);

    cv::line(image, v1, v3, Scalar(150, 255, 0), 2);
}
