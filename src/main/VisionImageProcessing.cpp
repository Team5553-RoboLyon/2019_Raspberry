#include "PrecompilationDefine.h"

#include <math.h>
#include <fstream>
#include <iostream>
#include "Tools.h"
#include "Values.h"

#include "Vision.h"

// find_package(raspicam PATH /usr/local/lib REQUIRED);//(dans le CMakeList)

void Vision::capturePause() {
    BITSET(m_flags, FLAG_CAPTURE_PAUSE);
}

void Vision::captureResume() {
    BITCLEAR(m_flags, FLAG_CAPTURE_PAUSE);
}

void Vision::capturePauseToggle() {
    BITTOGGLE(m_flags, FLAG_CAPTURE_PAUSE);
}

const bool Vision::imageCapture() {
    // Si on demande pause, on retourne true
    if (BITGET(m_flags, Vision::FLAG_CAPTURE_PAUSE)) {
        // Undistord Src Image
        m_imageUndistorded = m_imageSrc.clone();
        return true;
    }

    if (!m_captureDevice.read(m_imageSrc)) {
        std::cout << "Error Grabbing Video Frame" << std::endl;
        return false;
    } else {
        std::cout << std::endl << "+	Grab Video Frame" << std::endl;
        std::cout << "|" << std::endl;

        // Undistord Src Image
        undistort(m_imageSrc, m_imageUndistorded, m_intrinsic[m_currentCameraID],
                  m_distCoeffs[m_currentCameraID]);
        std::cout << "+--->	Undistort Video Frame" << std::endl;
        return true;
    }
}

const bool Vision::imageProcess() {
    //	En entr�e, l'image "Brute" ( sans traitement) captur�e par la camera.
    //	En sortie, l'image "filtr�e" pr�te � �tre analys�e.

    // First Method
#ifdef PRECOMPIL_IMAGEFILTER_HLS
    //########## BGR TO HLS ##########
    cv::cvtColor(m_imageUndistorded, m_imageHLS, cv::COLOR_BGR2HLS);
    //########## Threshold ##########
    cv::inRange(m_imageHLS, Scalar(m_Hlow, m_Llow, m_Slow), Scalar(m_Hhigh, m_Lhigh, m_Shigh), m_imageFiltered);
    // cv::inRange(HlsImage, cv::Scalar(50, 130, 150), cv::Scalar(95, 255, 255), FilteredImage);
    //########## Erode and Dilate ##########
    // test effectu� pas tr�s concluant. A appronfondir plus tard... pour l'instant on met de c�t�.
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(9, 9), cv::Point(-1, -1));
    // cv::morphologyEx(FilteredImage, TestImage, cv::MORPH_OPEN, kernel);
    // cv::imshow("TEST MORPHOLOGY", TestImage);

    // or (instead of Erode and Dilate )
    // FilteredImage.convertTo(FilteredImage, CV_8UC3);
    // cv::cvtColor(FilteredImage, FilteredImage, cv::COLOR_HSV2BGR);

#endif

    // Second Method
#ifdef PRECOMPIL_IMAGEFILTER_2GRAY_THRESHOLD
    // Filtre 1: On passe l'image en niveau de gris.
    cvtColor(m_imageUndistorded, m_imageFiltered, COLOR_BGR2GRAY);
    // Filtre 2: L'image est flout�e ( un peu )
    // GaussianBlur(FilteredImage, FilteredImage, Size(7, 7), 0, 0);
    // Filtre 3: L'image est "�cr�t�e" et passe en deux couleurs : Noir ou Blanc
    // ( le filtre n'est appliqu� que si ses deux param�tres �ditable sont differents de 0 )
    inRange(m_imageFiltered, m_threshMin, m_threshMax, m_imageFiltered);

#endif
#ifdef PRECOMPIL_IMAGEFILTER_CANNY
    // Filtre 4: Filtre sp�cial Canny qui dessinne en blanc sur fond noir les pourtours de l'image.
    // ( le filtre n'est appliqu� que si ses deux param�tres �ditables sont differents de 0 )
    if (m_cannyThreshMin && m_cannyThreshMax) {
        // blur(m_imageFiltered, m_imageFiltered, Size(3, 3));
        Canny(m_imageFiltered, m_imageFiltered, m_cannyThreshMin, m_cannyThreshMax, 3, false);
    }
#endif
    std::cout << "+--->	Process Video Frame" << std::endl;
    return false;
}

void Vision::videoBackRender() {}

void Vision::videoBackRender(const cv::Mat &img) {}
