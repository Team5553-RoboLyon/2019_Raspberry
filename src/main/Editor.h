#pragma once

#include "opencv2/opencv.hpp"
#include "Vision.h"

class Editor {

public:
    Editor(Vision *pvision);

    int m_cursorCameraY;
    int m_cursorBrigthness;
    int m_cursorContrast;
    int m_exposure;
    int m_cursorGain;

    int m_cursorAreaMinRatio;
    int m_cursorAreaMaxRatio;
    int m_cursorHorizonRatio;
#ifdef PRECOMPIL_IMAGEFILTER_HLS
    int m_cursorHlow;
    int m_cursorLlow;
    int m_cursorSlow;
    int m_cursorHhigh;
    int m_cursorLhigh;
    int m_cursorShigh;
#endif
#ifdef PRECOMPIL_IMAGEFILTER_2GRAY_THRESHOLD
    int m_cursorThreshMin;
    int m_cursorThreshMax;
#endif

#ifdef PRECOMPIL_IMAGEFILTER_CANNY
    int m_cursorCannyThreshMinRatio;
    int m_cursorCannyThreshMax;
#endif


};