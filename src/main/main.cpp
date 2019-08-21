#include "PrecompilationDefine.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <networktables/NetworkTableInstance.h>

#include "lib/MjpegStream.h"

#include "Values.h"
#include "Vision.h"

using namespace cv;
using namespace std;

#ifndef RUNNING_FRC_TESTS

int main() {
#ifdef __DESKTOP__
    std::cout << "Running on Desktop - imshow enabled" << std::endl;
#else
    std::cout << "Running embedded - imshow disabled" << std::endl;
#endif

    // Initialization
    Mat imageTopView(CAPTURED_IMAGE_HEIGHT, CAPTURED_IMAGE_WIDTH,
                     CV_8UC3);  // Image 3D calculée à partir des données capturées

    // initialisation de la classe vision
    Vision vision(CAPTURED_IMAGE_WIDTH, CAPTURED_IMAGE_HEIGHT);

    lyonlib::MjpegStream stream_server("Pince Avant", CAPTURED_IMAGE_WIDTH, CAPTURED_IMAGE_HEIGHT, 50);

    auto inst = nt::NetworkTableInstance::GetDefault();
    inst.StartClientTeam(5553);

    if (!vision.loadCameraIntrinsic(0, "data/camera_blanche.txt"))  // Camera Avant
    {
        return 0;
    }

    if (!vision.loadCameraIntrinsic(1, "data/camera_blanche.txt"))  // Camera Arriere
    {
        return 0;
    }

    vision.setCurrentCamera(0);

    // cas échéant, initialisation de l'interface d'édition ( sur PC, pour tester   )
#ifdef PRECOMPIL_EDITING
    Editor editor(&vision);
#endif

    // Boucle de traitement
    while (true) {
        if (vision.imageCapture()) {
            std::cout << vision.m_brightness << std::endl;
            std::cout << vision.m_awb_red << std::endl;
            std::cout << vision.m_awb_blue << std::endl;

#ifdef __DESKTOP__
            vision.renderVisionContours(vision.m_imageSrc);
            vision.renderVisionTargetCandidates(vision.m_imageUndistorded);
            vision.renderVisionTargetPairs(vision.m_imageUndistorded);
            vision.renderHorizon(vision.m_imageUndistorded, 1);
            vision.render3DVisionTargets(vision.m_imageUndistorded, 1);
            vision.render3DTopView(imageTopView);
            imshow("Undistort", vision.m_imageUndistorded);
            imshow("Src", vision.m_imageSrc);
            imshow("TopView", imageTopView);
            if (waitKey(10) == ' ') vision.capturePauseToggle();
#endif

            cv::Mat streamOutput = vision.m_imageUndistorded;
#ifndef __DESKTOP__
            // Code spécifique Raspberry (flip l'image en fonction de la position du pivot)
            if (inst.GetEntry("/SmartDashboard/Angle Pivot").GetDouble(1) < 0.0) {
                cv::flip(streamOutput, streamOutput, -1);
            }
#endif
            vision.imageProcess();
            vision.findVisionTargetPairs();
            vision.getCenteredTargetPair();
            vision.renderHud(vision.m_imageUndistorded, 1);
            vision.renderMax(vision.m_imageUndistorded);
            stream_server.PutFrame(streamOutput);
        }
    }

    return 0;
}

#endif
