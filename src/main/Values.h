#pragma once

#include "PrecompilationDefine.h"



// FLAGS

// Noms de fichiers Externes
#ifdef PRECOMPIL_TRAINING_ROBOT
#define CAMERA0_FILENAME                            "data/training_camera_front_640x480.txt"
#define CAMERA1_FILENAME                            "data/training_camera_back_640x480.txt"
#else
#define CAMERA0_FILENAME							"data/competition_camera_front_320x240.txt"
#define CAMERA1_FILENAME							"data/competition_camera_back_320x240.txt"
#endif


#define VISIONTARGET_FILENAME                        "data/vision_target.txt"        //"D:/_PROJETS/FIRST/C++/vision_target.txt"

// Param�tres des filtres images � l'initialisation
#define FILTRE_CANNY_THRESH1                                    34.32f
#define FILTRE_CANNY_THRESH2                                    104.0f

#define FILTRE_THRESHOLD_MIN                                    150
#define FILTRE_THRESHOLD_MAX                                    255

#define FILTRE_HLS_HLOW                                            40//50
#define FILTRE_HLS_LLOW                                            30//120//70// ou 25 pour voir les d�tails faiblement lumineux ( VT avec angle approchant les 80 Deg avec l'axe de la camera ... ).
#define FILTRE_HLS_SLOW                                            0//20// ..... MAIS ATTENTION RISQUE de faire apparaitre + de parasite sur le InRangeMask...
#define FILTRE_HLS_HHIGH                                        95
#define FILTRE_HLS_LHIGH                                        255
#define FILTRE_HLS_SHIGH                                        255

// Param�tres Avanc�s
#define ADVPARAM_HORIZON_RATIO                                    0.75f    //  r of screen height
#define ADVPARAM_VISIONTARGET_AREA_RATIO_MIN                    0.025f    //  r of image.width x r image.height
#define ADVPARAM_VISIONTARGET_AREA_RATIO_MAX                    0.85f    //  r of image.width x r image.height


// Mesures r�elles


#define REAL_CAMERA_FOV                                            52.0f        // DEGREE Exact FOV of the camera.

#define REAL_VISIONTARGET_WIDTHHEIGHT_RATIO                        (5.08f/13.97f) // Ratio largeur sur hauteur d'une vision target

#ifdef PRECOMPIL_MEASURE_MINIATURE
// Hauteurs miniatures ( maquettes ) Mesures sur la maquette N&B #2
#define REAL_Y_GROUND											0.0f		// Ground is level 0
#define REAL_Y_CAMERA											2.3f		// cm par rapport au sol, hauteur r�elle de la camera

#define REAL_VISIONTARGET_NORM_01								2.35f		// cm longueur du plus grand c�t� d'une vision target
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM				4.7f		// cm

#define REAL_Y_VISIONTARGETPAIR_CENTER							8.8f		// cm par rapport au sol. Point d'intersection des diagonales du VisionTarget QUAD [1],[6],[4],[3] cf "vision_target.txt"
#define REAL_Y_VISIONTARGETPAIR_BOTTOM							7.6f		// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond � l'ordonn�e des point [4],[3]
#define REAL_Y_VISIONTARGETPAIR_TOP								10.1f		// cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond � l'ordonn�e des point [1],[6]
#define REAL_Y_VISIONTARGETPAIR_HATCH							5.0f		// cm par rapport au sol. centre de la "trappe" situ�e sous la paire de vision target
// Valeurs limites pr�calcul�es � partir des donn�es ci-dessus
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMIN		( REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*0.5f*0.5f )	//  = dist�
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMAX		( REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*2.0f*2.0f )	//	= dist�
#else
// Hauteurs r�elles
#define REAL_Y_GROUND                                            0.0f        // Ground is level 0
#define REAL_Y_CAMERA                                            48.2f            // cm par rapport au sol, hauteur relle de la camera

#define REAL_VISIONTARGET_NORM_01                                14.865f        // cm longueur du plus grand c�t� d'une vision target
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM                27.316f        // cm

#define REAL_Y_VISIONTARGETPAIR_CENTER                            72.6116f            // cm par rapport au sol. Point d'intersection des diagonales du VisionTarget QUAD [1],[6],[4],[3] cf "vision_target.txt"
#define REAL_Y_VISIONTARGETPAIR_BOTTOM                            65.2131f    // cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond � l'ordonn�e des point [4],[3]
#define REAL_Y_VISIONTARGETPAIR_TOP                                80.01f        // cm par rapport au sol. Point le plus bas du Vision Target Quad, correspond � l'ordonn�e des point [1],[6]
#define REAL_Y_VISIONTARGETPAIR_HATCH                            48.26f        // cm par rapport au sol. centre de la "trappe" situ�e sous la paire de vision target

#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMIN        ( REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*0.5f*0.5f )    //  = dist�
#define REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMAX        ( REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM*2.0f*2.0f )    //	= dist�

#endif






// Vision Targets Recognition


#define VISIONTARGET1_DOT_VISIONTARGET2_STANDARD                0.968147640378107774966f // Cosinus d'un angle de 14.5 degr�s.
#define VISIONTARGET1_DOT_VISIONTARGET2_UNDERLIMIT                0.86f        // pour d�tecter un angle trop important entre les Visions Targets d'un couple.
// pour un angle de 30� entre les deux vision target ( ce qui correspond environ au double de l'angle r�el) ...
// ...on obtient un Dot Product de 0.8660f ( cos(30 deg) = 0.8660  )

#define VISIONTARGET_COUPLE_VTA_VTB_HEIGHT_RATIO_UNDERLIMIT        0.5f        // Pour d�tecter les differences de taille (hauteur) trop importantes entre les Visions Targets d'un couple

#define VISIONTARGET_COUPLE_QUADSIDE_RATIO_STANDARD                (REAL_VISIONTARGET_N01/REAL_VISIONTARGET_NTOP)        // valeur pour un Couple de face ( = 0.4929...)
#define VISIONTARGET_COUPLE_QUADSIDE_RATIO_UNDERLIMIT            0.4f        // Pour d�tecter les differences de proportion du quadrilatere du couple
#define VISIONTARGET_COUPLE_QUADSIDE_RATIO_MAX                    5.0f        // valeur limite constat�e empiriquement (peut-�tre affin�e)
#define VISIONTARGET_COUPLE_QUADSIDE_NORM_MIN                    0.0001f        // Longueur minimale d'un c�t� du quadrilatere du couple, en dessous on consid�re la longueur comme nulle.

#define VISIONTARGETPAIR_TRAPEZOID_INCLINE_LIMIT                0.9396926207859f    // cosinus de l'angle maximal de la base du trapezoid d'une paire avec l'horizontale.
// 0.9396926207859f		... Cosinus d'un angle de 20 deg
// 0.8660254037844f		... Cosinus d'un angle de 30 deg



#define SCALEFACTOR3D                5.0f                            // Used by TopView::render For test only

#define CAPTURED_IMAGE_WIDTH        320
#define CAPTURED_IMAGE_HEIGHT        240
#define CAPTURED_IMAGE_CONTRAST        0
#define CAPTURED_IMAGE_BRIGHTNESS    50
#define CAPTURED_IMAGE_EXPOSURE        0
#define CAPTURED_IMAGE_GAIN            0
#define CAPTURED_IMAGE_SATRUATION    0
#define CAPTURED_IMAGE_AWB_BLUE        0.015
#define CAPTURED_IMAGE_AWB_RED        0.015


#define STREAM_PORT    8081
#define STREAM_FPS    60






