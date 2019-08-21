#pragma once

// Attention � la version release FINALE !!! 
//#define PRECOMPIL_EDITING									// Si il est d�fini, Active les controles d'edition et le rendu de fen�tres/images suppl�mentaires.

#define PRECOMPIL_TRAINING_ROBOT                            // Si il est d�fini, switch sur les fichiers de calibrations camera du Robot d'entrainement
// sinon utilise les fichiers de calibration des cameras du robot de competition
// pour LA VERSION RELEASE COMPETITION DU PROGRAMME ce #define ne doit pas �tre d�fini et mis en commentaire !!!!!!

//#define PRECOMPIL_MEASURE_MINIATURE							// Si il est d�fini, switch sur les valeurs MINIATURES des "Real Y"
// ( hauteurs r�elles mesur�es et connues comme la hauteur de la camera et des vision target )
// not� que ce define ne peut �tre d�fini que en mode training.

// IMAGE PROCESS FILTERS
// ---------------------------------------------------------------------------------------------------------------------------------------------------
#define PRECOMPIL_IMAGEFILTER_HLS                            // Si il est d�fini, active le traitement HLS de la vid�o pour extraire les visiontarget
//#define PRECOMPIL_IMAGEFILTER_2GRAY_THRESHOLD				// Si il est d�fini, active le traitement THRESHOLD de la vid�o pour extraire les visiontarget
#define PRECOMPIL_IMAGEFILTER_CANNY

// Filtres logiques utilis�s pour d�cider si une paire de VisionTarget pr�sum�es est valide ( permet d'�liminer les vision target parasites qui n'en sont pas )
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_VISIONTARGETS_ANGLE_TOO_LARGE
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_HIGH_RATIO
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_QUADSIDE_RATIO
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_ANGLE_QUADTOP_HORIZONTAL
//#define PRECOMPIL_LOGICFILTER_VISIONTARGETPAIR_ANGLE_QUADBASE_HORIZONTAL
