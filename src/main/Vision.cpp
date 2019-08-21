#include "PrecompilationDefine.h"

#include <math.h>
#include <iostream>
#include <fstream>
#include "Vision.h"
#include "Tools.h"
#include "Values.h"


Vision::Vision(const float screenwidth, const float screenheight) {
    m_intrinsic[0] = Mat(3, 3, CV_64FC1);
    m_intrinsic[1] = Mat(3, 3, CV_64FC1);


    m_currentCameraYworld = REAL_Y_CAMERA;
    m_screenWidth = screenwidth;
    m_screenHeight = screenheight;
    m_brightness = CAPTURED_IMAGE_BRIGHTNESS;
    m_contrast = CAPTURED_IMAGE_CONTRAST;
    m_exposure = CAPTURED_IMAGE_EXPOSURE;
    m_gain = CAPTURED_IMAGE_GAIN;
    m_awb_red = CAPTURED_IMAGE_AWB_RED;
    m_awb_blue = CAPTURED_IMAGE_AWB_BLUE;
    m_saturation = CAPTURED_IMAGE_SATRUATION;

    m_currentCameraID = 0xffff;


    // valeur par d�faut des matrices intrinsic
    // on utilisera une focale par defaut de 60 degr�s
    double tangente = tan(DEGtoRAD(60.0));

    // | focalDistX		0		scrCenterX	|
    // | 	0		focalDistY	scrCenterY	|
    // | 	0			0			 1		|
    //
    m_intrinsic[0].at<double>(0, 0) = 0.5 * screenwidth / tangente; // focaldistx
    m_intrinsic[0].at<double>(0, 1) = 0.0;
    m_intrinsic[0].at<double>(0, 2) = screenwidth * 0.5;

    m_intrinsic[0].at<double>(1, 0) = 0.0;
    m_intrinsic[0].at<double>(1, 1) = 0.5 * screenheight / tangente; // focaldisty
    m_intrinsic[0].at<double>(1, 2) = screenheight * 0.5;

    m_intrinsic[0].at<double>(2, 0) = 0.0;
    m_intrinsic[0].at<double>(2, 1) = 0.0;
    m_intrinsic[0].at<double>(2, 2) = 1.0;

    cout << "M(0) = " << m_intrinsic[0].at<double>(0) << "\tM(0,0) = " << m_intrinsic[0].at<double>(0, 0) << endl;
    cout << "M(1) = " << m_intrinsic[0].at<double>(1) << "\tM(0,1) = " << m_intrinsic[0].at<double>(0, 1) << endl;
    cout << "M(2) = " << m_intrinsic[0].at<double>(2) << "\tM(0,2) = " << m_intrinsic[0].at<double>(0, 2) << endl;

    cout << "M(3) = " << m_intrinsic[0].at<double>(3) << "\tM(1,0) = " << m_intrinsic[0].at<double>(1, 0) << endl;
    cout << "M(4) = " << m_intrinsic[0].at<double>(4) << "\tM(1,1) = " << m_intrinsic[0].at<double>(1, 1) << endl;
    cout << "M(5) = " << m_intrinsic[0].at<double>(5) << "\tM(1,2) = " << m_intrinsic[0].at<double>(1, 2) << endl;

    cout << "M(6) = " << m_intrinsic[0].at<double>(6) << "\tM(2,0) = " << m_intrinsic[0].at<double>(2, 0) << endl;
    cout << "M(7) = " << m_intrinsic[0].at<double>(7) << "\tM(2,1) = " << m_intrinsic[0].at<double>(2, 1) << endl;
    cout << "M(8) = " << m_intrinsic[0].at<double>(8) << "\tM(2,2) = " << m_intrinsic[0].at<double>(2, 2) << endl;


    m_intrinsic[1] = m_intrinsic[0];

    m_areaMinRatio = ADVPARAM_VISIONTARGET_AREA_RATIO_MIN;    // means a% of each screen dimension >>> (a% of screen width) * (a% of screen height)
    m_areaMaxRatio = ADVPARAM_VISIONTARGET_AREA_RATIO_MAX;    // means a% of each screen dimension >>> (a% of screen width) * (a% of screen height)
    m_horizonRatio = ADVPARAM_HORIZON_RATIO;                    // means a% of screen height

    m_areaMin = (m_screenHeight * m_areaMinRatio) * (m_screenWidth * m_areaMinRatio);
    m_areaMax = (m_screenHeight * m_areaMaxRatio) * (m_screenWidth * m_areaMaxRatio);
    m_horizon = m_screenHeight * m_horizonRatio;

    loadCameraParam("data/cameraparam.txt");
    setCurrentCamera(0); // pour permettre le chargement des valeurs depuis le fichier


#ifdef PRECOMPIL_IMAGEFILTER_HLS
    m_Hlow = FILTRE_HLS_HLOW;
    m_Llow = FILTRE_HLS_LLOW;
    m_Slow = FILTRE_HLS_SLOW;
    m_Hhigh = FILTRE_HLS_HHIGH;
    m_Lhigh = FILTRE_HLS_LHIGH;
    m_Shigh = FILTRE_HLS_SHIGH;
#endif

#ifdef PRECOMPIL_IMAGEFILTER_2GRAY_THRESHOLD
    m_threshMin = FILTRE_THRESHOLD_MIN;
    m_threshMax = FILTRE_THRESHOLD_MAX;
#endif

#ifdef PRECOMPIL_IMAGEFILTER_CANNY
    m_cannyThreshMin = FILTRE_CANNY_THRESH1;
    m_cannyThreshMax = FILTRE_CANNY_THRESH2;
#endif

    m_pSelectedVisionTargetPair = nullptr;


    m_flags = 0;
}

Vision::~Vision() {}

// cv::Point3f *res					pointeur sur triplet de float, ou sera stocker les coordonn�es du point.
// const cv::Point2f *screenpoint	pointeur sur les coordonn�es 2D d'un point � l'�cran.
// const double planeY				hauteur du plan horizontal dans le "monde". c'est � dire une hauteur par rapport au sol.



void Vision::setCurrentCamera(const unsigned long id) {
    assert(id < 2);

    if (m_currentCameraID != id) {
        // Switcher ici de camera:
        //
        // .....
        //
        // Choses � faire au niveau de l'init de la capture pour travailler maintenant avec la nouvelle camera
        //
        // .....
        //

        switch (id) {
            // Camera AVANT:
            case 0:
                // pas besoin d'appeler m_captureDevice.release(); La m�thode sera appel�e directement par m_captureDevice.open...
                m_captureDevice.open(0);

                m_captureDevice.set(cv::CAP_PROP_FORMAT, CV_8UC3);
                m_captureDevice.set(cv::CAP_PROP_MODE, 7);
                m_captureDevice.set(cv::CAP_PROP_FRAME_WIDTH, m_screenWidth);
                m_captureDevice.set(cv::CAP_PROP_FRAME_HEIGHT, m_screenHeight);
                m_captureDevice.set(cv::CAP_PROP_FPS, 50);

                m_captureDevice.set(cv::CAP_PROP_BRIGHTNESS, m_brightness);
                m_captureDevice.set(cv::CAP_PROP_CONTRAST, m_contrast);
                m_captureDevice.set(cv::CAP_PROP_EXPOSURE, m_exposure);
                m_captureDevice.set(cv::CAP_PROP_GAIN, m_gain);
                m_captureDevice.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, m_awb_red);
                m_captureDevice.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, m_awb_blue);
                m_captureDevice.set(cv::CAP_PROP_SATURATION, m_saturation);

                break;

                // Camera ARRIERE:
            case 1:
                /*
                // pas besoin d'appeler m_captureDevice.release(); La m�thode sera appel�e directement par m_captureDevice.open...
                m_captureDevice.open(1);
                //m_captureDevice.open("D:/_PROJETS/FIRST/Image/videorobot.mp4");

                m_captureDevice.set(cv::CAP_PROP_BRIGHTNESS, m_brightness);
                m_captureDevice.set(cv::CAP_PROP_CONTRAST, m_contrast);
                m_captureDevice.set(cv::CAP_PROP_EXPOSURE, m_exposure);
                m_captureDevice.set(cv::CAP_PROP_GAIN, m_gain);

                m_captureDevice.set(cv::CAP_PROP_FRAME_WIDTH, m_screenWidth);
                m_captureDevice.set(cv::CAP_PROP_FRAME_HEIGHT, m_screenHeight);
                m_captureDevice.set(cv::CAP_PROP_FPS, 50);
                */
                break;

            default:
                break;
        }

        // on enregistre le nouvel id comme id courant
        m_currentCameraID = id;
    }
}


void Vision::setCameraYWorld(const float y) {
    m_currentCameraYworld = y;
}

void Vision::setCameraBrigthness(const double b) {
    m_brightness = b;
    m_captureDevice.set(cv::CAP_PROP_BRIGHTNESS, m_brightness);
}

void Vision::setCameraContrast(const double c) {
    m_contrast = c;
    m_captureDevice.set(cv::CAP_PROP_CONTRAST, m_contrast);
}

void Vision::setCameraExposure(const double e) {
    m_exposure = e;
    m_captureDevice.set(cv::CAP_PROP_EXPOSURE, m_exposure);
}

void Vision::setCameraGain(const double g) {
    m_gain = g;
    m_captureDevice.set(cv::CAP_PROP_GAIN, m_gain);
}

float Vision::getCameraYWorld() {
    return m_currentCameraYworld;
}

double Vision::getCameraBrigthness() {
    return m_brightness;
}

double Vision::getCameraContrast() {
    return m_contrast;
}

double Vision::getCameraExposure() {
    return m_exposure;
}

double Vision::getCameraGain() {
    return m_gain;
}


const bool Vision::estimate3DPoint(const cv::Point2f &point, const float realYFromTheGround, Point3f &resultat3D) {

    // Test pr�alable, on ne pourra rien faire si le point 2D � une ordonn�e camera nulle ...
    if (fabs(getCurrentScreenCenterY() - point.y) <= EPSILON_2DCOORD_f32)
        return false;

    // Pour faire ces calculs on se base sur les relations simples qui relie les coordonn�es 3Drelatives � la camera aux coordonn�es 2D du point  projet�.
    // X,Y,Z coordonn�es 3D du point A par rapport � la camera.
    // xecran,yecran coordonn�es ecran  de la projection de A.
    // xc,yc coordonn�es 2D de la projection A par rapport au centre ( corrig�) de l'�cran. Le centre de l'�cran coincide avec l'axe Z de la camera ( l'axe de vis�e )
    // Ox,Oy coordonn�es du centre de l'�cran ( corrig�)
    // Fx et Fy Distance focalex et distance focaley (corrig�es)
    // les relation de base sont:
    //								xc = xecran - Ox
    //								yc = Oy - yecran ( et non pas "yecran - Oy" car l'axe Y de l'ecran pointe vers le bas !!! donc on redresse )
    //
    //								X/Z = xc/Fx
    //								Y/Z = yc/Fy
    //
    // On connait xc,yc,Fx,Fy et Y !
    // donc on peut tout connaitre :)
    //								d'abord Z � partir de Y
    //
    //								Z = Y*Fy/yc
    //
    //								puis X � partir de Z
    //
    //								X = Z*xc/Fx
    //
    // C'est parti !
    //
    // On calcul d'abord Y ( Y correspond � la hauteur du point3D PAR RAPPORT A LA CAMERA ).
    // On connait la hauteur r�elle du point par rapport au sol et la hauteur de la camera par rapport au sol...
    // donc c'est facile !:
    resultat3D.y = realYFromTheGround - m_currentCameraYworld;

    // On retrouve Z a partir de Y ( merci Thales ) et des donn�es Intrinsic de calibrage cam�ra ( distance focale y et centreY corrig� de l'ecran  ).

    //resultat3D.z = (resultat3D.y*getCurrentCameraFocalDistY()) / (getCurrentScreenCenterY() - point.y );
    resultat3D.z = (resultat3D.y * getCurrentCameraFocalDistY()) / (point.y - getCurrentScreenCenterY());

    // Et enfin X � partir de Z ( merci Thales-bis) et des donn�es Intrinsic de calibrage cam�ra ( distance focale x et centreX corrig� de l'ecran  ).
    resultat3D.x = (resultat3D.z * (point.x - getCurrentScreenCenterX())) / getCurrentCameraFocalDistX();

    return true;
}

/*
const bool Vision::estimate3DVerticalBipoint(const cv::Point2f(&point)[2], const float(&realYFromTheGround)[2], Point3f(&resultat3D)[2])
{
	// Test pr�alable, on ne pourra rien faire si un des deux point 2D � une ordonn�e camera nulle ...
	if ((fabs(getCurrentScreenCenterY() - point[0].y) <= EPSILON_2DCOORD_f32) || (fabs(getCurrentScreenCenterY() - point[1].y) <= EPSILON_2DCOORD_f32))
		return false;


	// La d�marche est identique � celle de la fonction "estimate3DPoint" avec un "PLUS" important li� � "verticalit�" r�elle et connue des deux points.
	// "Vertical" signie que leur coordonn�es 3D X et Z seront identiques.
	// Cette propri�t� va nous permettre de d�tecter un �ventuel angle haut-bas de la camera ( li� � une erreur d'alignement, car la camera est cens�e �tre parfaitement align�e et horizontale ... ;) )
	// ... et surtout de prendre en compte cette erreur dans les calculs. Le r�sultat sera une plus grande pr�cision sur l'estimation des coordonn�es 3D du bipoint et plus particuli�rement du Z !

	// Etape 1) Premiere estimation de Z
	// d'abord les Y ( facile on les connait )
	resultat3D[0].y = realYFromTheGround[0] - m_currentCameraYworld;
	resultat3D[1].y = realYFromTheGround[1] - m_currentCameraYworld;
	// On retrouve les Z a partir des Y ( merci Thales ) et des donn�es Intrinsic de calibrage cam�ra ( distance focale y et centreY corrig� de l'ecran  ).
	resultat3D[0].z = (resultat3D[0].y*getCurrentCameraFocalDistY()) / ( point[0].y - getCurrentScreenCenterY() );
	resultat3D[1].z = (resultat3D[1].y*getCurrentCameraFocalDistY()) / ( point[1].y - getCurrentScreenCenterY() );
	// Le bipoint 3D est suppos� �tre vertical ! les Z sont donc cens�s �tre identiques !
	float erreur = resultat3D[0].z - resultat3D[1].z;

	if ( fabs(erreur) > TOLERANCE_3DCOORD_f32)
	{
		// En cas d'erreur d�tect�e, on suppose que l'erreur provient d'un mauvais alignement de la camera et on corrige.
		// pour �viter les ralentissement on ne fait q'une seule passe ( pas d'it�rations successives )
		float angDec = asinf(erreur / (fabs(resultat3D[0].y - resultat3D[1].y));

		// connaissant l'angle de d�calage on peut maintenant "recaler" en effet les �galit�s:
		//
		//								X/Z = xc/Fx
		//								Y/Z = yc/Fy
		//
		// Ces �galit�s sont en relation avec l'angle vertical et l'angle horizontal ( "camera vers point vis�", "axe de vis�e de la camera" ).
		// Si on appelle l'angle Vertical angV, et l'angle Horizontal angH on a,
		//
		//								X/Z = xc/Fx = Tan(angH)
		//								Y/Z = yc/Fy = Tan(angV)
		//
		// On vient de voir que la camera ne visait pas horizontalement, elle pointe trop haut ou trop bas et on a trouver l'angle vertical de d�calage.
		// Appellons angV0 l'angle mesur� initialement, anvDec l'angle de d�calage vertical de la camera et angV l'angle qu'on aurait du mesurer si la camera �tait parfaitement align�e
		// on a
		//								angV = angV0 - angDec
		//
		// ... On soustrait simplement l'angle de decalage aux angles mesur�s.
		// ( Pour r�cup�rer les angles mesur�s on fait appel a la fonction arctangente )
		//
		float angV0 = atanf( resultat3D[0].y / resultat3D[0].z ); // = atanf((getCurrentScreenCenterY() - point[0].y) / getCurrentCameraFocalDistY());
		float angV = angV0 - angDec;

		// Et on applique la m�me logique, mais cette fois avec l'angle corrig� !
		//
		//								Y/Z = Tan(angV)
		//								Z	= Y / Tan(angV)
		resultat3D[0].z = resultat3D[0].y / tanf(angV);

		// ... m�me chose pour le second point !
		angV0 = atanf(resultat3D[1].y / resultat3D[1].z); // = atanf((getCurrentScreenCenterY() - point[1].y) / getCurrentCameraFocalDistY());
		angV = angV0 - angDec;
		resultat3D[1].z = resultat3D[1].y / tanf(angV);
	}

	// Et enfin X � partir de Z ( merci Thales-bis) et des donn�es Intrinsic de calibrage cam�ra ( distance focale x et centreX corrig� de l'ecran  ).
	resultat3D[0].x = (resultat3D[0].z*(point[0].x - getCurrentScreenCenterX())) / getCurrentCameraFocalDistX();
	resultat3D[1].x = (resultat3D[1].z*(point[1].x - getCurrentScreenCenterX())) / getCurrentCameraFocalDistX();

	return true;
}
*/
const float Vision::estimate3DVerticalBipoint(const cv::Point2f(&point)[2], const float(&realYFromTheGround)[2],
                                              Point3f(&resultat3D)[2]) {
    // Test pr�alable, on ne pourra rien faire si un des deux point 2D � une ordonn�e camera nulle ...
    if ((fabs(getCurrentScreenCenterY() - point[0].y) <= EPSILON_2DCOORD_f32) ||
        (fabs(getCurrentScreenCenterY() - point[1].y) <= EPSILON_2DCOORD_f32))
        return false;


    // La d�marche est identique � celle de la fonction "estimate3DPoint" avec un "PLUS" important li� � "verticalit�" r�elle et connue des deux points.
    // "Vertical" signie que leur coordonn�es 3D X et Z seront identiques.
    // Cette propri�t� va nous permettre de d�tecter un �ventuel angle haut-bas de la camera ( li� � une erreur d'alignement, car la camera est cens�e �tre parfaitement align�e et horizontale ... ;) )
    // ... et surtout de prendre en compte cette erreur dans les calculs. Le r�sultat sera une plus grande pr�cision sur l'estimation des coordonn�es 3D du bipoint et plus particuli�rement du Z !

    // Etape 1) Premiere estimation de Z
    // d'abord les Y ( facile on les connait )
    resultat3D[0].y = realYFromTheGround[0] - m_currentCameraYworld;
    resultat3D[1].y = realYFromTheGround[1] - m_currentCameraYworld;
    // On retrouve les Z a partir des Y ( merci Thales ) et des donn�es Intrinsic de calibrage cam�ra ( distance focale y et centreY corrig� de l'ecran  ).
    resultat3D[0].z = (resultat3D[0].y * getCurrentCameraFocalDistY()) / (point[0].y - getCurrentScreenCenterY());
    resultat3D[1].z = (resultat3D[1].y * getCurrentCameraFocalDistY()) / (point[1].y - getCurrentScreenCenterY());
    // Le bipoint 3D est suppos� �tre vertical ! les Z sont donc cens�s �tre identiques !
    float erreur = resultat3D[1].z - resultat3D[0].z;

    if (fabs(erreur) > TOLERANCE_3DCOORD_f32) {
        // En cas d'erreur d�tect�e, on suppose que l'erreur provient d'un mauvais alignement de la camera et on corrige.
        // pour �viter les ralentissement on ne fait q'une seule passe ( pas d'it�rations successives )
        float angsin = erreur / (resultat3D[0].y - resultat3D[1].y);
        float angDec = asinf(angsin);
        // connaissant l'angle de d�calage on peut maintenant "calculer" le "vrai" Z commun aux deux points du bipoint:
        float z = resultat3D[0].z * cosf(angDec) + (resultat3D[0].y - resultat3D[1].y) * tanf(angDec);
        resultat3D[0].z = resultat3D[1].z = z;
    }

    // Et enfin X � partir de Z ( merci Thales-bis) et des donn�es Intrinsic de calibrage cam�ra ( distance focale x et centreX corrig� de l'ecran  ).
    resultat3D[0].x = (resultat3D[0].z * (point[0].x - getCurrentScreenCenterX())) / getCurrentCameraFocalDistX();
    resultat3D[1].x = (resultat3D[1].z * (point[1].x - getCurrentScreenCenterX())) / getCurrentCameraFocalDistX();

    return resultat3D[0].z;
}


void writePoint3f(Point3f &p, ofstream &fout) {
    fout << p.x << p.y << p.z << endl;
}

void readPoint3f(Point3f &p, ifstream &fin) {
    fin >> p.x >> p.y >> p.z;
}

void writeMat(Mat &m, ofstream &fout) {
    fout << m.rows << " " << m.cols << " " << m.type() << endl;

    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            fout << m.at<double>(i, j) << "\t";
        }
        fout << endl;
    }
}

void readMat(Mat &m, ifstream &fin) {
    int r, c, t;
    fin >> r >> c >> t;
    m = Mat(r, c, t);

    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            fin >> m.at<double>(i, j);
        }
    }
}


const bool Vision::loadCameraIntrinsic(const unsigned long camid, const char *filename) {
    assert(camid < 2);

    // Fichier cr�� et enregistr� par le programme de calibration de camera.
    ifstream fin(filename, ios::in);  //d�claration du flux et ouverture du fichier
    if (fin)  // si l'ouverture a r�ussi
    {
        float w, h;

        fin >> w >> h;

        if ((w != m_screenWidth) || (h != m_screenHeight)) {
            // sinon
            cerr << "READ FILE ERROR. [ " << filename << " ]"
                 << " !!!! Inconsistency between Camera Intrinsic Data and Screen size !!!! " << endl;
            // fermeture du fichier
            fin.close();
            return false;
        }

        unsigned int vecs_size;
        // Lecture Intrinsic data
        readMat(m_intrinsic[camid], fin);
        readMat(m_distCoeffs[camid], fin);

        fin >> vecs_size;
        m_rvecs[camid].resize(vecs_size);
        for (int i = 0; i < m_rvecs[camid].size(); i++)
            readMat(m_rvecs[camid][i], fin);

        fin >> vecs_size;
        m_tvecs[camid].resize(vecs_size);
        for (int i = 0; i < m_tvecs[camid].size(); i++)
            readMat(m_tvecs[camid][i], fin);

        // fermeture du fichier
        fin.close();

        cout << "Camera " << camid << "Intrinsic Data LOADED" << endl;

        cout << "focal Dist X:" << getCurrentCameraFocalDistX() << endl;
        cout << "focal Dist Y:" << getCurrentCameraFocalDistY() << endl;
        cout << "Center x:" << getCurrentScreenCenterX() << endl;
        cout << "Center y:" << getCurrentScreenCenterY() << endl;

        cout << "M(0) = " << m_intrinsic[0].at<double>(0) << "\tM(0,0) = " << m_intrinsic[0].at<double>(0, 0) << endl;
        cout << "M(1) = " << m_intrinsic[0].at<double>(1) << "\tM(0,1) = " << m_intrinsic[0].at<double>(0, 1) << endl;
        cout << "M(2) = " << m_intrinsic[0].at<double>(2) << "\tM(0,2) = " << m_intrinsic[0].at<double>(0, 2) << endl;

        cout << "M(3) = " << m_intrinsic[0].at<double>(3) << "\tM(1,0) = " << m_intrinsic[0].at<double>(1, 0) << endl;
        cout << "M(4) = " << m_intrinsic[0].at<double>(4) << "\tM(1,1) = " << m_intrinsic[0].at<double>(1, 1) << endl;
        cout << "M(5) = " << m_intrinsic[0].at<double>(5) << "\tM(1,2) = " << m_intrinsic[0].at<double>(1, 2) << endl;

        cout << "M(6) = " << m_intrinsic[0].at<double>(6) << "\tM(2,0) = " << m_intrinsic[0].at<double>(2, 0) << endl;
        cout << "M(7) = " << m_intrinsic[0].at<double>(7) << "\tM(2,1) = " << m_intrinsic[0].at<double>(2, 1) << endl;
        cout << "M(8) = " << m_intrinsic[0].at<double>(8) << "\tM(2,2) = " << m_intrinsic[0].at<double>(2, 2) << endl;
        return true;
    } else {
        // sinon
        cerr << "OPEN FILE ERROR. [ " << filename << " ]" << endl;
        return false;
    }
}

const bool Vision::saveCameraIntrinsic(const unsigned long camid, const char *filename) {
    ofstream fout(filename, ios::out | ios::trunc);  //d�claration du flux et ouverture du fichier
    if (fout)  // si l'ouverture a r�ussi
    {
        fout << m_screenWidth << m_screenHeight << endl;

        // Ecriture Intrinsic data
        writeMat(m_intrinsic[camid], fout);
        writeMat(m_distCoeffs[camid], fout);
        fout << m_rvecs[camid].size() << endl;
        for (int i = 0; i < m_rvecs[camid].size(); i++)
            writeMat(m_rvecs[camid][i], fout);
        fout << m_tvecs[camid].size() << endl;
        for (int i = 0; i < m_tvecs[camid].size(); i++)
            writeMat(m_tvecs[camid][i], fout);

        // Ecriture Pied de page
        fout << "# ROBO'LYON CAMERA CALIBRATION DATA -FIRST 2019- #" << endl;
        fout << "# VISION::CAMERA ID #" << camid << endl;


        // instructions
        fout.close();  // on referme le fichier
        return true;
    } else {
        // sinon
        cerr << "Erreur � l'ouverture !" << endl;
        return false;
    }

}

const bool Vision::loadVisionTargetModel(const char *filename) {
    // Fichier cr�� manuellement depuis un export "ASE" depuis 3DSMAX.
    ifstream fin(filename, ios::in);  //d�claration du flux et ouverture du fichier
    if (fin)  // si l'ouverture a r�ussi
    {
        unsigned int nb;

        // First read the number of vertex to read
        fin >> nb;

        m_visionTarget3DModel.clear();
        m_visionTarget3DModel.reserve(nb);

        //then read all the value and push them into the vision target 3D model
        Point3f p;
        for (unsigned int i = 0; i < nb; i++) {
            fin >> p.x >> p.y >> p.z;
            m_visionTarget3DModel.push_back(p);
        }
        // fermeture du fichier
        fin.close();
        return true;
    } else {
        // sinon
        cerr << "OPEN FILE ERROR. [ " << filename << " ]" << endl;
        return false;
    }

}

const bool Vision::loadCameraParam(const char *filename) {
    // Fichier cr�� manuellement .
    ifstream fin(filename, ios::in);  //d�claration du flux et ouverture du fichier
    if (fin)  // si l'ouverture a r�ussi
    {
        double awb_red, awb_blue, brightness, contrast, saturation, exposure, gain;

        // First read the number of vertex to read
        fin >> awb_red;
        fin >> awb_blue;
        fin >> brightness;
        fin >> contrast;
        fin >> saturation;
        fin >> exposure;
        fin >> gain;

        m_brightness = brightness;
        m_contrast = contrast;
        m_exposure = exposure;
        m_gain = gain;
        m_awb_red = awb_red;
        m_awb_blue = awb_blue;
        m_saturation = saturation;


        // fermeture du fichier
        fin.close();
        return true;
    } else {
        // sinon
        cerr << "OPEN FILE ERROR. [ " << filename << " ]" << endl;
        return false;
    }
}


const float Vision::getCurrentCameraFocalDistX() {
    return (float) m_intrinsic[m_currentCameraID].at<double>(0);// (0, 0);
}

const float Vision::getCurrentCameraFocalDistY() {
    return (float) m_intrinsic[m_currentCameraID].at<double>(4);// (1, 1);
}

const float Vision::getCurrentScreenCenterX() {
    return (float) m_intrinsic[m_currentCameraID].at<double>(2);// (0, 2);
}

const float Vision::getCurrentScreenCenterY() {
    return (float) m_intrinsic[m_currentCameraID].at<double>(5);// (1, 2);
}

const float Vision::estimateIntrinsicHorizontalAngleRad(const cv::Point2f &p) {
    //return atan( (p.x - getCurrentScreenCenterX()) / getCurrentCameraFocalDistX() );
    return (float) atan(
            (p.x - m_intrinsic[m_currentCameraID].at<double>(2)) / m_intrinsic[m_currentCameraID].at<double>(0));
}

const float Vision::estimateIntrinsicVerticalAngleRad(const cv::Point2f &p) {
    //return atan( (p.y - getCurrentScreenCenterY()) / getCurrentCameraFocalDistY() );
    return (float) atan(
            (p.y - m_intrinsic[m_currentCameraID].at<double>(5)) / m_intrinsic[m_currentCameraID].at<double>(4));
}

const float Vision::estimateIntrinsicDistanceZ(const cv::Point2f &p, const float realHeightFromTheGround) {
    // On connait la hauteur de la camera par rapport au sol:
    // ( realHeightFromTheGround - m_cameraYworld ) = Hauteur r�elle par rapport � la cam�ra du point point de l'espace projetant p
    // return ( ( (realHeightFromTheGround - m_currentCameraYworld)*getCurrentCameraFocalDistY() ) / (getCurrentScreenCenterY() - p.y) );
    return (float) (((realHeightFromTheGround - m_currentCameraYworld) * m_intrinsic[m_currentCameraID].at<double>(4)) /
                    (m_intrinsic[m_currentCameraID].at<double>(5) - p.y));
}


const unsigned long Vision::extractVisionTargetCandidates() {
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ETAPE 2/3 : ANALYSE DE L'IMAGE FILTREE ET DETECTION DE CONTOUR
    //			En entr�e, l'image "filtr�e"
    //			En sortie, une liste de "contours" d�tect�s par OpenCV
    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    assert(m_contours.size() == 0);
    cv::findContours(m_imageFiltered, m_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::cout << "+--->	Extract Contours - Found: " << m_contours.size() << std::endl;

    // ---------------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------------
    // ETAPE 3	ANALYSE DE LA LISTE DES CONTOURS ET IDENTIFICATION DES "VISION TARGET"
    //			En sortie d'�tape 2 on se retrouve avec une liste de contours "bruts".
    //			En fait, avec tous les contours que OpenCV � pu trouver...
    //			Potentiellement cette liste contient des contours "parasites" comme des surfaces blanches, spots, flash, etc ...
    //			qui ne sont pas les Vision target refl�chissantes que nous voulons rep�rer ...
    //			Le but de cette 3�me �tape est d'essayer de rep�rer les contours qui s'apparentent le plus � des Visions target.
    //
    //			Nous allons donc appliquer une s�rie de test sur chaque contour de la liste et d�cider si ce dernier peut "�tre ou ne pas �tre" ... une Vision Target !
    //			Il est important de noter que pour cette analyse chaque contour est test� ind�pendement des autres.
    //
    //				-test 0: test de hauteur apparente.		>>> Si le contour est trop bas dans l'image, c'est � dire en dessous d'une limite arbitraire qu'on appelle horizon  ... on passe.
    //				-test 1: test de Taille (surface)		>>> Si le contour est trop petit, ou trop grand... on passe. ( valeur min et max fix�es )
    //				-test 2: test de forme et d'orientation	>>> Si le contour a des proportions qui ne "collent pas" avec les proportions que pourrait avoir un scotch reflechissant ( 5cm X 14.5cm )
    //															m�me d�form� par la perspective ... on passe.
    //														>>> Si le contour n'est pas un rectangle "vertical" ( avec son plus grand c�t� plut�t dans la hauteur ) ... on passe

    assert(m_visionTargets.size() == 0);

    Rect rect;
    RotatedRect rotrect;
    VisionTarget shape;
    Point2f vertices[4];
    Point2f v01, v03;
    Point2f u, v;
//	float			unorm, vnorm;
    float area;
//	float proportion;
    float test;

    // ---------------------------------------------------------------------------------------------
    // "Boucle d�cisionnelle" :
    //		Un contour est-il une vision target ?
    //		Si oui, on cr�e et stocke une VisionTarget et on la "push" dans le tableau des Vision Targets
    float norm01, norm03;
    for (unsigned int i = 0; i < m_contours.size(); i++) {
        rotrect = minAreaRect(m_contours[i]);
        rotrect.points(vertices);

        // test 0: Under Horizon limit ?
        if (vertices[0].y > m_horizon)
            continue;

        // test 1: Too small ?
        area = rotrect.size.height * rotrect.size.width;
        if ((area < m_areaMin) || (area > m_areaMax))
            continue;

        //shape.boundingRect = boundingRect(contours[i]);
        // Bad proportion ?
        //proportion = MAX(rotrect.size.height, rotrect.size.width) / MIN(rotrect.size.height, rotrect.size.width);
        //if (proportion < 2.25f/* || proportion > 3.75f*/)
        //	continue;


        // Not a "vertical bar" ?
        // Quel est le plus long c�t� ? 01 or 03

        // calcul des deux vecteur 01 et 03
        rotrect.points(vertices);
        v01.x = vertices[1].x - vertices[0].x;
        v01.y = vertices[1].y - vertices[0].y;
        norm01 = sqrt(v01.x * v01.x + v01.y * v01.y);

        v03.x = vertices[2].x - vertices[1].x;
        v03.y = vertices[2].y - vertices[1].y;
        norm03 = sqrt(v03.x * v03.x + v03.y * v03.y);

        // Pour que notre Rectangle soit valide il faut que son plus long c�t� soit �galement le plus "vertical"

        if (norm01 > norm03) // 01 est le plus long
        {
            v01.x /= norm01;
            v01.y /= norm01;
            test = v01.y;

            shape.m_point[0] = vertices[0];
            shape.m_point[1] = vertices[1];
            shape.m_point[2] = vertices[2];
            shape.m_point[3] = vertices[3];
            shape.m_n01 = v01;
            shape.m_n03 = v03;
            shape.m_norm01 = norm01;
            shape.m_norm03 = norm03;

        } else // 03 est le plus long
        {
            v03.x /= norm03;
            v03.y /= norm03;
            test = v03.y;

            shape.m_point[0] = vertices[1];
            shape.m_point[1] = vertices[2];
            shape.m_point[2] = vertices[3];
            shape.m_point[3] = vertices[0];

            shape.m_n01 = v03;
            shape.m_n03 = v01;
            shape.m_norm01 = norm03;
            shape.m_norm03 = norm01;
        }

        // Reste � v�rifier que le plus long est bien le plus "vertical"
        // normalement on devrait ic tester contre sinus 45� (=0.707...), mais on en profite pour eliminer un peu plus largement les candidats ...( 0.5f correspond au sinus d'un angle de 30��  )
        if (test > -0.5f) // Vertical Screen Axis is from top to bottom, ... donc la limite est n�gative...
            continue;

        //Fin de l'initialisation de la Vision Target
        // centre2d (attention il s'agit du centre du rectangle orient�, rien � voir avec le centre projet� du vision target que nous n'avons pas ... ! ), surface et contour associ�
        shape.m_center = rotrect.center;
        shape.m_area = shape.m_norm01 * shape.m_norm03;
        shape.m_contourId = i;

        // On d�fini les points haut et bas. Pour plus de facilit� on rep�re ces point par leur ID dans le vector m_point
        // ... ET on duplique �galement les coordonn�es de ces 2 point dans m_lower et m_higher pour un acc�s plus direct et simple.
        if (shape.m_point[3].y > shape.m_point[0].y) {
            shape.m_lowerId = 3;
            shape.m_lower2D = shape.m_point[3];
        } else {
            shape.m_lowerId = 0;
            shape.m_lower2D = shape.m_point[0];
        }

        if (shape.m_point[2].y < shape.m_point[1].y) {
            shape.m_higherId = 2;
            shape.m_higher2D = shape.m_point[2];
        } else {
            shape.m_higherId = 1;
            shape.m_higher2D = shape.m_point[1];
        }

        // Pour la construction du chain graph
        shape.m_linkSize = 0;
        shape.m_chainQuality = 0.0f;
        shape.m_pChainPrev = NULL;
        shape.m_pLinkPrev = NULL;
        shape.m_isPiledUp = false;

        // Et on enregistre !
        m_visionTargets.push_back(shape);
    }

    std::cout << "+--->	Extract Vision Target Candidates - Found: " << m_visionTargets.size() << std::endl;
    return (unsigned long) m_visionTargets.size();
}


// Pour chaque VisionTarget , on recherche  toutes les autres vision targets avec lesquelles on pourrait potentiellement cr�er un couple.
// l'�valuation entre une VisionTarget A et une vision target B, se fait sur les crit�res suivants:
//							D'abord les crit�res d'�valuations r�dibitoires:
//							------------------------------------------------
//							1/ Les c�t�s verticaux de A et B sont-ils parall�les entre eux ? Si OUI, couple impossible.
//							2/

// Si on estime que ce couple est potentiellement viable on conserve un lien avec ses notes d'�valuation dans le tableau de liens de la vision target.


bool sort_from_left_to_right_x(const VisionTarget &shapeA, const VisionTarget &shapeB) {
    return (shapeA.m_center.x < shapeB.m_center.x);
}

bool sort_from_right_to_left_x(const VisionTarget &shapeA, const VisionTarget &shapeB) {
    return (shapeA.m_center.x > shapeB.m_center.x);
}

const unsigned long Vision::buildVisionTargetsChainGraph() {
    assert(m_visionTargets.size() > 0);

    Point3f u3f, v3f;
    Point2f u2f, v2f;
    float unorm, vnorm;
    float cp;
    VisionTarget *pvta;
    VisionTarget *pvtb;
    unsigned int i, j;


    if (m_visionTargets.size() == 1) {
        m_visionTargets[0].estimate3Dposition(this);
        return 0;
    }

    // Pour commencer on trie le vector de shape selon leur coordonn�e X de gauche � droite
    std::sort(m_visionTargets.begin(), m_visionTargets.end(), sort_from_left_to_right_x);

    // Ensuite,on estime la position 3D de chaque Vision Target d�tect�e
    for (unsigned int i = 0; i < m_visionTargets.size(); i++) {
        m_visionTargets[i].estimate3Dposition(this);
    }


    pvta = &m_visionTargets[0];

    for (i = 0; i < m_visionTargets.size() - 1; i++, pvta++) {
        // Initialisation des variables de travail "chainGraph" de la VisionTarget:
        pvta->m_pChainPrev = NULL;
        pvta->m_pLinkPrev = NULL;
        pvta->m_chainQuality = 0.0f;

        // Recherche/evaluation des liens
        pvtb = pvta + 1;
        for (j = i + 1; j < m_visionTargets.size(); j++, pvtb++) {
            // A) TESTS REDIBITOIRES:
            // --------------------------------------------------------------------------------------------------------------------------------------

            // 1) VisionTargets Parall�les ?
            //		Deux Visiontargets successives sont toujours orient�es sym�triquement par rapport � Y. Jamais elles ne "regardent" dans la meme direction.
            //		Elles sont comme ceci:
            //							/ \  ou  \ /  ! Les visions Target forment un angle ouvert ou ferm� ! >>> C'est bien !
            //		Jamais comme cela:
            //							/ /  ni  \ \  ! Les VisionTargets sont parall�les ! >>> Impossible !
            cp = pvta->m_n01.x * pvtb->m_n01.y - pvtb->m_n01.x * pvta->m_n01.y;
            if (fabs(cp) < EPSILON_VECTOR_CROSSPRODUCT_f32) {
                cout << "+ -> !ECHEC: VisionTarget Candidate n�" << i << " et N�" << j << " sont Vert.parall�les."
                     << endl;
                continue;
            }
            // 2) Angle des deux VisionTarget avec l'horizontale (En vue 2D ). On effectue deux tests, un pour les points 2D haut et un pour les points 2D bas.
            //		Les Points 2D Hauts des deux visions targets sont-ils suffisement align�s avec l'horizontale ?
            u2f = pvta->m_higher2D - pvtb->m_higher2D;
            unorm = sqrtf(u2f.x * u2f.x + u2f.y * u2f.y);
            u2f.x /= unorm;
            //u2f.y /= unorm;//Seul le x normalis� nous int�resse ( = cosinus) donc on evite de calculer le y normalis� pour rien

            //		Les Points Bas des deux visions targets sont-ils suffisement align�s avec l'horizontale ?
            v2f = pvta->m_lower2D - pvtb->m_lower2D;
            vnorm = sqrtf(v2f.x * v2f.x + v2f.y * v2f.y);
            v2f.x /= vnorm;
            //v2f.y /= vnorm;//Seul le x normalis� nous int�resse ( = cosinus) donc on evite de calculer le y normalis� pour rien.
            // Plus le cosinus est proche de 1 mieux c'est. A l'inverse, plus il est petit ( proche de zero ), plus la pente augmente.. .
            if ((fabs(u2f.x) < VISIONTARGETPAIR_TRAPEZOID_INCLINE_LIMIT) ||
                (fabs(v2f.x) < VISIONTARGETPAIR_TRAPEZOID_INCLINE_LIMIT)) {
                cout << "+ -> !ECHEC: VisionTarget Candidate n�" << i << " et N�" << j
                     << " forment un angle trop important avec l'horizontale." << endl;
                continue;
            }

            // 3) Distance entre les deux vision targets dans le plan XZ ( en Vue 3D )
            //		Cette �valuation ne se fait que sur les points bas des trapezoid, car cette distance est constante entre deux VisionTarget successives, quelquesoit l'orientation.
            // ...	Ce qui n'est pas le cas pour les points hauts.
            // ...	La distance entre les points est plus courte quand les VisionTaRGrget sont comme ceci /\ et plus longue quand elles sont comme ceici \/ !

            //		Les Points Bas des deux visions targets sont-ils s�par�S par une distance suffisement en phase avec la r�alit� ?
            u3f = pvta->m_lower3D - pvtb->m_lower3D;
            unorm = u3f.x * u3f.x + u3f.z *
                                    u3f.z;    // On s'interresse uniquement � ce qui se passe en "vue de haut" donc aux coordonn�es X et Z.
            // Tout aussi juste ici que la 'vraie' distance ( mais on gagne une racine carr�e :) )
            if ((unorm < REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMIN) ||
                (unorm > REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMAX)) {
                cout << "+ -> !ECHEC: VisionTarget Candidate n�" << i << " et N�" << j
                     << " sont trop �loign�ex/trop proches." << endl;
                continue;
            }

            // B) EVALUATIONS et VALIDATION/ENREGISTREMENT DU LIEN:
            // --------------------------------------------------------------------------------------------------------------------------------------
            if (pvta->m_linkSize < VISIONTARGET_LINKSIZE) {
                // 1) Qualit� de la distance des deux points bas par rapport � la distance �talon ( distance r�elle ).
                if (unorm < REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM) {
                    pvta->m_links[pvta->m_linkSize].m_sizeQuality = 1.0f -
                                                                    (REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM -
                                                                     unorm) /
                                                                    (REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM -
                                                                     REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMIN);
                } else {
                    pvta->m_links[pvta->m_linkSize].m_sizeQuality = 1.0f - (unorm -
                                                                            REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM) /
                                                                           (REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM_DISTMAX -
                                                                            REAL_VISIONTARGETPAIR_NORM_TRAPEZOID_BOTTOM);
                }

                // 2) Qualit� de l'angle du segment form� par les deux points bas avec l'horizontale ( en 2D ).
                unorm = sqrtf(unorm);
                pvta->m_links[pvta->m_linkSize].m_nXZ.x = u3f.x /
                                                          unorm;        // |_ m_nXZ Vecteur unitaire directeur de la droite passant par les points bas de vtA et vtB en vue de haut !
                pvta->m_links[pvta->m_linkSize].m_nXZ.y =
                        u3f.z / unorm;        // |	( note: on sait que unorm est non nul car test� plus haut ... )
                pvta->m_links[pvta->m_linkSize].m_angleQuality = fabs(
                        v2f.x);    // VALEUR CALCULEE PLUS HAUT LORS DU TEST REDIBITOIRE N�2
                pvta->m_links[pvta->m_linkSize].m_pto = pvtb;

                if (cp < 0.0f)
                    pvta->m_links[pvta->m_linkSize].m_bType = false;    // Ferm� vers le bas
                else
                    pvta->m_links[pvta->m_linkSize].m_bType = true;        // Ouvert vers le bas


                pvta->m_linkSize++;
                cout << "+ -> !!! SUCCES: VisionTarget Candidate n�" << i << " et N�" << j
                     << " forme une paire potentielle de Qualit�. Ang/Dist = "
                     << pvta->m_links[pvta->m_linkSize].m_angleQuality << ","
                     << pvta->m_links[pvta->m_linkSize].m_sizeQuality << endl;
            }
            //else // Le tableau de lien est d�j� plein ! Pour le moment on d�cide juste de ne pa rajouter ce nouveau lien. Par la suite on pourrait envisager d'oublier le lien le moins bien not�.
            //{
            //}
        }
    }

    return 1; // chain build !
}


const unsigned long Vision::extractBestVisualTargetPairChain() {
    assert(m_visionTargetPairs.size() == 0);


    if (m_visionTargets.size() < 2)
        return 0;

    VisionTarget *pvt;
    VisionTarget *pvtin;
    VisionTarget *pvtbestchainend;
    unsigned int i;
    unsigned long j;
    VisionTargetLink *plink;
    float continuity, eval;
    list<VisionTarget *> vtPtrList;


    pvtin = &m_visionTargets[0];
    pvtbestchainend = pvtin;    // initialisation de la fin de la meilleure chaine sur la premi�re vision target ( dont la m_chainQuality = 0 !)
    for (i = 0; i < m_visionTargets.size() - 1; i++, pvtin++) {
        // La vision target est un point d'entr�e si elle n'est pas d�j� dans une chaine et quelle poss�de des liens vers d'autres VisionTargets
        if (!pvtin->m_pChainPrev && pvtin->m_linkSize) {
            // on l'ins�re sur la pile FIFO explicite
            pvtin->m_isPiledUp = true;
            vtPtrList.push_back(pvtin);
        }

        // Parcours r�cursif ( explicite avec pile FILO ) du chain graph et �valuation/comparaison des liens.
        while (vtPtrList.size()) {
            pvt = vtPtrList.front();
            vtPtrList.pop_front();

            assert(pvt->m_isPiledUp);
            pvt->m_isPiledUp = false;

            plink = pvt->m_links;
            for (j = 0; j < pvt->m_linkSize; j++, plink++) {
                if (pvt->m_pChainPrev) {
                    assert(pvt->m_pLinkPrev != NULL);

                    // Continuit� entre les Paires...
                    // 	a) Alternance Paire ouverte / Paire ferm�e.

                    //if (pvt->m_pLinkPrev->m_bType == plink->m_bType)
                    //{
                    // Oups: nous avons ici une succession de deux liens ouverts ou de deux liens ferm�s.
                    // La chaine ainsi form�e n'est pas valide. On donne une note(evaluation de qualit�) tr�s mauvaise pour rester dans le cadre de l'algo tout en "bloquant" la progression � ce niveau.
                    //eval = -1000.0f;
                    //}
                    //else
                    //{
                    //	b)	Bas�es sur le ratio des qualit�s d'angle et de distance des deux paires.
                    continuity = MIN(pvt->m_pLinkPrev->m_sizeQuality, plink->m_sizeQuality) /
                                 MAX(pvt->m_pLinkPrev->m_sizeQuality, plink->m_sizeQuality);
                    continuity *= MIN(pvt->m_pLinkPrev->m_angleQuality, plink->m_angleQuality) /
                                  MAX(pvt->m_pLinkPrev->m_angleQuality, plink->m_angleQuality);

                    //	c)	Note de continuit� �tendue: evalue l'alignement de la paire avec la pr�c�dente en vue de top !
                    //		plus les deux vecteurs m_nXZ sont align�s et dans le m�me sens et plus il "dot" se rapproche 1...
                    continuity *= plink->m_nXZ.dot(pvt->m_pLinkPrev->m_nXZ);

                    eval = continuity * (plink->m_sizeQuality + plink->m_angleQuality) + pvt->m_chainQuality;
                    //}
                } else {
                    // c'est comme si on avait:  continuity = 1.0f;
                    eval = (plink->m_sizeQuality + plink->m_angleQuality) + pvt->m_chainQuality;
                }


                if (eval > plink->m_pto->m_chainQuality) {
                    // La note de qualit� glonbale obtenue est meilleure que celle actuellement enregistr�e dans la VisionTarget point�e.
                    // Donc le chemin actuellement pris est meilleur, du moins jusqu'ici.
                    plink->m_pto->m_chainQuality = eval;
                    plink->m_pto->m_pChainPrev = pvt;
                    plink->m_pto->m_pLinkPrev = plink;

                    if (!plink->m_pto->m_isPiledUp) {
                        if (plink->m_pto->m_linkSize) {
                            plink->m_pto->m_isPiledUp = true;
                            vtPtrList.push_back(plink->m_pto);
                        } else {
                            // "plink->m_pto" est en bout de chaine, cette vision target n'a aucun lien vers d'autres vision target avec qui elle pourrait form� une paire.
                            // Nous sommes donc en bout de chaine. Regardons si la chaine obtenue est celle de meilleure qualit�.
                            if (plink->m_pto->m_chainQuality > pvtbestchainend->m_chainQuality) {
                                pvtbestchainend = plink->m_pto;
                            }
                        }
                    }
                }
            }
        }
    }

    // Construction des meilleures paire en remontant la meilleure chaine par la fin !
    VisionTargetPair pair;
    pvt = pvtbestchainend;
    while (pvt->m_pChainPrev) {
        if (pair.assemble(pvt, pvt->m_pChainPrev, pvt->m_pLinkPrev->m_bType))
            m_visionTargetPairs.push_back(pair);
        // on remonte
        pvt = pvt->m_pChainPrev;
    }
    std::cout << "|" << std::endl;
    std::cout << "+--->	Extract Vision Target Pairs ( best chain ) - Found: " << m_visionTargetPairs.size()
              << std::endl;

    return (unsigned long) m_visionTargetPairs.size();
}

VisionTargetPair *Vision::getCenteredTargetPair() {
    float limit = 1000000.0f;
    float d;

    m_pSelectedVisionTargetPair = nullptr;

    for (int i = 0; i < m_visionTargetPairs.size(); i++) {
        if (m_visionTargetPairs[i].m_bType == false) // Angle ouvert vers le bas
        {
            d = abs(m_visionTargetPairs[i].m_center[VisionTargetPair::TRAPEZOID].x - getCurrentScreenCenterX());
            if (d < limit) {
                limit = d;
                m_pSelectedVisionTargetPair = &m_visionTargetPairs[i];
            }
        }
    }

    return m_pSelectedVisionTargetPair;
}

const unsigned long Vision::findVisionTargetPairs() {
    // Reset all the vectors !
    m_contours.resize(0);
    m_visionTargets.resize(0);
    m_visionTargetPairs.resize(0);

    if (extractVisionTargetCandidates())
        if (buildVisionTargetsChainGraph())
            return extractBestVisualTargetPairChain();

    return 0;
}


void Vision::buildRegressionLines() {
    /*
    int i;
    double xsomme, ysomme, xysomme, xxsomme;
    double ai, bi;

    xsomme = 0.0; ysomme = 0.0;
    xysomme = 0.0; xxsomme = 0.0;
    for (i = 0; i < n; i++)
    {
        xsomme = xsomme + x[i];
        ysomme = ysomme + y[i];
        xysomme = xysomme + x[i] * y[i];
        xxsomme = xxsomme + x[i] * x[i];
    }
    ai = (n*xysomme - xsomme * ysomme) / (n*xxsomme - xsomme * xsomme);
    bi = (ysomme - ai * xsomme) / n;
    *a = ai;
    *b = bi;
    return;
    */
}











/// METHODES ANCIENNES / ABANDONNEES *****************************************************************************************************************
///
/*
void Vision::setFov(const double camerafov_deg)
{
	m_fovY = camerafov_deg;
	// pre-computed data
	m_tangent = tan(NDEGtoRAD(m_fovY * 0.5));
	m_inverseTangent = 1 / m_tangent;

	m_fovDist = m_screenWidth / (2 * m_tangent);
}
float Vision::estimateIntrinsicCameraFovXRad()
{
	return atan(m_screenWidth / (2 * (float)m_intrinsic.at<double>(0, 0)));
}

float Vision::estimateIntrinsicCameraFovYRad()
{
	return atan(m_screenHeight / (2 * (float)m_intrinsic.at<double>(1, 1)));
}

bool Vision::get3DPointOnHorizontalPlane(cv::Point3f& res, const cv::Point2f& screenpoint,const size_t plane_index)
{
	double		xl, yl,zl;
	double		x, y;

	double		xc, yc, zc;

	// A/ point de l'�cran --> point 3D en coordonn�es camera: (xc,yc,zc)
	// -----------------------------------------------------------------
	zl = 0.6;//cm
	yl = zl * m_tangent;
	xl = yl * m_aspectRatio;

	x = ((double)screenpoint.x - m_widthoutof2 ) / m_widthoutof2;
	y = (m_heightoutof2 - (double)screenpoint.y ) / m_heightoutof2;

	// 3D point ( on screen) in Camera Coordinates system: xc,yc,zc
	xc = x * xl;
	yc = y * yl;
	zc = zl;

	// B/ INTERSECTION de la demi-droite ( camera, point 3D ) et du plan
	// ------------------------------------------------------------------
	// Donc on a maintenant un rayon qui part de la camera et qui passe par le point de l'�cran qu'on a  situ� en 3D devant la camera...
	// Il ne reste plus qu'a determiner l'intersection de ce rayon avec le plan horizontal situ� � la hauteur "planeY" pass�e en param�tre.
	// !!! Attention cette hauteur est d�finie par rapport au niveau du sol (=0) ou est pos� le robot.

	// 0/ normalisation du vecteur directeur du rayon
	double norm = sqrt(xc * xc + yc * yc + zc * zc);
	xc /= norm;
	yc /= norm;
	zc /= norm;

	// 1/ on calcul le produit scalaire des vecteurs directeurs du rayon et du plan.
	//	celui du rayon est (xc,yc,zc) celui du plan est (0,1,0) car il s'agit d'un plan hoprizontal...
	//		on a donc, dot1	= plandir_x * xc + plandir_y * yc + plandir_z * zc;
	//						= 0 * xc + 1 * yc + 0 * zc
	//						= yc
	double dot1 = yc;

	if (NABS(dot1) <= NF64_EPSILON_VECTOR_DOTPRODUCT) // a la place de = 0, siginie que la droite et le plan sont parall�les ou presque !
	{
		return false;
	}
	else
	{
		// L'id�e est la suivante: ...
		// on fait: u = p - camerapos.
		//				Avec p(0,m_planeY[],0)  point du plan et camerapos(0,m_cameraYWorld,0)) position de la camera
		// on a donc,	ux	= 0 - 0
		//				uy	= m_planeY[plane_index] - m_cameraYworld
		//				uz	= 0 - 0
		//
		// Ensuite, on calcul le produit scalaire du vecteur directeur du plan et de u.
		// soit,	dot2	= 0 * ux  + 1 * uy + 0 * uz
		//					= uy
		//					= (m_planeY[plane_index] - m_cameraYworld)
		//
		// puis on calcul le ratio r des deux produits scalaires.
		//				r	= dot2/dot1
		double r = (m_planeY[plane_index] - m_cameraYworld) / dot1;
		//
		// finalement pour obtenir le point d'intersection on multiplie le vecteur directeur du rayon par r et on ajoute la position de la camera.

		res.x = (float)(xc * r );
		res.y = (float)(yc * r + m_cameraYworld);
		res.z = (float)(zc * r );

		return true;
	}
}
*/
