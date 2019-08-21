#pragma once

#include "opencv2/opencv.hpp"

#define ODOMETRY_ENCODER_COUNT            3        // nombre d'encodeurs � suivre
#define ODOMETRY_ENCODER_LEFT_WHEELS    0
#define ODOMETRY_ENCODER_RIGHT_WHEELS    1
#define ODOMETRY_ENCODER_ARM            2

#define ODOMETRY_INTEGRATION_STEP_f64    0.01    // en seconde. Pas d'int�gration de la vitesse ET de l'acc�l�ration dans la position utilis� par la m�thode it�rative d'estimation de position. ( pour une meilleure pr�cision )
// 0.01 signifie une simulation avec un pas fixe de 1/100�me de seconde
// Un vecteur 2D (x,y) en double precision.
typedef struct {
    double x;
    double y;
} V2d;

// description d'une base 2D sous la forme origin et orientation
typedef struct {
    V2d origin;
    double orientation;
} Basis;

class Encoder {

public:
    Encoder(const double countperturn);

    void update();

    void reset();

    double m_countPerTurn;    // nombre de tick par tour ( valeur fixe potentiellement specifique par encodeur)
    double m_prevCount;    // en "tick" peut-�tre + ou - ( + sens trigo ? )
    double m_currentCount;    // en "tick" peut-�tre + ou - ( + sens trigo ? )
    double m_delta;        // diff�rence entre currentCount et prevCount. En RADIAN
    //--------------------------------------
    // Il ne semble pas utile de calculer les donn�es cin�matiques de chaque moteur/roue.
    // ( au cas ou, juste enlever les commentaires ici et dans la fonction 'Encoder::update' et r�cup�rer les time stamps)
    // double			m_previousTime;
    // double			m_currentTime;
    // Kinematics		m_rotationKin;
    // Kinematics		m_rotationPrevKin;
};

// Donn�es cinematiques 1D
typedef struct {
    double delta;            // diff�rence de position/angle ( dX )
    double speed;            // vitesse						( dX/dt )
    double acc;            // acceleration					( dX/(dt*dt))
    double jerk;            // Jerk							( dX/(dt*dt*dt) )
} Kinematics1D;

// Donn�es cin�matiques 2D
typedef struct {
    V2d vDelta;        // diff�rence de position/angle ( dX )
    V2d vSpeed;        // vitesse						( dX/dt )
    V2d vAcc;        // acceleration					( dX/(dt*dt))
    V2d vJerk;        // Jerk							( dX/(dt*dt*dt) )
} Kinematics2D;

class Odometry {
public:
    // No ring buffer until now, but very soon ...
    Encoder m_encoders[ODOMETRY_ENCODER_COUNT];
};

class MobileBase {
public:
    MobileBase(Encoder *pleftEncoder, Encoder *prightEncoder, const double leftwheelradius,
               const double righttwheelradius, const double axleTrack);

    void update(const double timestamp);

    void resetBasis(const Basis &basis = {0});

    Encoder *m_pLeftEncoder;
    Encoder *m_pRightEncoder;

    double m_leftWheelRadius;        // Rayon de la roue de gauche
    double m_rightWheelRadius;        // Rayon de la roue de droite
    double m_AxleTrack;            // Voie ( entraxe  ou distance entre les deux roues )

    // Donn�es cin�matiques de la base ( Rotation / Translation / Vecteurs / position )
    // ( notes: on appelle rotation une variation d'orientation. L'orientation repr�sente 'ou regarde la base mobile')
    double m_previousTime;
    double m_currentTime;

    Kinematics1D m_rotationPrevKin;
    Kinematics1D m_rotationKin;

    Kinematics1D m_translationPrevKin;
    Kinematics1D m_translationKin;

    Kinematics2D m_vectorPrevKin;
    Kinematics2D m_vectorKin;

    double m_currentOrientation;
    double m_prevOrientation;    // inutile ?

    V2d m_currentPosition;
    V2d m_prevPosition;        // inutile ?
};

