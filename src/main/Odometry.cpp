#include "Odometry.h"
#include "Tools.h"

Encoder::Encoder(const double countperturn) {
    m_countPerTurn = countperturn;
    m_currentCount = 0;
    m_prevCount = 0;
    m_delta = 0;
}


void Encoder::update() {

    // On enregistre les donn�es pr�c�dentes avant de les '�craser' par les nouvelles
    m_prevCount = m_currentCount;
    // et on enregistre les nouvelles donn�es de l'encodeur
    m_delta = 2.0 * PI_f64 * (m_currentCount - m_prevCount) / m_countPerTurn;    // Rad

    // Il ne semble pas utile de calculer les donn�es cin�matiques de chaque moteur.
    // ( au cas ou, il conviendrait de faire comme ceci, en ayant recup�rer les informations de temps pour calculer dt )
    /*
    m_previousTime		= m_currentTime;
    m_rotationPrevKin	= m_rotationKin;
    m_currentTime = timestamp;
    double odt = 1.0/(timestamp - m_previousTime);
    m_rotationKin.delta	= 2.0*PI_f64*(m_sample.count - m_prevSample.count) / m_countPerTurn;	// Rad			Angle
    m_rotationKin.speed	= m_rotationKin.delta * odt;											// Rad/s		vitesse angulaire
    m_rotationKin.acc	= (m_rotationPrevKin.speed - m_rotationKin.speed) * odt;				// Rad/s/s		acceleration angulaire
    m_rotationKin.jerk	= (m_rotationPrevKin.acc - m_rotationKin.acc) * odt;					// Rad/s/s/s	'� coup' angulaire
    */

}

void Encoder::reset() {
    //m_countPerTurn ... ne change pas !;
    m_currentCount = 0;
    m_prevCount = 0;
    m_delta = 0;
}

MobileBase::MobileBase(Encoder *pleftEncoder, Encoder *prightEncoder, const double leftwheelradius,
                       const double righttwheelradius, const double axleTrack) {
    assert(pleftEncoder != nullptr);
    assert(prightEncoder != nullptr);
    assert(axleTrack > 0.0);
    assert(leftwheelradius > 0.0);
    assert(righttwheelradius > 0.0);

    m_AxleTrack = axleTrack;            // Entraxe (voie) roue gauche/roue droite
    m_pLeftEncoder = pleftEncoder;            // Encodeur Roue Gauche
    m_pRightEncoder = prightEncoder;        // Encodeur Roue Droite
    m_leftWheelRadius = leftwheelradius;    // Rayon de la roue Gauche
    m_rightWheelRadius = righttwheelradius; // Rayon de la roue Droite

    m_previousTime = 0;
    m_currentTime = 0;

    m_rotationPrevKin = {0};
    m_rotationKin = {0};

    m_translationPrevKin = {0};
    m_translationKin = {0};

    m_vectorPrevKin = {0};
    m_vectorKin = {0};

    m_currentOrientation = 0;
    m_prevOrientation = 0;

    m_currentPosition = {0};
    m_prevPosition = {0};
}

void MobileBase::update(const double timestamp) {
    assert(timestamp != m_previousTime); // sinon dt = 0 et ... division par zero

    // Mise � jour des encodeurs ...
    m_pLeftEncoder->update();
    m_pRightEncoder->update();

    // Mise � jour des donn�es cin�matiques de la base roulante.
    // Enregistrement des donn�es pr�c�dentes
    m_previousTime = m_currentTime;
    m_vectorPrevKin = m_vectorKin;
    m_translationPrevKin = m_translationKin;
    m_rotationPrevKin = m_rotationKin;
    m_prevOrientation = m_currentOrientation;
    m_prevPosition = m_currentPosition;

    // Calcul de la translation constat�e depuis la pr�c�dente mesure. ( aux travers des 2 encodeurs roue gauche et roue droite )
    // ... et d�duction de la vitesse et de la position de la base roulante.
    double ldist, rdist;
    ldist = m_leftWheelRadius * m_pLeftEncoder->m_delta;        // Distance parcourue par la roue Gauche
    rdist = m_rightWheelRadius * m_pRightEncoder->m_delta;    // Distance parcourue par la roue Droite

    // dt
    m_currentTime = timestamp;
    double odt = 1.0 / (timestamp - m_previousTime);

    // Delta
    m_translationKin.delta =
            (rdist + ldist) * 0.5;                //	Distance parcourue par la base depuis la derniere mesure.
    m_rotationKin.delta = (rdist - ldist) / m_AxleTrack;        //  Rotation de la base depuis la derniere mesure.
    m_vectorKin.vDelta.x = m_translationKin.delta * cos(m_prevOrientation);
    m_vectorKin.vDelta.y = m_translationKin.delta * sin(m_prevOrientation);

    //Speed
    m_translationKin.speed = m_translationKin.delta * odt;
    m_rotationKin.speed = m_rotationKin.delta * odt;
    m_vectorKin.vSpeed.x = m_vectorKin.vDelta.x * odt;
    m_vectorKin.vSpeed.y = m_vectorKin.vDelta.y * odt;
    //Acceleration
    m_translationKin.acc = (m_translationKin.speed - m_translationPrevKin.speed) * odt;
    m_rotationKin.acc = (m_rotationKin.speed - m_rotationPrevKin.speed) * odt;
    m_vectorKin.vAcc.x = (m_vectorKin.vSpeed.x - m_vectorPrevKin.vSpeed.x) * odt;
    m_vectorKin.vAcc.y = (m_vectorKin.vSpeed.y - m_vectorPrevKin.vSpeed.y) * odt;
    //Jerk
    m_translationKin.jerk = (m_translationKin.acc - m_translationPrevKin.acc) * odt;
    m_rotationKin.jerk = (m_rotationKin.acc - m_rotationPrevKin.acc) * odt;
    m_vectorKin.vJerk.x = (m_vectorKin.vAcc.x - m_vectorPrevKin.vAcc.x) * odt;
    m_vectorKin.vJerk.y = (m_vectorKin.vAcc.y - m_vectorPrevKin.vAcc.y) * odt;

    // Orientation
    m_currentOrientation += m_rotationKin.delta;

    // Position
    m_currentPosition.x += m_vectorKin.vDelta.x;
    m_currentPosition.y += m_vectorKin.vDelta.y;

    /*
    // ...ou Integration progressive pour une meilleure pr�cision ? A voir ....
    unsigned int itercount = (unsigned int)(time_delay / ODOMETRY_INTEGRATION_STEP_f64) + 1;
    double sim_delay = time_delay / (double)itercount;

    for (int i = 0; i < itercount; i++)
    {

    }
    */
}

void MobileBase::resetBasis(const Basis &basis) {
    m_currentPosition = m_prevPosition = basis.origin;
    m_currentOrientation = m_prevOrientation = basis.orientation;
}
