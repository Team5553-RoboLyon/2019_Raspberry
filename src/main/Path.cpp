#include "Path.h"

/*
Path::~Path()
{
}
*/

bool Path::buildPath(cv::Point2f start, cv::Point2f startdir, cv::Point2f end, cv::Point2f enddir) {
    cv::Point2f i;
    cv::Point2f j;
    cv::Point2f p;
    cv::Point2f p2;
    cv::Point2f c;
    cv::Point2f v;
    cv::Point2f t;
    float jnorm;
    float r;


    // ij: base orthonormée
    j = enddir;
    jnorm = sqrtf(j.x * j.x + j.y * j.y);
    j.x /= jnorm;
    j.y /= jnorm;

    i.x = -j.y;
    i.y = j.x;

    // 1) En fonction du point de dépard et du point d'arrivée souhaités, on va définir combien de 'knot' devra contenir notre spline (chemin).

    // 1) Définition du Rayon du cercle d'approche final
    //
    //		distance_depart_arrivee = 2*ktang*r + r
    //
    //		Avec "ktang" coefficient donnant la longueur de la tangente par rapport au rayon pour effectivement dessiner un cercle (mesure empirique sur Photoshop)
    //		ktang > 0.5f donc 2*ktang > 1
    //
    //		Donc, "2*ktang*r + r > 2*R"
    //
    v.x = start.x - end.x;
    v.x = start.y - end.y;
    p.x = i.dot(v);
    p.y = j.dot(v);

    // le robot est trop près du mur
    if (fabs(p.y) < ROBOT_BOUNDING_CIRCLE_RADIUS)
        return false;

    // le robot est trop près de la cible.
    r = p.x / (1.0f + 2.0f * CIRCLE_TANGENT_COEF);
    if (fabs(r) < m_endRadiusMin)
        return false;

    // Si le rayon trouvé est "possible" on place le centre du cercle
    // d = sqrtf(v.x*v.x + v.y*v.y);
    c.x = r * p.x;
    c.y = r * p.y;
    // ... et le point intermédiaire
    p2.x = c.x + r * j.x;
    p2.y = c.y + r * j.y;

    // Enregistrement des Knot un à un
    if (m_spline.m_splineKnots.size() != 3)
        m_spline.m_splineKnots.resize(3);
    // start
    m_spline.m_splineKnots[0].setKnotPosition(start);
    // TODO ... tangente !!!
    m_spline.m_splineKnots[0].setKnotTangenteB(t);

    // intermédiaire ( point de contact/ralliement au cercle final)
    m_spline.m_splineKnots[1].setKnotPosition(p2);
    t.x = p2.x - r * CIRCLE_TANGENT_COEF * i.x;
    t.y = p2.y - r * CIRCLE_TANGENT_COEF * i.y;
    m_spline.m_splineKnots[1].setKnotTangenteA(t);
    t.x = p2.x + r * CIRCLE_TANGENT_COEF * i.x;
    t.y = p2.y + r * CIRCLE_TANGENT_COEF * i.y;
    m_spline.m_splineKnots[1].setKnotTangenteB(t);

    // end
    m_spline.m_splineKnots[2].setKnotPosition(end);
    t.x = p2.x - r * CIRCLE_TANGENT_COEF * j.x;
    t.y = p2.y - r * CIRCLE_TANGENT_COEF * j.y;
    m_spline.m_splineKnots[2].setKnotTangenteA(t);
    t.x = p2.x + r * CIRCLE_TANGENT_COEF * j.x;
    t.y = p2.y + r * CIRCLE_TANGENT_COEF * j.y;
    m_spline.m_splineKnots[2].setKnotTangenteB(t);


    return false;
}
