#include "VisionTargetPair.h"
#include "Tools.h"
#include "values.h"

const bool VisionTargetPair::assemble(VisionTarget *pvta, VisionTarget *pvtb, const bool type) {

    // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Angle apparent entre les deux shapes:
    // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // 1: L'angle est-il "bien" ou "mal" orient� ?
    //		X product VT1.n1 x VT2.n1 < 0 signifie que l'angle entre les deux visions target est 'bien' orient� ( l'angle entre les deux vecteurs est ouvert en bas )
    //		X product VT1.n1 x VT2.n1 > 0 signifie que l'angle entre les deux visions target est 'mal' orient� ( l'angle entre les deux vecteurs est ouvert en haut/ferm� en bas )
    //
    //		 .....		.....						.....				 .....
    //		/ 	/		 \   \						 \   \				/ 	/
    //	   /   /		  \   \						  \   \			   /   /
    //    /   /			   \   \					   \   \		  /   /
    //	 /   /			    \   \					    \   \		 /   /
    //  .....			     .....					     .....		.....
    //
    //		Ouvert vers le Bas							 Ferm� vers le Bas
    //
    //
    // Ferm� vers le bas
    /*
    if (pvta->m_n01.cross(pvtb->m_n01) < 0.0f)
    {
        return false;
    }
    else
    {
    */
    // setup
    m_pvisionTargetA = pvta;
    m_pvisionTargetB = pvtb;

    // Setup Finalisation
    // Calcul du centre du trapeze[A3][A0][B1][B2]
    Tools::segXseg(pvta->m_higher2D, pvtb->m_lower2D, pvta->m_lower2D, pvtb->m_higher2D,
                   m_center[VisionTargetPair::TRAPEZOID]);

    // Calcul du projet� (selon l'axe Y ) du centre du trap�ze sur le segement [B1][B2], pour obtenir le milieu du segment [B1][B2] en respectant la d�formation perspective
    // ( On suppose donc ici qu'une droite verticale dans le monde reste une droite verticale une fois projet�e ... ( ce qui n'est pas tout a fait vrai ... mais bon )
    cv::Point2f basev = m_center[0];
    basev.y += CAPTURED_IMAGE_HEIGHT;
    Tools::segXseg(pvta->m_lower2D, pvtb->m_lower2D, m_center[0], basev,
                   m_center[VisionTargetPair::TRAPEZOID_BOTTOM_SIDE]);

    cv::Point3f v;

    v = m_pvisionTargetB->m_lower3D - m_pvisionTargetA->m_lower3D;
    //m_dirVect.x = m_pvisionTargetB->m_lower3D.x - m_pvisionTargetA->m_lower3D.x;
    float n = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    v.x /= n;
    v.y /= n;
    v.z /= n;

    m_dirVect.x = v.z;
    m_dirVect.y = v.y;
    m_dirVect.z = -v.x;

    m_bType = type;
    return true;
    //}
}

