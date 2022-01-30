/*!
  \file cptImgLs.cpp
  \brief Compute interaction matrix for visual servoing based on intensity features
*/


#include "usIntensityInteraction.h"

#define TO_COMPLETE


/*!
  \brief compute the interaction matrix used in the control law
  \param dIdx,dIdy,dIdz: gradient images along x,y and z directions
  \param sx,sy: scaling parameters (in meter)
  \param ip0,ipf: upper left and lower right corners of the ROI (in pixels)
  \return Ls: interaction matrix used in the control law of size nbP*6 (with nbP the number of features)
*/

vpMatrix cptImgLs::Ls( vpImage<double> &dIdx, vpImage<double> &dIdy, vpImage<double> &dIdz, double sx, double sy, vpImagePoint ip0, vpImagePoint ipf )
{
    int imgW = dIdx.getWidth() ; // ultrasound image width in number of pixels
    int imgH = dIdx.getHeight() ; // ultrasound image height in number of pixels
    int Wmin = (int)ip0.get_u(); // pixel u coordinate of the ROI left-top corner
    int Wmax = (int)ipf.get_u(); // pixel u coordinate of the ROI right-bottom corner
    int Hmin = (int)ip0.get_v(); // pixel v coordinate of the ROI left-top corner
    int Hmax = (int)ipf.get_v(); // pixel v coordinate of the ROI right-bottom corner
    int nbPx = (Hmax-Hmin+1)*(Wmax-Wmin+1) ; // number of visual features
    int ROIW = (Wmax-Wmin+1); // Width of the ROI in number of pixels
    int ROIH = (Hmax-Hmin+1); // Height of the ROI in number of pixels

    vpMatrix Ls(nbPx, 6);

#ifdef TO_COMPLETE
    // Question 4
    // compute the interaction matrix related to the visual features
    //
    // Ls[...][...] = ...;


#endif

    return Ls ;
}



