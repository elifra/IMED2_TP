/*!
  \file usImageGradient.cpp
  \brief Process images
*/

#include "usImageGradient.h"

#define TO_COMPLETE

/*!
  \brief Compute the gradient images along x, y and z directions from a thin volume composed of 3 2D slices
  \param imga, img0, imgb: parallel images captured
  \param ip0,ipf: upper left and lower right corners of the ROI (in pixels) inside the gradient is computed
  \param dIdx,dIdy,dIdz: gradient images along x,y and z directions to compute
*/
void usImageGradient::Grad3DF3x3x3 (vpImage<unsigned char> &imga, vpImage<unsigned char> &img0, vpImage<unsigned char> &imgb, vpImage<double> &dIdx, vpImage<double> &dIdy, vpImage<double> &dIdz, vpImagePoint ip0, vpImagePoint ipf )
{

    int Wmin = (int)ip0.get_u(); // pixel u coordinate of the ROI left-top corner
    int Wmax = (int)ipf.get_u(); // pixel u coordinate of the ROI right-bottom corner
    int Hmin = (int)ip0.get_v(); // pixel v coordinate of the ROI left-top corner
    int Hmax = (int)ipf.get_v(); // pixel v coordinate of the ROI right-bottom corner

#ifdef TO_COMPLETE
    // Question 3
    // Compute the gradient images along x, y and z directions from a thin volume composed of three 2D slices
    // corresponding to images imga, img0, imgb
    // Note that dIdx, dIdy and dIdz have the same size that the whole 2D ultrasound image but the gradient could
    // be computed only for the pixels contained in the ROI

    // dIdx[...][...] = ...;
    // dIdy[...][...] = ...;
    // dIdz[...][...] = ...;

#endif

}


