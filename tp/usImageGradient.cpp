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

    for(int i = Hmin; i < Hmax; i++) {
      for(int j = Wmin; j < Wmax; j++) {
        int res0X = -2*img0[i-1][j-1] + 2*img0[i-1][j+1] - 4*img0[i][j-1] + 4*img0[i][j+1] - 2*img0[i+1][j-1] + 2*img0[i+1][j+1];
        int resBX = -1*imgb[i-1][j-1] + 1*imgb[i-1][j+1] - 2*imgb[i][j-1] + 2*imgb[i][j+1] - 1*imgb[i+1][j-1] + 1*imgb[i+1][j+1];
        int resAX = -1*imga[i-1][j-1] + 1*imga[i-1][j+1] - 2*imga[i][j-1] + 2*imga[i][j+1] - 1*imga[i+1][j-1] + 1*imga[i+1][j+1];
        dIdx[i][j] = res0X+resBX+resAX;

        int res0Y = -2*img0[i-1][j-1] + 2*img0[i+1][j-1] - 4*img0[i-1][j] + 4*img0[i+1][j] - 2*img0[i-1][j+1] + 2*img0[i+1][j+1];
        int resBY = -1*imgb[i-1][j-1] + 1*imgb[i+1][j-1] - 2*imgb[i-1][j] + 2*imgb[i+1][j] - 1*imgb[i-1][j+1] + 1*imgb[i+1][j+1];
        int resAY = -1*imga[i-1][j-1] + 1*imga[i+1][j-1] - 2*imga[i-1][j] + 2*imga[i+1][j] - 1*imga[i-1][j+1] + 1*imga[i+1][j+1];
        dIdy[i][j] = res0Y+resBY+resAY;

        int res0Z = 0;
        int resBZ = 1*imgb[i-1][j-1] + 2*imgb[i-1][j] + 1*imgb[i-1][j+1] + 2*imgb[i][j-1] + 4*imgb[i][j] + 2*imgb[i][j+1] + 1*imgb[i+1][j-1] + 2*imgb[i+1][j] + 1*imgb[i+1][j+1];
        int resAZ = -(1*imga[i-1][j-1] + 2*imga[i-1][j] + 1*imga[i-1][j+1] + 2*imga[i][j-1] + 4*imga[i][j] + 2*imga[i][j+1] + 1*imga[i+1][j-1] + 2*imga[i+1][j] + 1*imga[i+1][j+1]);
        dIdz[i][j] = res0Z+resBZ+resAZ;
      }
    }

#endif

}


