/*!
  \file usImageGradient.h
  \brief process images
*/

#ifndef usImageGradient_h
#define usImageGradient_h

#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>

#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpMath.h>

/*!
  \class usImageGradient
  \brief This class gives method to compute the image 3D gradient
*/

class usImageGradient
{

public:

    //! Apply a 3x3x3 filter to compute the derivatives of the intensity
    static
    void Grad3DF3x3x3 (vpImage<unsigned char> &img0, vpImage<unsigned char> &imga, vpImage<unsigned char> &imgb, vpImage<double> &dIdx, vpImage<double> &dIdy, vpImage<double> &dIdz, vpImagePoint ip0, vpImagePoint ipf );

};

#endif

