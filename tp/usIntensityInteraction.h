/*!
  \file cptImgLs.h
  \brief Compute Interaction Matrix for visual servo control
*/

#ifndef usIntensityInteraction_h
#define usIntensityInteraction_h


#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>


/*!
  \class cptImgLs
  \brief  This class gives method to compute the interaction matrix of the control law for visual servoing based on intensity features
*/

class cptImgLs
{
public:

    //!compute the interaction matrix for visual servoing based on intensity features
    static vpMatrix Ls( vpImage<double> &dIdx, vpImage<double> &dIdy, vpImage<double> &dIdz, double sx, double sy, vpImagePoint ip0, vpImagePoint ipf );

};

#endif
