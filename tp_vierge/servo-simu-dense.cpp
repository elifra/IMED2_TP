/*!

  \example servo-simu-dense.cpp

  Simulation of ultrasound dense visual servoing

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpPoseVector.h>
#include <visp/vpTime.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpImageTools.h>
#include "vtkRenderWindowInteractor.h"
#include <UsSimulator/usSimulator.h>

#include "usImageGradient.h"
#include "usIntensityInteraction.h"

#include "vpCurvePlotter.h"

#define TO_COMPLETE


using namespace std;

int main()
{

    ///////////////////////////////////////////////////////////////////////
    //
    // Init and first render the ultrasound simulator
    //
    ///////////////////////////////////////////////////////////////////////

    char *filename = new char [FILENAME_MAX];

    // Set 3D ulstrasound volume path
    sprintf(filename, "../../3DUSdata/liver.txt");

    // Create the VTK window
    vtkRenderWindowInteractor *mVTKWindow;
    mVTKWindow = vtkRenderWindowInteractor::New();

    // Create the simulator
    usSimulator *simulator=NULL;
    if(simulator ==NULL) simulator = new usSimulator(filename, mVTKWindow, 600, 400);

    // Create an 2D image display
    vpDisplayX display(simulator->Image, 664, 0,"Current image");

    // Display virtual ultrasound probe image
    vpDisplay::display(simulator->Image) ;
    vpDisplay::flush(simulator->Image) ;


    // Simulation paremeters
    double time = 0; // time initialization
    double period = 0.04; // Control sample time (second)

    vpPoseVector world_poseProbe; // pose ot the ultrasound probe frame with respect to the world frame (6 dimension vector)
    simulator->getProbePosition(world_poseProbe);

    vpHomogeneousMatrix worldMprobe; // Homogeneous 4x4 matrix describing the probe pose
    worldMprobe.buildFrom(world_poseProbe);

    // Initialize probe and object (volume) velociy vectors
    vpColVector probe_velocity(6), object_velocity(6);


    ///////////////////////////////////////////////////////////////////////
    //
    // Initialization of different images
    //
    ///////////////////////////////////////////////////////////////////////

    // Ultrasound image sizes
    int height = simulator->Image.getHeight();
    int width = simulator->Image.getWidth();

    // size of voxel along X direction
    double sx = simulator->spacingZ;

    // size of voxel along Y direction
    double sy = simulator->spacingY;

    // size of voxel along Z direction
    double sz = simulator->spacingZ;


    // Five grey-level images that will be used to define the thin volume of 5 slices needed for 3D gradient estimation
    vpImage<unsigned char> imga(height, width);
    vpImage<unsigned char> img0(height, width); // central slice
    vpImage<unsigned char> imgb(height, width);



    ///////////////////////////////////////////////////////////////////////
    //
    // Capture the 5 images of the thin volume of 5 slices needed for the 3D gradient computation
    //
    ///////////////////////////////////////////////////////////////////////

    vpPoseVector init_world_poseProbe; // Initial probe pose coresponding to the third (central) slice of the thin volume of 5 slices
    init_world_poseProbe = world_poseProbe;

    world_poseProbe[2] = init_world_poseProbe[2] - sz; // Modify probe pose to observe second slice of the thin volume
    simulator->setProbePosition(world_poseProbe); // Set the simulator with this new probe pose
    simulator->render(); // Render the simulator to take into account modification
    vpDisplay::display(simulator->Image); // Display ultrasound probe image
    vpDisplay::flush(simulator->Image);
    imga = simulator->Image; // Copy current ultrasound image to second image of the thin volume

    world_poseProbe[2] = init_world_poseProbe[2] + sz; // Modify probe pose to observe fourth slice of the thin volume
    simulator->setProbePosition(world_poseProbe); // Set the simulator with this new probe pose
    simulator->render(); // Render the simulator to take into account modification
    vpDisplay::display(simulator->Image); // Display ultrasound probe image
    vpDisplay::flush(simulator->Image);
    imgb = simulator->Image; // Copy current ultrasound image to fourth image of the thin volume

    world_poseProbe = init_world_poseProbe; // Modify probe pose to observe third (central) slice of the thin volume
    simulator->setProbePosition(world_poseProbe);  // Set the simulator with this new probe pose
    simulator->render(); // Render the simulator to take into account modification
    vpDisplay::display(simulator->Image); // Display ultrasound probe image
    vpDisplay::flush(simulator->Image);
    img0 = simulator->Image; // Copy current ultrasound image to third (central) image of the thin volume


    // Memorize the initial probe of the probe before the visual servoing
    vpHomogeneousMatrix bMp0;
    bMp0.buildFrom(world_poseProbe);


    ///////////////////////////////////////////////////////////////////////
    //
    // Define with the mouse the region of interest (ROI) containing the pixels intensities
    //
    ///////////////////////////////////////////////////////////////////////

    cout << "Please use the mouse to define the region of interest (ROI)" << endl;
    cout << "First clic on the top-left corner of the ROI and then move the mouse pointer" << endl;
    cout << "to the bottom-right before releasing the mouse button" << endl;

    vpDisplay::displayCharString(simulator->Image, 20, 20, "Initialize the ROI:", vpColor::red);
    vpDisplay::displayCharString(simulator->Image, 40, 20, "First clic on the top-left corner of the ROI", vpColor::red);
    vpDisplay::displayCharString(simulator->Image, 60, 20, "and move the mouse pointer to the bottom-right ", vpColor::red);
    vpDisplay::displayCharString(simulator->Image, 80, 20, "before releasing the mouse button", vpColor::red);
    vpDisplay::flush(simulator->Image);

    vpImagePoint ip0; // 2D pixel coordinate of the top-left corner of the ROI
    vpImagePoint ipf; // 2D pixel coordinate of the botom-right corner of the ROI

    vpMouseButton::vpMouseButtonType button ;
    bool bt = vpDisplay::getClick(simulator->Image, ip0, button);
    bool rect_ok  = false;
    while(!rect_ok)
    {
        vpDisplay::getPointerPosition(simulator->Image, ipf);
        vpDisplay::display(simulator->Image);
        vpDisplay::displayRectangle(simulator->Image, ip0, ipf, vpColor::blue);
        vpDisplay::flush(simulator->Image);
        rect_ok = vpDisplay::getClickUp(simulator->Image, ipf, button, false);
    }
    vpDisplay::display(simulator->Image);
    vpDisplay::displayRectangle(simulator->Image, ip0, ipf, vpColor::red);
    vpDisplay::flush(simulator->Image);

    int Wmin = (int)ip0.get_u();
    int Wmax = (int)ipf.get_u();
    int Hmin = (int)ip0.get_v();
    int Hmax = (int)ipf.get_v();

    int ROI_width, ROI_height; //width and height (size) of ROI in number of pixels
    ROI_width = Wmax-Wmin+1;
    ROI_height = Hmax-Hmin+1;


    ///////////////////////////////////////////////////////////////////////
    //
    // Initialize the current features vector, desired features vector and interaction matrix
    // and other parameters of the control law
    //
    ///////////////////////////////////////////////////////////////////////

    vpImage <unsigned char> ImageD; // Desired image
    ImageD = simulator->Image; // Set the desired image to be the current one

    vpImage <unsigned char> Idiff; // Image difference
    vpImageTools::imageDifference(ImageD ,simulator->Image, Idiff) ;

    // Create an 2D image display for the desired image
    vpDisplayX displayD(ImageD, 664, 330,"Desired image");
    vpDisplay::display(ImageD);
    vpDisplay::displayRectangle(ImageD, ip0, ipf, vpColor::cyan, false, 2);
    vpDisplay::flush(ImageD);

    // Create an 2D image display for the image difference
    vpDisplayX displayDiff(Idiff,664, 634,"Image difference");
    vpDisplay::display(Idiff);
    vpDisplay::flush(Idiff);

    vpColVector CurrentFeatures; // Current features
    vpColVector DesiredFeatures; // Desired features
    int nbPx = (Hmax-Hmin+1)*(Wmax-Wmin+1) ; // Number of features
    DesiredFeatures.resize(nbPx);
    CurrentFeatures.resize(nbPx);

    // Compute the desired features from ROI pixel intensities
#ifdef TO_COMPLETE
    // Question 2
    // Compute the visual features vector from the desired image (ImageD)
    // and from the coordinates of the ROI left-top corner (Hmin, Wmin) and right-bottom corner (Hmax, Wmax)
    //
    // DesiredFeatures[...] = ...;

#endif

    // Initialize the 3D gradient of the 2D ultrasound image
    vpImage<double> dI0dx(height, width); // x gradient components of the 2D image
    vpImage<double> dI0dy(height, width); // y gradient components of the 2D image
    vpImage<double> dI0dz(height, width); // z gradient components of the 2D image

    // Compute the 3D gradient of the ROI from the previously captured 3 slices (thin volume) using spatial derivative filters
    usImageGradient::Grad3DF3x3x3(imga, img0, imgb, dI0dx, dI0dy, dI0dz, ip0, ipf) ;

    vpMatrix Ls; // Interaction matrix related to the features
    Ls.resize(nbPx,6);
    vpMatrix Lspi; // Inverse of the interaction matrix
    Lspi.resize(6,nbPx);

    // Compute the interaction matrix for visual servoing based on intensity features at the desired location
    Ls = cptImgLs::Ls(dI0dx, dI0dy, dI0dz, sx, sy, ip0, ipf);

    vpImage <unsigned char> Image; // temporary image

    // Initialize sum of visual features error to 0 for tracking error reduction
    vpColVector sum_dedt(nbPx);
    sum_dedt = 0;


    ///////////////////////////////////////////////////////////////////////
    //
    // Compute the ground truth relative desired pose between volume and probe cartesian frames
    // that should be obtained at the convergence of the visual servoing
    //
    ///////////////////////////////////////////////////////////////////////

    vpPoseVector world_poseVolume; // pose ot the US volume frame with respect to the world frame
    simulator->getObjectPosition(world_poseVolume); // obtain the pose ot the US volume frame with respect to the world frame
    vpHomogeneousMatrix worldMvolume;
    worldMvolume.buildFrom(world_poseVolume); // Convert this vector pose in a homogeneous matrix

    simulator->getProbePosition(world_poseProbe); // obtain the probe pose with respect to the world frame
    worldMprobe.buildFrom(world_poseProbe); // Convert this vector pose in a homogeneous matrix

    vpHomogeneousMatrix probeMvolume; // homogeneous matrix describing the pose of the volume with respect to the probe frame
    probeMvolume = worldMprobe.inverse()*worldMvolume;

    // compute the desired relative pose (ground truth) of the volume with respect to the probe frame that should be obtained at the convergence
    vpHomogeneousMatrix ground_truth_desired_probeMvolume;
    ground_truth_desired_probeMvolume = probeMvolume; // Set the desired relative pose to be the initial one

    ///////////////////////////////////////////////////////////////////////
    //
    // Inititialisation of curve plotters
    //
    ///////////////////////////////////////////////////////////////////////


    // Create curve plotters
    vpVelocityPlotter velocityPlotter(664+400, 0);
    vpPoseErrorPlotter poseErrorPlotter(664+400, 370);
    vpVisualErrorCostPlotter visualErrorCostPlotter(664+400, 740);

    // Move the probe to another initial pose before lauching the visal servoing
#ifdef TO_COMPLETE
    // Question 13
    // Modify the initial pose of the probe by applying to its cartesian frame
    // a relative translation of 0.002 meter along x axis
    // a relative translation of 0.002 meter along y axis
    // a relative translation of 0.002 meter along z axis
    // a relative roration of 2 deg around x axis
    // a relative roration of 2 deg around y axis
    // a relative roration of 4 deg around z axis
    //
    // world_poseProbe[...] = world_poseProbe[...] + ...; //deg has to be conveted in rad with vpMath::rad()

    // simulator->setProbePosition(world_poseProbe); // Set the probe pose to the new location
    // simulator->render(); // Render the simulator to take into account modification
#endif

    // For waiting a click on the image before lauching the visual servoing
    vpDisplay::display(simulator->Image) ; // Display the current image
    vpDisplay::displayRectangle(simulator->Image, ip0, ipf, vpColor::cyan, false, 2); // Display the ROI
    vpDisplay::displayCharString(simulator->Image, 20, 20, "Click to launch the visual seroing", vpColor::red);
    vpDisplay::flush(simulator->Image);  // Force to flush all the display associated to simulator->Image
    vpDisplay::getClick(simulator->Image, true); // wait a click to launch the visual servoing

    ///////////////////////////////////////////////////////////////////////
    //
    // Control loop
    //
    ///////////////////////////////////////////////////////////////////////
    try  {
        while(1)
        {
            simulator->render(); // Render the simulator to take into account modification

            // Display the ultrasound current image with the ROI
            vpDisplay::display(simulator->Image) ;
            vpDisplay::displayRectangle(simulator->Image, ip0, ipf, vpColor::cyan, false, 2);

            vpDisplay::displayCharString(simulator->Image, 20, 20, "A click to exit...", vpColor::red);
            // Force to flush all the display associated to simulator->Image
            vpDisplay::flush(simulator->Image);


            Image = simulator->Image; // copy ultrasound image in temporary image


#ifdef TO_COMPLETE
            // Question 5
            // Update the current visual features vector from the current image (Image)
            // using the coordinates of the ROI left-top corner (Hmin, Wmin) and right-bottom corner (Hmax, Wmax)
            //
            // CurrentFeatures[...] = ...;

#endif

            vpImageTools::imageDifference(ImageD, simulator->Image, Idiff) ; //
            vpDisplay::flush(Idiff);
            vpDisplay::display(Idiff);

#ifdef TO_COMPLETE
            // Question 6
            // Compute the control law that provides the velocity to apply to the probe
            //
            //  vpColVector VisualFeatureError;
            //
            // VisualFeatureError = ...; // Compute the visual feature error
            // probe_velocity = ...; // Compute the probe velocity

#endif


#ifdef TO_COMPLETE
            // Question 12
            // Compute the integral term and add it to the control law to compensate the tracking error
            // double opt_mu = 0.01;
            // probe_velocity -= opt_mu * ...;
#endif


            // Compute the visual error cost function
            double cost = 0;
#ifdef TO_COMPLETE
            // Question 7
            // Compute the current value of the normalized visual cost function.
            // To compute the Euclidean norm you can use the euclideanNorm() method of the vpColVector class of VisP
            //
            // cost = ...;

#endif


            // Apply sinusoidal motion to the volume to simulate physiological motion
#ifdef TO_COMPLETE
            // Question 1
            // Update the object_velocity in order to apply sinusoidal signals of amplitude of 0.005 (meter)
            // on the 3 translational components and amplitude of 5 (deg) on the 3 rotational components.
            // All sisunoidal signals should have a period of 5 seconds. The current time is given by
            // the already defined variable (double)time
            //
            // Question 11
            // Same than Question 1 but with sisusoidal amplitude of 0.02 (meter)
            // on the 3 translational components and amplitude of 20 (deg) on the 3 rotational components.
            //
            // object_velocity[0] = ...; // translation (translational velocity has to be provided in meter/s)
            // object_velocity[1] = ...; // translation (translational velocity has to be provided in meter/s)
            // object_velocity[2] = ...; // translation (translational velocity has to be provided in meter/s)
            // object_velocity[3] = ...; // rotation (angular velocity has to be provided in rad/s. Use vpMath::rad() for converting degree to rad)
            // object_velocity[4] = ...; // rotation (angular velocity has to be provided in rad/s. Use vpMath::rad() for converting degree to rad)
            // object_velocity[5] = ...; // rotation (angular velocity has to be provided in rad/s. Use vpMath::rad() for converting degree to rad)
#endif

            simulator->sendControlVelocity(probe_velocity, object_velocity, period);

            // update the current time
            time = time + period;

            // wait the time of a period (in ms) before looping
            vpTime::wait(period*1000);


            ///////////////////////////////////////////////
            /// Code for the curve plotters
            ///
            /// The following code is only used for comparing the current relative pose between
            /// the volume and probe frames to the desired relative pose that we want to achieve (ground thruth)
            /// It does not affect the previous control law that is only based on intensities features

            // Compute the current relative pose between volume and probe cartesian frames
            simulator->getObjectPosition(world_poseVolume); // obtain the current pose ot the US volume frame with respect to the world frame
            vpHomogeneousMatrix worldMvolume;
            worldMvolume.buildFrom(world_poseVolume); // Convert this vector pose in a homogeneous matrix

            simulator->getProbePosition(world_poseProbe); // obtain the current probe pose with respect to the world frame
            vpHomogeneousMatrix worldMprobe;
            worldMprobe.buildFrom(world_poseProbe); // Convert this vector pose in a homogeneous matrix

            probeMvolume = worldMprobe.inverse()*worldMvolume; // homogeneous matrix describing the pose of the volume with respect to the probe frame

            // compute the error between the relative current pose and the desired relative pose that should be reached at the convergence
            vpHomogeneousMatrix error_volumeMprobe; // positionning error of the probe with respect to the volume
            error_volumeMprobe = ground_truth_desired_probeMvolume.inverse()*probeMvolume;
            vpPoseVector error_volume_poseProbe; // convert this homogeneous matrix to a vpPoseVector
            error_volume_poseProbe.buildFrom(error_volumeMprobe);

            // Update the curves plotter
            velocityPlotter.update(time, probe_velocity);
            poseErrorPlotter.update(time, error_volume_poseProbe);
            visualErrorCostPlotter.update(time, cost);

            // A mouse click to exit the loop
            if (vpDisplay::getClick(simulator->Image, false)) {
                break;
            }
        }
    }
    catch(...) {
        // A click to exit
        vpDisplay::displayCharString(simulator->Image, 20, 20, "Catch an error: a click to exit...", vpColor::red);
        vpDisplay::flush(simulator->Image);
        vpDisplay::getClick(simulator->Image);
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // free memory
    //
    ///////////////////////////////////////////////////////////////////////

    if(simulator != NULL) delete simulator;
    if(mVTKWindow) mVTKWindow->Delete(); mVTKWindow=NULL;
    delete [] filename;
}
