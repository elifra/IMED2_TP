#ifndef vpCurvePlotter_h
#define vpCurvePlotter_h

#include <visp/vpPlot.h> // For the curves

/*!
  Class that allows to plot the probe velocities vx, vy, vz, wx, wy, wz during the visual servo.
*/
class vpVelocityPlotter
{
public:
    /*!
    Initialize the curve plotter.
    \param pos_x : Horizontal position of the viewer.
    \param pos_y : Vertical position of the viewer.
  */
    vpVelocityPlotter(const unsigned int pos_x, const unsigned int pos_y)
    {
        // Create a window with two graphics
        plot_.init(2, 320, 600, pos_x, pos_y);
        plot_.initGraph(0, 3); // Probe velocities vx,vy,vz
        plot_.initGraph(1, 3); // Probe velocities wx,wy,wz
        plot_.setTitle(0, "Probe translational velocities (meter/s)");
        plot_.setTitle(1, "Probe rotational velocities (deg/s)");
        plot_.initRange(0,0, 60, -1e-6, 10e-6);
        plot_.initRange(1,0, 60, -1e-6, 10e-6);
        // Set the curves legend
        char legend[10];
        sprintf(legend, "vx");
        plot_.setLegend(0, 0, legend);
        sprintf(legend, "vy");
        plot_.setLegend(0, 1, legend);
        sprintf(legend, "vz");
        plot_.setLegend(0, 2, legend);
        sprintf(legend, "wx");
        plot_.setLegend(1, 0, legend);
        sprintf(legend, "wy");
        plot_.setLegend(1, 1, legend);
        sprintf(legend, "wz");
        plot_.setLegend(1, 2, legend);
    }

    /*!
    Update the curves with the new values.
    \param time : time.
    \param v : Camera velocity vector of dimension 6 with vx,vy,vz,wx,wy,wz
  */
    void update(double time, const vpColVector &v)
    {
        // Update the curves
        vpColVector temp(3);
        for (unsigned int i=0; i < 3; i++)
        {
            temp[i] = v[i];
        }
        plot_.plot(0, time, temp); // plot probe translational velocities

        for (unsigned int i=0; i < 3; i++)
        {
            temp[i] = vpMath::deg(v[i+3]);
        }
        plot_.plot(1, time, temp); // plot probe rotational velocities
    }
private:
    vpPlot plot_;
};

/*!
  Class that allows to plot the pose error (relative pose with thetaU representation)
  between the ultrasound probe and volume cartesian frames tx, ty, tz, thetaU_x, thetaU_y, thetaU_z during the visual servo.
*/
class vpPoseErrorPlotter
{
public:
    /*!
    Initialize the curve plotter.
    \param pos_x : Horizontal position of the viewer.
    \param pos_y : Vertical position of the viewer.
  */
    vpPoseErrorPlotter(const unsigned int pos_x, const unsigned int pos_y)
    {
        // Create a window with two graphics
        plot_.init(2, 320, 600, pos_x, pos_y);
        plot_.initGraph(0, 3); // Probe pose error (vpPoseVector format)
        plot_.initGraph(1, 3); // Probe pose error (vpPoseVector format)
        plot_.setTitle(0, "Probe pose error: translations (meter)");
        plot_.setTitle(1, "Probe pose error: rotations (deg)");
        plot_.initRange(0,0, 60, -1e-6, 10e-6);
        plot_.initRange(1,0, 60, -1e-6, 10e-6);
        // Set the curves legend
        char legend[10];
        sprintf(legend, "tx");
        plot_.setLegend(0, 0, legend);
        sprintf(legend, "ty");
        plot_.setLegend(0, 1, legend);
        sprintf(legend, "tz");
        plot_.setLegend(0, 2, legend);
        sprintf(legend, "thetaU_x");
        plot_.setLegend(1, 0, legend);
        sprintf(legend, "thetaU_y");
        plot_.setLegend(1, 1, legend);
        sprintf(legend, "thetaU_z");
        plot_.setLegend(1, 2, legend);
    }

    /*!
    Update the curves with the new values.
    \param time : time.
    \param pose : Pose error between the ultrasound probe and volume cartesian frames
  */
    void update(double time, const vpPoseVector &pose)
    {
        // Update the curves
        vpColVector trans(3);
        vpColVector rota(3);
        for (unsigned int i=0; i < 3; i++)
        {
            trans[i] = pose[i];
            rota[i] = vpMath::deg(pose[i+3]);
        }
        plot_.plot(0, time, trans); // translational compoments of the pose
        plot_.plot(1, time, rota); // rotational compoments of the pose
    }
private:
    vpPlot plot_;
};

/*!
  Class that allows to plot the visual error cost function during the visual servo.
*/
class vpVisualErrorCostPlotter
{
public:
    /*!
    Initialize the curve plotter.
    \param pos_x : Horizontal position of the viewer.
    \param pos_y : Vertical position of the viewer.
  */
    vpVisualErrorCostPlotter(const unsigned int pos_x, const unsigned int pos_y)
    {
        // Create a window with one graphics
        plot_.init(1, 320, 600, pos_x, pos_y);
        plot_.initGraph(0, 1); // visual error cost
        plot_.setTitle(0, "visual error cost function");
        plot_.initRange(0,0, 60, 0, 10e-6);
    }

    /*!
    Update the curves with the new values.
    \param time : time.
    \param cost : visual error cost.
  */
    void update(double time, const double cost)
    {
        // Update the curves
        vpColVector temp(1);
        temp[0] = cost;
        plot_.plot(0, time, temp); // plot visual error cost
    }

private:
    vpPlot plot_;
};


#endif
