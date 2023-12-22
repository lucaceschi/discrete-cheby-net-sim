#ifndef SIMAPP_H_
#define SIMAPP_H_

#include "framework/app.hpp"
#include "net.hpp"
#include "forces.hpp"
#include "constraint_solver.hpp"

#include <string>
#include <vector>
#include <thread>

#include <json/json.h>
#include <Eigen/Dense>
#include <wrap/gui/trackball.h>


class SimulatorApp : public frmwrk::App
{
private:
    static const Eigen::Vector2i WINDOW_SIZE;
    static const bool WINDOW_RESIZABLE;
    static const GLclampf RENDER_BG_COLOR[3];

    enum class CameraProjection { PERSP, ORTHO };
    enum class CameraViewpoint { TOP, FRONT, RIGHT };

public:
    SimulatorApp(std::string sceneName, Json::Value sceneConfig);
    virtual ~SimulatorApp();

private:
    virtual bool initApp();
    virtual bool mainLoop(double delta);
    virtual void quitApp();

    // - - - -

    void setProjectionMatrix();
    void setModelViewMatrix();

    void drawGridRenderMode();
    void drawGridPickingMode();
    void drawGUI();

    void handleMouseEvents();
    void handleKeyEvents();

    void simulate();

    // - - - -

    Json::Value sceneConfig_;

    CameraProjection cameraMode_;
    CameraViewpoint cameraViewpoint_;
    vcg::Trackball trackball_;

    int nNets_;
    std::vector<Net*> nets_;
    int totNodes_;

    Force* force_;
    bool applyForce_;
    
    ConstraintSolver* solver_;
    int solverFpsCap_;

    bool playSim_;
    std::thread simThread_;
    bool byebye_;

};

#endif