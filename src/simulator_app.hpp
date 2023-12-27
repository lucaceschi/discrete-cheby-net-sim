#ifndef SIMAPP_H_
#define SIMAPP_H_

#include "framework/app.hpp"
#include "net.hpp"
#include "forces.hpp"
#include "constraints.hpp"
#include "constraint_solver.hpp"

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <memory>

#include <json/json.h>
#include <Eigen/Dense>
#include <wrap/gui/trackball.h>
#include <GL/glew.h>


class SimulatorApp : public frmwrk::App
{
private:
    static const Eigen::Vector2i WINDOW_SIZE;
    static const bool WINDOW_RESIZABLE;
    static const GLclampf RENDER_BG_COLOR[3];
    static const GLclampf PICKING_BG_COLOR[3];

    enum class CameraProjection { PERSP, ORTHO };
    enum class CameraViewpoint { TOP, FRONT, RIGHT };

    struct Pick
    {
        Pick(int netIdx, int nodeIdx, GLfloat depth)
            : pick(true), netIdx(netIdx), nodeIdx(nodeIdx), depth(depth) {};
        Pick() : pick(false) {};
        
        bool pick;
        int netIdx;
        int nodeIdx;
        GLfloat depth;
    };

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

    void drawNetsRenderMode();
    void drawNetPickingMode(int netIdx);
    void drawGUI();

    void handleMouseEvents();
    void handleKeyEvents();

    void simulate();
    void cutNets();

    // - - - -

    Json::Value sceneConfig_;

    CameraProjection cameraMode_;
    CameraViewpoint cameraViewpoint_;
    vcg::Trackball trackball_;

    std::vector<Net*> nets_;
    int totNodes_;

    Force* force_;
    bool applyForce_;
    
    std::shared_ptr<FixedNodeConstraint> fixedCs_;
    std::vector<std::shared_ptr<EdgeLengthConstraint>> edgeLenCs_;
    std::shared_ptr<ShearLimitConstr> shearLimitCs_;
    std::shared_ptr<ConstraintTask> collisionCs_;
    ConstraintSolver* solver_;
    int solverFpsCap_;

    std::atomic<bool> playSim_;
    std::thread simThread_;
    bool byebye_;

    Pick pick_;

};


#endif