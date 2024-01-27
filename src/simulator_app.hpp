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

    static constexpr int HISTORY_LENGTH = 1000;

    static const float INIT_CONTACT_CONSTRAINT_MAX_EE_DISTANCE_REL;
    static const float INIT_CONTACT_CONSTRAINT_MIN_CN_DISTANCE_REL;
    static const float INIT_CONTACT_CONSTRAINT_MIN_CC_DISTANCE_REL;


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
    void initGUI();
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

    float getMinEdgeLength();
    int addAllContactConstraints();

    // - - - -

    Json::Value sceneConfig_;

    CameraProjection cameraMode_;
    CameraViewpoint cameraViewpoint_;
    vcg::Trackball trackball_;

    std::vector<Net*> nets_;
    int totNodes_;

    UnaryForce* force_;

    bool softNodeDragging_;
    FixedNodesForce fixedNodesForce_;
    std::shared_ptr<FixedNodeConstraint> fixedCs_;

    std::vector<std::shared_ptr<EdgeLengthConstraint>> edgeLenCs_;
    std::vector<std::shared_ptr<ShearLimitConstr>> shearLimitCs_;
    std::shared_ptr<ConstraintTask> collisionCs_;
    std::shared_ptr<ContactConstraint> contactCs_;
    ConstraintSolver* solver_;
    int solverSpsCap_;

    float maxEEDistRel_;
    float minCNDistRel_;
    float minCCDistRel_;

    std::atomic<bool> playSim_;
    std::array<float, HISTORY_LENGTH> nItersHist_;
    std::array<float, HISTORY_LENGTH> meanDeltaHist_;
    int histNextIndex_;
    std::thread simThread_;
    bool byebye_;

    Pick pick_;

};


#endif