#include "simulator_app.hpp"
#include "simulator_app_gui.hpp"
#include "framework/debug.hpp"

#include <iostream>
#include <chrono>

#include <Eigen/Dense>
#include <imgui.h>


#define TRY_PARSE(errstr, ...) do {                                                                 \
        try { __VA_ARGS__ }                                                                         \
        catch(const Json::LogicError& e) { throw (std::string(errstr) + ": " + e.what()).c_str(); } \
    } while(0)


const Eigen::Vector2i SimulatorApp::WINDOW_SIZE = {1200, 800};
const bool SimulatorApp::WINDOW_RESIZABLE = true;
const GLclampf SimulatorApp::RENDER_BG_COLOR[3] = {0.95, 0.95, 0.95};

SimulatorApp::SimulatorApp(std::string sceneName, Json::Value sceneConfig)
    : App(sceneName, SimulatorApp::WINDOW_SIZE, SimulatorApp::WINDOW_RESIZABLE),
      sceneConfig_(sceneConfig),
      cameraMode_(CameraProjection::PERSP),
      cameraViewpoint_(CameraViewpoint::TOP),
      trackball_(),
      nNets_(0),
      nets_(),
      totNodes_(0),
      force_(nullptr),
      applyForce_(true),
      solver_(nullptr),
      playSim_(false),
      simThread_(),
      byebye_(false)
{}

bool SimulatorApp::initApp()
{    
    try
    {
        // initialize constraint solver

        Json::Value& solverObj = sceneConfig_["constraint_solver"];
        if(solverObj.isNull() || !solverObj.isObject())
            throw "No configuration for constraint_solver found";

        float absTol;
        TRY_PARSE("Parsing absolute_tolerance of constraint solver",
            absTol = solverObj["absolute_tolerance"].asFloat();
        );

        float relTol;
        TRY_PARSE("Parsing relative_tolerance of constraint solver",
            relTol = solverObj["relative_tolerance"].asFloat();
        );

        int maxIters;
        TRY_PARSE("Parsing max_iters of constraint solver",
            maxIters = solverObj["max_iters"].asInt();
        );

        TRY_PARSE("Parsing max_fps of constraint solver",
            solverFpsCap_ = solverObj["max_fps"].asInt();
        );

        solver_ = new ConstraintSolver(nets_, nNets_, absTol, relTol, maxIters);
        
        // initialize nets
        
        Json::Value& netsArray = sceneConfig_["nets"];
        if(netsArray.isNull() || !netsArray.isArray() || netsArray.size() == 0)
            throw "No nets specified";
        nNets_ = netsArray.size();
        nets_.reserve(nNets_);

        for(int n = 0; n < netsArray.size(); n++)
        {
            Eigen::Vector2i size;
            TRY_PARSE("Parsing size of net " + std::to_string(n), 
                size = Eigen::Vector2i(netsArray[n]["size"][0].asInt(),
                                       netsArray[n]["size"][1].asInt());
            );

            Eigen::Vector3f center;
            TRY_PARSE("Parsing center of net " + std::to_string(n),
                center = Eigen::Vector3f(netsArray[n]["center"][0].asFloat(),
                                         netsArray[n]["center"][1].asFloat(),
                                         netsArray[n]["center"][2].asFloat());
            );

            Eigen::Vector3f xTangVec;
            TRY_PARSE("Parsing x_tangent_vector of net " + std::to_string(n),
                xTangVec = Eigen::Vector3f(netsArray[n]["x_tangent_vector"][0].asFloat(),
                                           netsArray[n]["x_tangent_vector"][1].asFloat(),
                                           netsArray[n]["x_tangent_vector"][2].asFloat());
            );

            Eigen::Vector3f yTangVec;
            TRY_PARSE("Parsing y_tangent_vector of net " + std::to_string(n),
                yTangVec = Eigen::Vector3f(netsArray[n]["y_tangent_vector"][0].asFloat(),
                                           netsArray[n]["y_tangent_vector"][1].asFloat(),
                                           netsArray[n]["y_tangent_vector"][2].asFloat());
            );

            float edgeLength;
            TRY_PARSE("Parsing edge_length of net " + std::to_string(n),
                edgeLength = netsArray[n]["edge_length"].asFloat();
            );

            float shearLimit;
            TRY_PARSE("Parsing shear_angle_limit of net " + std::to_string(n),
                shearLimit = netsArray[n]["shear_angle_limit"].asFloat();
            );

            GLubyte color[3];
            TRY_PARSE("Parsing color of net " + std::to_string(n),
                for(int i = 0; i < 3; i++)
                {
                    unsigned int c = netsArray[n]["color"][i].asUInt();
                    color[i] = (GLubyte)((c < 255)? c : 255);
                }
            );

            nets_.push_back(new Net(size, edgeLength, center, xTangVec, yTangVec, color));
            totNodes_ += nets_[n]->getNNodes();

            solver_->addConstraint(std::make_unique<EdgeLengthConstraint>(n, edgeLength));
            solver_->addConstraint(std::make_unique<ShearLimitConstr>(n, edgeLength, shearLimit));
        }

        // initialize force        

        Json::Value& forceObj = sceneConfig_["force"];
        if(forceObj.isNull() || !forceObj.isObject())
            throw "No force specified";
        
        std::string forceType;
        TRY_PARSE("Parsing type of force",
            forceType = forceObj["type"].asString();
        );

        if(forceType == "constant")
        {
            Eigen::Vector3f translationVec;
            TRY_PARSE("Parsing translation_vector of constant force",
                translationVec = Eigen::Vector3f(forceObj["translation_vector"][0].asFloat(),
                                                 forceObj["translation_vector"][1].asFloat(),
                                                 forceObj["translation_vector"][2].asFloat());
            );

            force_ = new ConstantForce(translationVec);
        }
        // TODO: else if...
        else
            throw "invalid type of force";

        // initialize collider

        Json::Value& colliderObj = sceneConfig_["collider"];
        if(!colliderObj.isNull() && colliderObj.isObject())
        {
            std::string colliderType;
            TRY_PARSE("Parsing type of force",
                colliderType = colliderObj["type"].asString();
            );

            if(colliderType == "sphere")
            {
                Eigen::Vector3f origin;
                TRY_PARSE("Parsing origin of sphere collider",
                    origin = Eigen::Vector3f(colliderObj["origin"][0].asFloat(),
                                            colliderObj["origin"][1].asFloat(),
                                            colliderObj["origin"][2].asFloat());
                );

                float radius;
                TRY_PARSE("Parsing radius of sphere collider",
                    radius = colliderObj["radius"].asFloat();
                );

                for(int n = 0; n < nNets_; n++)
                    solver_->addConstraint(std::make_unique<SphereCollConstr>(n, origin, radius));
            }
            // TODO: else if...
            else
                throw "invalid type of collider";
        }
    }
    catch(const char* s)
    {
        std::cerr << s << std::endl;
        return false;
    }

    initGUI();

    simThread_ = std::thread(&SimulatorApp::simulate, this);
    
    return true;
}

void SimulatorApp::quitApp()
{
    byebye_ = true;
    simThread_.join();
}

SimulatorApp::~SimulatorApp()
{
    for(int n = 0; n < nets_.size(); n++)
        delete nets_[n];
    
    if(solver_ != nullptr)
        delete solver_;

    if(force_ != nullptr)
        delete force_;
}

bool SimulatorApp::mainLoop(double delta)
{
    setProjectionMatrix();
    setModelViewMatrix();

    handleMouseEvents();
    handleKeyEvents();

    drawGridRenderMode();
    drawGUI();
    
    return true;
}

void SimulatorApp::setProjectionMatrix()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    GLfloat framebufferRatio = (GLfloat)getFramebufferSize()(0) / getFramebufferSize()(1);
    if(cameraMode_ == CameraProjection::PERSP)
        gluPerspective(40, framebufferRatio, 0.1, 100);
    else
        glOrtho(-1.25 * framebufferRatio, 1.25 * framebufferRatio, -1.25, 1.25, -10, 10);        
}

void SimulatorApp::setModelViewMatrix()
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    switch(cameraViewpoint_)
    {
        case CameraViewpoint::TOP:
            gluLookAt(0,4,0,   0,0,0,   0,0,1);
            break;
        case CameraViewpoint::RIGHT:
            gluLookAt(-4,0,0,   0,0,0,   0,1,0);
            break;
        case CameraViewpoint::FRONT:
        default:
            gluLookAt(0,0,-4,  0,0,0,   0,1,0);
            break;
    }

    trackball_.GetView();
    trackball_.Apply();
}

void SimulatorApp::drawGridRenderMode()
{
    glClearColor(RENDER_BG_COLOR[0], RENDER_BG_COLOR[1], RENDER_BG_COLOR[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    for(int netIdx = 0; netIdx < nNets_; netIdx++)
    {
        Net& net = *nets_[netIdx];
        
        // Draw edges

        glColor3ubv(net.color);
        glLineWidth(2.0f * trackball_.track.sca);

        glBegin(GL_LINES);
        for(int e = 0; e < net.getNEdges(); e++)
        {
            Eigen::Vector3f nodePosA = net.nodePos(net.edge(e)[0]);
            Eigen::Vector3f nodePosB = net.nodePos(net.edge(e)[1]);
            glVertex3d(nodePosA(0), nodePosA(1), nodePosA(2));
            glVertex3d(nodePosB(0), nodePosB(1), nodePosB(2));
        }
        glEnd();

        // Draw nodes

        glPointSize(10.0f * trackball_.track.sca);

        glBegin(GL_POINTS);
        for(int n = 0; n < net.getNNodes(); n++)
        {
            const Eigen::Vector3f nodePos = net.nodePos(n);
            glVertex3d(nodePos(0), nodePos(1), nodePos(2));
        }
        glEnd();
    }
}

void SimulatorApp::drawGridPickingMode() {}

void SimulatorApp::handleMouseEvents()
{
    if(ImGui::GetIO().WantCaptureMouse)
        return;

    Eigen::Vector2d curPos = input_.getCursorPos();
    curPos(0) = curPos(0) * getWindowContentScale()(0);
    curPos(1) = getFramebufferSize()(1) - curPos(1) * getWindowContentScale()(1);

    if(input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
    {
        trackball_.MouseDown((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
    }

    if(input_.isMouseButtonHeld(GLFW_MOUSE_BUTTON_LEFT))
    {
        trackball_.MouseMove((int)curPos(0), (int)curPos(1));
    }

    if(input_.isMouseButtonReleased(GLFW_MOUSE_BUTTON_LEFT))
    {
        trackball_.MouseUp((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
    }

    if(input_.getMouseScrollOffset() != 0)
    {
        trackball_.MouseWheel(input_.getMouseScrollOffset());
    }
}

void SimulatorApp::handleKeyEvents()
{
    if(ImGui::GetIO().WantCaptureKeyboard)
        return;

    if(input_.isKeyPressed(GLFW_KEY_LEFT_SHIFT))
        trackball_.ButtonDown(vcg::Trackball::KEY_SHIFT);
    if(input_.isKeyReleased(GLFW_KEY_LEFT_SHIFT))
        trackball_.ButtonUp(vcg::Trackball::KEY_SHIFT);
    if(input_.isKeyPressed(GLFW_KEY_LEFT_CONTROL))
        trackball_.ButtonDown(vcg::Trackball::KEY_CTRL);
    if (input_.isKeyReleased(GLFW_KEY_LEFT_CONTROL))
        trackball_.ButtonUp(vcg::Trackball::KEY_CTRL);
    if (input_.isKeyPressed(GLFW_KEY_LEFT_ALT))
        trackball_.ButtonDown(vcg::Trackball::KEY_ALT);
    if (input_.isKeyReleased(GLFW_KEY_LEFT_ALT))
        trackball_.ButtonUp(vcg::Trackball::KEY_ALT);

    if(input_.isKeyPressed(GLFW_KEY_SPACE))
        playSim_ = !playSim_; 
}

void SimulatorApp::simulate() {
    const std::chrono::duration TIME_STEP = std::chrono::nanoseconds(1000000000) / solverFpsCap_;
    std::cout << solverFpsCap_ << " " << TIME_STEP.count() << std::endl;

    while(!byebye_) {

        if(!playSim_)
        {
            std::this_thread::yield();
            continue;
        }

        auto startTime = std::chrono::high_resolution_clock::now();
        auto minEndTime = startTime + TIME_STEP;

        if(applyForce_)
        {
            #pragma omp parallel for schedule(static)
            for(int i = 0; i < totNodes_; i++)
            {
                int net = 0;
                int nodeIdx = i;

                while(nodeIdx >= nets_[net]->getNNodes())
                {
                    nodeIdx -= nets_[net]->getNNodes();
                    net++;
                }

                force_->applyForce(nets_[net]->nodePos(nodeIdx));
            }
        }

        
        if(solver_->solve() == ConstraintSolver::ReturnCode::REACHED_MAX_ITERS)
        {
            frmwrk::Debug::logWarning("Constraint solver reached max iters, stopping");
            playSim_ = false;
        }

        while(std::chrono::high_resolution_clock::now() < minEndTime)
            std::this_thread::yield();
    }
}