#include "simulator_app.hpp"
#include "simulator_app_gui.hpp"
#include "sdfs.hpp"
#include "framework/debug.hpp"

#include <iostream>
#include <chrono>
#include <vector>
#include <map>

#include <Eigen/Dense>
#include <imgui.h>
#include <GL/glew.h>


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
      nets_(),
      totNodes_(0),
      force_(nullptr),
      applyForce_(true),
      solver_(nullptr),
      playSim_(false),
      simThread_(),
      byebye_(false),
      pick_()
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

        solver_ = new ConstraintSolver(nets_, absTol, relTol, maxIters);
        
        // initialize nets
        
        Json::Value& netsArray = sceneConfig_["nets"];
        if(netsArray.isNull() || !netsArray.isArray() || netsArray.size() == 0)
            throw "No nets specified";

        int nNets = netsArray.size();
        nets_.reserve(nNets);
        edgeLenCs_.reserve(nNets);
        for(int n = 0; n < nNets; n++)
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

            edgeLenCs_.push_back(std::make_shared<EdgeLengthConstraint>(nets_, n));
            solver_->addConstraint(edgeLenCs_.back());

            shearLimitCs_ = std::make_shared<ShearLimitConstr>(n, edgeLength, shearLimit);
            solver_->addConstraint(shearLimitCs_);
        }

        fixedCs_ = std::make_shared<FixedNodeConstraint>(nets_);
        solver_->addConstraint(fixedCs_);

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

                collisionCs_ = std::make_shared<SphereCollConstr>(origin, radius);
                solver_->addConstraint(collisionCs_);
            }
            else if(colliderType == "planar_boundary")
            {
                Eigen::Vector3f point;
                TRY_PARSE("Parsing point of planar boundary collider",
                    point = Eigen::Vector3f(colliderObj["point"][0].asFloat(),
                                            colliderObj["point"][1].asFloat(),
                                            colliderObj["point"][2].asFloat());
                );

                Eigen::Vector3f normal;
                TRY_PARSE("Parsing normal of planar boundary collider",
                    normal = Eigen::Vector3f(colliderObj["normal"][0].asFloat(),
                                            colliderObj["normal"][1].asFloat(),
                                            colliderObj["normal"][2].asFloat());
                );

                collisionCs_ = std::make_shared<PlanarBoundaryConstr>(point, normal);
                solver_->addConstraint(collisionCs_);
            }
            else if(colliderType == "sdf")
            {
                using SDFCollConstrF = SDFCollConstr<float(Eigen::Vector3f), Eigen::Vector3f(Eigen::Vector3f)>;

                std::string preset;
                TRY_PARSE("Parsing preset of sdf collider",
                    preset = colliderObj["preset"].asString();
                );

                if(preset == "nut")
                    collisionCs_ = std::make_shared<SDFCollConstrF>(SDFs::Nut::sdf, SDFs::Nut::dsdf, false);
                else if(preset == "torus")
                    collisionCs_ = std::make_shared<SDFCollConstrF>(SDFs::Torus::sdf, SDFs::Torus::dsdf, true);
                else
                    throw "Invalid preset for sdf collider";

                solver_->addConstraint(collisionCs_);
            }
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

    drawNetsRenderMode();
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

void SimulatorApp::drawNetsRenderMode()
{
    glClearColor(RENDER_BG_COLOR[0], RENDER_BG_COLOR[1], RENDER_BG_COLOR[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    for(int netIdx = 0; netIdx < nets_.size(); netIdx++)
    {
        Net& net = *nets_[netIdx];
        
        // Draw edges

        glColor3ubv(net.renderColor);
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
        for(int nodeIdx = 0; nodeIdx < net.getNNodes(); nodeIdx++)
        {
            if(fixedCs_->isNodeFixed(netIdx, nodeIdx)
               || (pick_.pick && pick_.netIdx == netIdx && pick_.nodeIdx == nodeIdx))
                glColor3ub(0xff, 0x00, 0x00);
            else
                glColor3ubv(net.renderColor);
            
            const Eigen::Vector3f nodePos = net.nodePos(nodeIdx);
            glVertex3d(nodePos(0), nodePos(1), nodePos(2));
        }
        glEnd();
    }
}

void SimulatorApp::simulate() {
    const std::chrono::duration TIME_STEP = std::chrono::nanoseconds(1000000000) / solverFpsCap_;

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


void SimulatorApp::cutNets()
{
    const Eigen::Vector3f cutPlaneNormal = {0, 1, 0};

    std::vector< std::vector<Eigen::Vector3f> > newNodes        (nets_.size());
    std::vector< std::unordered_map<int, int> > nodeIndexMap    (nets_.size());
    std::vector< std::vector<Eigen::Array2i>  > newEdges        (nets_.size());
    //std::vector< std::unordered_map<int, int> > edgeIndexMaps   (nets_.size());
    std::vector< std::vector<float>           > newEdgeLengths  (nets_.size());
    std::vector< std::vector<Net::Quad>       > newQuads        (nets_.size());
    std::vector< std::vector<Eigen::Array2i>  > newDiagonals    (nets_.size());
    std::vector< std::vector<int>             > newFixedNodes   (nets_.size());
    
    totNodes_ = 0;
    // cutting edges occurs for each net independently
    for(int netIdx = 0; netIdx < nets_.size(); netIdx++)
    {
        Net* net = nets_[netIdx];

        // first, preserve all nodes above the cutting plane
        for(int nodeIdx = 0; nodeIdx < net->getNNodes(); nodeIdx++)
        {
            const Eigen::Vector3f p = net->nodePos(nodeIdx);

            if(p(1) < 0)
                continue;

            newNodes[netIdx].push_back(p);
            nodeIndexMap[netIdx][nodeIdx] = newNodes[netIdx].size() - 1;
        }

        // only preserve quads formed by nodes above the cutting plane
        for(int quadIdx = 0; quadIdx < net->quads.size(); quadIdx++)
        {
            auto node1IndexPair = nodeIndexMap[netIdx].find(net->quads[quadIdx].node1Index);
            auto node2IndexPair = nodeIndexMap[netIdx].find(net->quads[quadIdx].node2Index);
            auto node3IndexPair = nodeIndexMap[netIdx].find(net->quads[quadIdx].node3Index);
            auto node4IndexPair = nodeIndexMap[netIdx].find(net->quads[quadIdx].node4Index);

            if(node1IndexPair != nodeIndexMap[netIdx].cend() &&
               node2IndexPair != nodeIndexMap[netIdx].cend() &&
               node3IndexPair != nodeIndexMap[netIdx].cend() &&
               node4IndexPair != nodeIndexMap[netIdx].cend())
            {
                newQuads[netIdx].emplace_back(Net::Quad{node1IndexPair->second,
                                                        node2IndexPair->second,
                                                        node3IndexPair->second,
                                                        node4IndexPair->second});

                newDiagonals[netIdx].emplace_back(node1IndexPair->second, node3IndexPair->second);
                newDiagonals[netIdx].emplace_back(node2IndexPair->second, node4IndexPair->second);
            }
        }

        for(int edgeIdx = 0; edgeIdx < net->getNEdges(); edgeIdx++)
        {
            auto p1IndexPair = nodeIndexMap[netIdx].find(net->edge(edgeIdx)[0]);
            auto p2IndexPair = nodeIndexMap[netIdx].find(net->edge(edgeIdx)[1]);

            // scrap edges below the cutting plane
            if(p1IndexPair == nodeIndexMap[netIdx].end() && p2IndexPair == nodeIndexMap[netIdx].end())
                continue;

            int p1Index, p2Index;
            double edgeLength;
            
            if(p1IndexPair != nodeIndexMap[netIdx].end() && p2IndexPair != nodeIndexMap[netIdx].end())
            {
                // edge is above the cutting plane, preserve it
                p1Index = p1IndexPair->second;
                p2Index = p2IndexPair->second;
                edgeLength = net->edgeLengths[edgeIdx];
            }
            else
            {
                // edge crosses the cutting plane: retrieve the node below the cutting plane
                // and move it to the closest intersection between the edge and the plane

                Eigen::Vector3f newNodePos;
                Eigen::Vector3f edgeVec = (net->nodePos(net->edge(edgeIdx)[0])
                                           - net->nodePos(net->edge(edgeIdx)[1])).normalized();

                if(p1IndexPair == nodeIndexMap[netIdx].end())
                {
                    newNodePos = net->nodePos(net->edge(edgeIdx)[0]);
                    p1Index = newNodes[netIdx].size();
                    p2Index = p2IndexPair->second;
                }
                else
                {
                    newNodePos = net->nodePos(net->edge(edgeIdx)[1]);
                    p1Index = p1IndexPair->second;
                    p2Index = newNodes[netIdx].size();
                }

                double deltaLength = newNodePos.dot(cutPlaneNormal) / edgeVec.dot(cutPlaneNormal);

                newNodePos -= (deltaLength * edgeVec);
                edgeLength = net->edgeLengths[edgeIdx] - abs(deltaLength);
                newNodes[netIdx].push_back(newNodePos);
                newFixedNodes[netIdx].push_back(newNodes[netIdx].size());
            }

            newEdges[netIdx].emplace_back(p1Index, p2Index);
            newEdgeLengths[netIdx].push_back(edgeLength);
        }

        // create the new net

        Eigen::Matrix3Xf newNodeMatrix(3, newNodes[netIdx].size());
        for(int nodeIdx = 0; nodeIdx < newNodes[netIdx].size(); nodeIdx++)
            newNodeMatrix.col(nodeIdx) = newNodes[netIdx][nodeIdx];

        Eigen::Array2Xi newEdgesMatrix(2, newEdges[netIdx].size());
        Eigen::ArrayXf newEdgesLengthsArray(newEdges[netIdx].size());
        for(int edgeIdx = 0; edgeIdx < newEdges[netIdx].size(); edgeIdx++)
        {
            newEdgesMatrix.col(edgeIdx) = newEdges[netIdx][edgeIdx];
            newEdgesLengthsArray[edgeIdx] = newEdgeLengths[netIdx][edgeIdx];
        }

        Eigen::Array2Xi newDiagonalsMatrix(2, newDiagonals[netIdx].size());
        for(int diagIdx = 0; diagIdx < newDiagonals[netIdx].size(); diagIdx++)
            newDiagonalsMatrix.col(diagIdx) = newDiagonals[netIdx][diagIdx];

        GLubyte netColor[3];
        memcpy(netColor, nets_[netIdx]->renderColor, 3);
        delete nets_[netIdx];
        nets_[netIdx] = new Net(newNodeMatrix,
                                newEdgesMatrix,
                                newEdgesLengthsArray,
                                newDiagonalsMatrix,
                                newQuads[netIdx],
                                netColor);

        totNodes_ += newNodes[netIdx].size();

        // update constraints

        edgeLenCs_[netIdx]->updateEdgeLengths(nets_);

        fixedCs_->freeNodes(netIdx);
        for(int i : newFixedNodes[netIdx])
            fixedCs_->fixNode(nets_, netIdx, i);
    }
}