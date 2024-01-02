#include "simulator_app.hpp"
#include "sdfs.hpp"

#include <string>
#include <memory>
#include <thread>

#include <json/json.h>
#include <Eigen/Dense>
#include <GL/glew.h>
#include <imgui.h>


#define TRY_PARSE(errstr, ...) do {                                                                      \
        try { __VA_ARGS__ }                                                                              \
        catch(const Json::LogicError& e) { throw (std::string(errstr) + ": " + std::string(e.what())); } \
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
            throw std::string("No configuration for constraint_solver found");

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
            throw std::string("No nets specified");

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
            throw std::string("No force specified");
        
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
            throw std::string("Invalid type of force");

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
                    throw std::string("Invalid preset for sdf collider");

                solver_->addConstraint(collisionCs_);
            }
            else if(colliderType == "discrete_sdf")
            {
                std::string vdbPath;
                TRY_PARSE("Parsing vdb_path of sdf collider",
                    vdbPath = colliderObj["vdb_path"].asString();
                );

                openvdb::io::File vdbFile(vdbPath);
                try
                {
                    vdbFile.open(false);
                }
                catch(std::exception& e)
                {
                    throw "An error occured while loading \"" + vdbPath + "\": " + std::string(e.what());
                }
                if(vdbFile.getGrids()->empty())
                {
                    throw "An error occured while loading \"" + vdbPath + "\": no grids found";
                }
                openvdb::FloatGrid::Ptr sdfGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(vdbFile.getGrids()->front());
                vdbFile.close();

                collisionCs_ = std::make_shared<DiscreteSDFCollConstr>(nets_, sdfGrid);
                solver_->addConstraint(collisionCs_);
            }
            else
                throw std::string("Invalid type of collider");
        }
    }
    catch(std::string s)
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