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

#define JSON_KEY_SIM_PARAMS             std::string("simulator_params")
#define JSON_KEY_SIM_PARAMS_MAX_SPS     std::string("max_steps_per_second")
#define JSON_KEY_SIM_PARAMS_ABS_TOL     std::string("absolute_tolerance")
#define JSON_KEY_SIM_PARAMS_REL_TOL     std::string("relative_tolerance")
#define JSON_KEY_SIM_PARAMS_MAX_IPS     std::string("max_iters_per_step")

#define JSON_KEY_NETS_LIST          std::string("nets")
#define JSON_KEY_NET_SIZE           std::string("size")
#define JSON_KEY_NET_CENTER         std::string("center")
#define JSON_KEY_NET_X_TAN_VEC      std::string("x_tangent_vector")
#define JSON_KEY_NET_Y_TAN_VEC      std::string("y_tangent_vector")
#define JSON_KEY_NET_EDGE_LEN       std::string("edge_length")
#define JSON_KEY_NET_SHEAR_LIM      std::string("shear_angle_limit")
#define JSON_KEY_COLOR              std::string("color")

#define JSON_KEY_COLLIDER                       std::string("collider")
#define JSON_VAL_COLLIDER_PLANE_TYPE            std::string("planar_boundary")
#define JSON_KEY_COLLIDER_PLANE_POINT           std::string("point")
#define JSON_KEY_COLLIDER_PLANE_NORMAL          std::string("normal")
#define JSON_VAL_COLLIDER_SPHERE_TYPE           std::string("sphere")
#define JSON_KEY_COLLIDER_SPHERE_ORIGIN         std::string("origin")
#define JSON_KEY_COLLIDER_SPHERE_RADIUS         std::string("radius")
#define JSON_VAL_COLLIDER_SDF_TYPE              std::string("sdf")
#define JSON_KEY_COLLIDER_SDF_PRESET            std::string("preset")
#define JSON_VAL_COLLIDER_SDF_PRESET_TORUS      std::string("torus")
#define JSON_VAL_COLLIDER_SDF_PRESET_PEANUT     std::string("peanut")
#define JSON_VAL_COLLIDER_DISCRETE_SDF_TPYE     std::string("discrete_sdf")
#define JSON_KEY_COLLIDER_SDF_VDB_PATH          std::string("vdb_path")

#define JSON_KEY_FORCE                                   std::string("force")
#define JSON_KEY_FORCE_TYPE                              std::string("type")
#define JSON_VAL_FORCE_CONSTANT_TYPE                     std::string("constant")
#define JSON_KEY_FORCE_CONSTANT_TRANSLATION_VEC          std::string("translation_vector")
#define JSON_VAL_FORCE_FITTING_TYPE                      std::string("discrete_sdf_fitting")
#define JSON_KEY_FORCE_ATTARCTION_VDB_PATH               std::string("vdb_path")
#define JSON_KEY_FORCE_FITTING_SMOOTHSTEP_NEAR_BOUND     std::string("smoothstep_near_bound")
#define JSON_KEY_FORCE_FITTING_SMOOTHSTEP_FAR_BOUND      std::string("smoothstep_far_bound")
#define JSON_KEY_FORCE_FITTING_WORKD_TRANSLATIION_VEC    std::string("world_translation_vec")


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
      solver_(nullptr),
      playSim_(false),
      nItersHist_(),
      meanDeltaHist_(),
      histNextIndex_(0),
      simThread_(),
      byebye_(false),
      pick_()
{}

bool SimulatorApp::initApp()
{    
    try
    {
        // initialize constraint solver

        Json::Value& simParamObj = sceneConfig_[JSON_KEY_SIM_PARAMS];
        if(simParamObj.isNull() || !simParamObj.isObject())
            throw std::string("No configuration for" + JSON_KEY_SIM_PARAMS + "found");

        float absTol;
        TRY_PARSE("Parsing " + JSON_KEY_SIM_PARAMS_ABS_TOL + " of constraint solver",
            absTol = simParamObj[JSON_KEY_SIM_PARAMS_ABS_TOL].asFloat();
        );

        float relTol;
        TRY_PARSE("Parsing " + JSON_KEY_SIM_PARAMS_REL_TOL + " of constraint solver",
            relTol = simParamObj[JSON_KEY_SIM_PARAMS_REL_TOL].asFloat();
        );

        int maxIters;
        TRY_PARSE("Parsing " + JSON_KEY_SIM_PARAMS_MAX_IPS + " of constraint solver",
            maxIters = simParamObj[JSON_KEY_SIM_PARAMS_MAX_IPS].asInt();
        );

        TRY_PARSE("Parsing " + JSON_KEY_SIM_PARAMS_MAX_SPS + " of constraint solver",
            solverSpsCap_ = simParamObj[JSON_KEY_SIM_PARAMS_MAX_SPS].asInt();
        );

        solver_ = new ConstraintSolver(nets_, absTol, relTol, maxIters);
        
        // initialize nets
        
        Json::Value& netsArray = sceneConfig_[JSON_KEY_NETS_LIST];
        if(netsArray.isNull() || !netsArray.isArray() || netsArray.size() == 0)
            throw std::string("No nets specified");

        int nNets = netsArray.size();
        nets_.reserve(nNets);
        edgeLenCs_.reserve(nNets);
        shearLimitCs_.reserve(nNets);
        for(int n = 0; n < nNets; n++)
        {
            Eigen::Vector2i size;
            TRY_PARSE("Parsing " + JSON_KEY_NET_SIZE + " of net " + std::to_string(n), 
                size = Eigen::Vector2i(netsArray[n][JSON_KEY_NET_SIZE][0].asInt(),
                                       netsArray[n][JSON_KEY_NET_SIZE][1].asInt());
            );

            Eigen::Vector3f center;
            TRY_PARSE("Parsing " + JSON_KEY_NET_CENTER + " of net " + std::to_string(n),
                center = Eigen::Vector3f(netsArray[n][JSON_KEY_NET_CENTER][0].asFloat(),
                                         netsArray[n][JSON_KEY_NET_CENTER][1].asFloat(),
                                         netsArray[n][JSON_KEY_NET_CENTER][2].asFloat());
            );

            Eigen::Vector3f xTangVec;
            TRY_PARSE("Parsing " + JSON_KEY_NET_X_TAN_VEC + " of net " + std::to_string(n),
                xTangVec = Eigen::Vector3f(netsArray[n][JSON_KEY_NET_X_TAN_VEC][0].asFloat(),
                                           netsArray[n][JSON_KEY_NET_X_TAN_VEC][1].asFloat(),
                                           netsArray[n][JSON_KEY_NET_X_TAN_VEC][2].asFloat());
            );

            Eigen::Vector3f yTangVec;
            TRY_PARSE("Parsing " + JSON_KEY_NET_Y_TAN_VEC + " of net " + std::to_string(n),
                yTangVec = Eigen::Vector3f(netsArray[n][JSON_KEY_NET_Y_TAN_VEC][0].asFloat(),
                                           netsArray[n][JSON_KEY_NET_Y_TAN_VEC][1].asFloat(),
                                           netsArray[n][JSON_KEY_NET_Y_TAN_VEC][2].asFloat());
            );

            float edgeLength;
            TRY_PARSE("Parsing " + JSON_KEY_NET_EDGE_LEN + " of net " + std::to_string(n),
                edgeLength = netsArray[n][JSON_KEY_NET_EDGE_LEN].asFloat();
            );

            float shearLimit;
            TRY_PARSE("Parsing " + JSON_KEY_NET_SHEAR_LIM + " of net " + std::to_string(n),
                shearLimit = netsArray[n][JSON_KEY_NET_SHEAR_LIM].asFloat();
            );

            GLubyte color[3];
            TRY_PARSE("Parsing " + JSON_KEY_COLOR + " of net " + std::to_string(n),
                for(int i = 0; i < 3; i++)
                {
                    unsigned int c = netsArray[n][JSON_KEY_COLOR][i].asUInt();
                    color[i] = (GLubyte)((c < 255)? c : 255);
                }
            );

            nets_.push_back(new Net(size, edgeLength, center, xTangVec, yTangVec, color));
            totNodes_ += nets_[n]->getNNodes();

            edgeLenCs_.push_back(std::make_shared<EdgeLengthConstraint>(nets_, n));
            solver_->addConstraint(edgeLenCs_.back());

            shearLimitCs_.push_back(std::make_shared<ShearLimitConstr>(n, edgeLength, shearLimit));
            solver_->addConstraint(shearLimitCs_.back());
        }

        fixedCs_ = std::make_shared<FixedNodeConstraint>(nets_);
        solver_->addConstraint(fixedCs_);

        // initialize force        

        Json::Value& forceObj = sceneConfig_[JSON_KEY_FORCE];
        if(!forceObj.isNull() && forceObj.isObject())
        {
            std::string forceType;
            TRY_PARSE("Parsing " + JSON_KEY_FORCE_TYPE + " of force",
                forceType = forceObj[JSON_KEY_FORCE_TYPE].asString();
            );

            if(forceType == JSON_VAL_FORCE_CONSTANT_TYPE)
            {
                Eigen::Vector3f translationVec;
                TRY_PARSE("Parsing " + JSON_KEY_FORCE_CONSTANT_TRANSLATION_VEC + " of " + JSON_VAL_FORCE_CONSTANT_TYPE + " force",
                    translationVec = Eigen::Vector3f(forceObj[JSON_KEY_FORCE_CONSTANT_TRANSLATION_VEC][0].asFloat(),
                                                    forceObj[JSON_KEY_FORCE_CONSTANT_TRANSLATION_VEC][1].asFloat(),
                                                    forceObj[JSON_KEY_FORCE_CONSTANT_TRANSLATION_VEC][2].asFloat());
                );

                force_ = new ConstantForce(translationVec);
            }
            else if(forceType == JSON_VAL_FORCE_FITTING_TYPE)
            {
                std::string vdbPath;
                TRY_PARSE("Parsing " + JSON_KEY_FORCE_ATTARCTION_VDB_PATH + " of " + JSON_VAL_FORCE_FITTING_TYPE + " force",
                    vdbPath = forceObj["vdb_path"].asString();
                );

                float nearBound;
                TRY_PARSE("Parsing " + JSON_KEY_FORCE_FITTING_SMOOTHSTEP_NEAR_BOUND + " of " + JSON_VAL_FORCE_FITTING_TYPE + " force",
                    nearBound = forceObj[JSON_KEY_FORCE_FITTING_SMOOTHSTEP_NEAR_BOUND].asFloat();
                );

                float farBound;
                TRY_PARSE("Parsing " + JSON_KEY_FORCE_FITTING_SMOOTHSTEP_FAR_BOUND + " of " + JSON_VAL_FORCE_FITTING_TYPE + " force",
                    farBound = forceObj[JSON_KEY_FORCE_FITTING_SMOOTHSTEP_FAR_BOUND].asFloat();
                );

                Eigen::Vector3f worldTranslationVec;
                TRY_PARSE("Parsing " + JSON_KEY_FORCE_FITTING_WORKD_TRANSLATIION_VEC + " of " + JSON_VAL_FORCE_FITTING_TYPE + " force",
                    worldTranslationVec = Eigen::Vector3f(forceObj[JSON_KEY_FORCE_FITTING_WORKD_TRANSLATIION_VEC][0].asFloat(),
                                                        forceObj[JSON_KEY_FORCE_FITTING_WORKD_TRANSLATIION_VEC][1].asFloat(),
                                                        forceObj[JSON_KEY_FORCE_FITTING_WORKD_TRANSLATIION_VEC][2].asFloat());
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
            
                force_ = new DiscreteSDFFittingForce(nets_, sdfGrid, nearBound, farBound, worldTranslationVec);
            }
            else
                throw std::string("Invalid type of force");
        }

        // initialize collider

        Json::Value& colliderObj = sceneConfig_[JSON_KEY_COLLIDER];
        if(!colliderObj.isNull() && colliderObj.isObject())
        {
            std::string colliderType;
            TRY_PARSE("Parsing " + JSON_KEY_FORCE_TYPE + " of collider",
                colliderType = colliderObj[JSON_KEY_FORCE_TYPE].asString();
            );

            if(colliderType == JSON_VAL_COLLIDER_SPHERE_TYPE)
            {
                Eigen::Vector3f origin;
                TRY_PARSE("Parsing " + JSON_KEY_COLLIDER_SPHERE_ORIGIN + " of " + JSON_VAL_COLLIDER_SPHERE_TYPE + " collider",
                    origin = Eigen::Vector3f(colliderObj[JSON_KEY_COLLIDER_SPHERE_ORIGIN][0].asFloat(),
                                             colliderObj[JSON_KEY_COLLIDER_SPHERE_ORIGIN][1].asFloat(),
                                             colliderObj[JSON_KEY_COLLIDER_SPHERE_ORIGIN][2].asFloat());
                );

                float radius;
                TRY_PARSE("Parsing " + JSON_KEY_COLLIDER_SPHERE_RADIUS + " of " + JSON_VAL_COLLIDER_SPHERE_TYPE + " collider",
                    radius = colliderObj[JSON_KEY_COLLIDER_SPHERE_RADIUS].asFloat();
                );

                collisionCs_ = std::make_shared<SphereCollConstr>(origin, radius);
                solver_->addConstraint(collisionCs_);
            }
            else if(colliderType == JSON_VAL_COLLIDER_PLANE_TYPE)
            {
                Eigen::Vector3f point;
                TRY_PARSE("Parsing " + JSON_KEY_COLLIDER_PLANE_POINT + " of " + JSON_VAL_COLLIDER_PLANE_TYPE + " collider",
                    point = Eigen::Vector3f(colliderObj[JSON_KEY_COLLIDER_PLANE_POINT][0].asFloat(),
                                            colliderObj[JSON_KEY_COLLIDER_PLANE_POINT][1].asFloat(),
                                            colliderObj[JSON_KEY_COLLIDER_PLANE_POINT][2].asFloat());
                );

                Eigen::Vector3f normal;
                TRY_PARSE("Parsing " + JSON_KEY_COLLIDER_PLANE_NORMAL + " of " + JSON_VAL_COLLIDER_PLANE_TYPE + " collider",
                    normal = Eigen::Vector3f(colliderObj[JSON_KEY_COLLIDER_PLANE_NORMAL][0].asFloat(),
                                             colliderObj[JSON_KEY_COLLIDER_PLANE_NORMAL][1].asFloat(),
                                             colliderObj[JSON_KEY_COLLIDER_PLANE_NORMAL][2].asFloat());
                );

                collisionCs_ = std::make_shared<PlanarBoundaryConstr>(point, normal);
                solver_->addConstraint(collisionCs_);
            }
            else if(colliderType == JSON_VAL_COLLIDER_SDF_TYPE)
            {
                using SDFCollConstrF = SDFCollConstr<float(Eigen::Vector3f), Eigen::Vector3f(Eigen::Vector3f)>;

                std::string preset;
                TRY_PARSE("Parsing " + JSON_KEY_COLLIDER_SDF_PRESET + " of " + JSON_VAL_COLLIDER_SDF_TYPE + " collider",
                    preset = colliderObj[JSON_KEY_COLLIDER_SDF_PRESET].asString();
                );

                if(preset == JSON_VAL_COLLIDER_SDF_PRESET_PEANUT)
                    collisionCs_ = std::make_shared<SDFCollConstrF>(SDFs::Peanut::sdf, SDFs::Peanut::dsdf, false);
                else if(preset == JSON_VAL_COLLIDER_SDF_PRESET_TORUS)
                    collisionCs_ = std::make_shared<SDFCollConstrF>(SDFs::Torus::sdf, SDFs::Torus::dsdf, true);
                else
                    throw std::string("Invalid " + JSON_KEY_COLLIDER_SDF_PRESET + " for " + JSON_VAL_COLLIDER_SDF_TYPE + " collider");

                solver_->addConstraint(collisionCs_);
            }
            else if(colliderType == JSON_VAL_COLLIDER_DISCRETE_SDF_TPYE)
            {
                std::string vdbPath;
                TRY_PARSE("Parsing " + JSON_KEY_COLLIDER_SDF_VDB_PATH + " of " + JSON_VAL_COLLIDER_DISCRETE_SDF_TPYE + " collider",
                    vdbPath = colliderObj[JSON_KEY_COLLIDER_SDF_VDB_PATH].asString();
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