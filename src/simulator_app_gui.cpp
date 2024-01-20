#include "simulator_app.hpp"

#include <imgui.h>


#define DRAG_FLOAT_SPEED 1e-3


void SimulatorApp::initGUI()
{
    ImGui::GetIO().Fonts->AddFontFromFileTTF("../external/imgui/misc/fonts/DroidSans.ttf", 14.0);

    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsLight(&style);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.62f, 0.70f, 0.72f, 0.56f);
    style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.95f, 0.33f, 0.14f, 0.47f);
    style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.97f, 0.31f, 0.13f, 0.81f);
    style.WindowRounding = 4;
    style.FrameRounding = 1;
    style.FrameBorderSize = 1;
}

void SimulatorApp::drawGUI()
{   
    {
        if(ImGui::BeginMainMenuBar())
        {           
            ImGuiID exportPopupId = ImGui::GetID("##export_nets");
            if(ImGui::BeginPopup("##export_nets")) {
                ImGui::SeparatorText("Export nets");
                static char filename[64] = "";
                bool pressedEnter = ImGui::InputTextWithHint("##export_input_text", "base file name", filename, 64, ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                if(pressedEnter || ImGui::Button("Export"))
                {
                    for(int n = 0; n < nets_.size(); n++)
                        nets_[n]->exportPly(std::string(filename) + "_" + std::to_string(n) + ".ply");
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
            
            if(ImGui::BeginMenu("File"))
            {
                if(ImGui::MenuItem("Export nets"))
                    ImGui::OpenPopup(exportPopupId);
                ImGui::EndMenu();
            }

            if(ImGui::BeginMenu("Nets"))
            {
                if(ImGui::MenuItem("Cut and fix nets"))
                    cutNets();
                ImGui::EndMenu();
            }

            if(ImGui::BeginMenu("Camera"))
            {
                ImGui::MenuItem("Ortho proj", nullptr, (bool*)&cameraMode_);
                ImGui::Separator();

                auto setViewpoint = [&](CameraViewpoint newVp)
                {
                    cameraViewpoint_ = newVp;
                    trackball_.Reset();
                };

                if(ImGui::MenuItem("Top viewpoint"))
                    setViewpoint(CameraViewpoint::TOP);
                if(ImGui::MenuItem("Front viewpoint"))
                    setViewpoint(CameraViewpoint::FRONT);
                if(ImGui::MenuItem("Right viewpoint"))
                    setViewpoint(CameraViewpoint::RIGHT);
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }
    }
    
    {
        ImGui::Begin("Simulation");
        
        ImGui::SeparatorText("Constraint solver params");

        ImGui::DragFloat("Abs tol", &solver_->absoluteTol, 1e-2, 0.1, 1e-18, "%.2e");
        ImGui::DragFloat("Rel tol", &solver_->relativeTol, 1e-2, 0.1, 1e-18, "%.2e");
        ImGui::DragInt("Max iters", &solver_->maxIters, 5, 1, INT_MAX);

        ImGui::SeparatorText("Scene params");
        
        static ConstantForce* forceConstant = dynamic_cast<ConstantForce*>(force_);
        static DiscreteSDFFittingForce* fittingForce = dynamic_cast<DiscreteSDFFittingForce*>(force_);
        if(forceConstant)
        {
            if(ImGui::CollapsingHeader("Constant force"))
            {
                static float vec[3] = {
                    forceConstant->vec[0],
                    forceConstant->vec[1],
                    forceConstant->vec[2]
                };

                if(ImGui::DragFloat3("Vector", vec, DRAG_FLOAT_SPEED, -1, 1, "%.6f"))
                    forceConstant->vec = Eigen::Vector3f(vec);
            }
        }
        else if(fittingForce)
        {
            if(ImGui::CollapsingHeader("Fitting force"))
            {
                static float worldTranslationVec[3] = {
                    fittingForce->getWorldTranslationVec()[0],
                    fittingForce->getWorldTranslationVec()[1],
                    fittingForce->getWorldTranslationVec()[2]
                };

                if(ImGui::DragFloat3("Vector", worldTranslationVec, DRAG_FLOAT_SPEED, -1, 1, "%.6f"))
                    fittingForce->setWorldTranslationVec(Eigen::Vector3f(worldTranslationVec));

                static float nearBound = fittingForce->getNearBound();
                static float farBound = fittingForce->getFarBound();
                if(ImGui::DragFloatRange2("Smoothstep bounds", &nearBound, &farBound, DRAG_FLOAT_SPEED,
                                          0, fittingForce->getMaxFarBound(), "%.6f"))
                {
                    fittingForce->setNearBound(nearBound);
                    fittingForce->setFarBound(farBound);
                }
            }
        }

        static std::shared_ptr<SphereCollConstr> sphereCollision = std::dynamic_pointer_cast<SphereCollConstr>(collisionCs_);
        static std::shared_ptr<PlanarBoundaryConstr> planarCollision = std::dynamic_pointer_cast<PlanarBoundaryConstr>(collisionCs_);
        if(sphereCollision)
        {
            if(ImGui::CollapsingHeader("Sphere collision"))
            {
                ImGui::Checkbox("Active", &sphereCollision->active);
                ImGui::DragFloat("Radius", &sphereCollision->radius, DRAG_FLOAT_SPEED);

                static float origin[3] = {
                    sphereCollision->centerPos[0],
                    sphereCollision->centerPos[1],
                    sphereCollision->centerPos[2]
                };
                if(ImGui::DragFloat3("Origin", origin, DRAG_FLOAT_SPEED))
                    sphereCollision->centerPos = Eigen::Vector3f(origin);
            }
        }
        else if(planarCollision)
        {
            if(ImGui::CollapsingHeader("Planar boundary collision"))
            {
                ImGui::Checkbox("Active", &planarCollision->active);

                static float point[3] = {
                    planarCollision->pointOnBoundary[0],
                    planarCollision->pointOnBoundary[1],
                    planarCollision->pointOnBoundary[2]
                };
                if(ImGui::DragFloat3("Point", point, DRAG_FLOAT_SPEED))
                    planarCollision->pointOnBoundary = Eigen::Vector3f(point);

                static float normal[3] = {
                    planarCollision->normalVec[0],
                    planarCollision->normalVec[1],
                    planarCollision->normalVec[2]
                };
                if(ImGui::DragFloat3("Normal", normal, DRAG_FLOAT_SPEED))
                    planarCollision->normalVec = Eigen::Vector3f(normal);
            }
        }

        if(ImGui::CollapsingHeader("Shearing limit"))
        {
            static float minRadians = shearLimitCs_->getLimit();

            if(ImGui::DragFloat("Min radians", &minRadians, 0.05, 0, M_PI_2))
                shearLimitCs_->setLimit(minRadians);
        }

        ImGui::End();
    }

    {
        static int location = 2;
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration |
                                        ImGuiWindowFlags_AlwaysAutoResize |
                                        ImGuiWindowFlags_NoSavedSettings |
                                        ImGuiWindowFlags_NoFocusOnAppearing |
                                        ImGuiWindowFlags_NoNav;
        if (location >= 0)
        {
            const float PAD = 10.0f;
            const ImGuiViewport* viewport = ImGui::GetMainViewport();
            ImVec2 work_pos = viewport->WorkPos;
            ImVec2 work_size = viewport->WorkSize;
            ImVec2 window_pos, window_pos_pivot;
            window_pos.x = (location & 1) ? (work_pos.x + work_size.x - PAD) : (work_pos.x + PAD);
            window_pos.y = (location & 2) ? (work_pos.y + work_size.y - PAD) : (work_pos.y + PAD);
            window_pos_pivot.x = (location & 1) ? 1.0f : 0.0f;
            window_pos_pivot.y = (location & 2) ? 1.0f : 0.0f;
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
            window_flags |= ImGuiWindowFlags_NoMove;
        }
        else if (location == -2)
        {
            ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
            window_flags |= ImGuiWindowFlags_NoMove;
        }
        ImGui::SetNextWindowBgAlpha(0.35f);
        if (ImGui::Begin("##Stats_overlay", nullptr, window_flags))
        {    
            static bool playSimGui;
            playSimGui = playSim_;
            if(ImGui::Checkbox("Play sim", &playSimGui))
                playSim_ = playSimGui;
            
            ImGui::SeparatorText("Constraint solver stats");
            ImGui::PlotLines("##nItersPlot", nItersHist_.data(),
                             HISTORY_LENGTH, histNextIndex_,
                             "N iters", 0, FLT_MAX, {200, 30});
            ImGui::PlotLines("##meanDeltaPlot", meanDeltaHist_.data(),
                             HISTORY_LENGTH, histNextIndex_,
                             "Mean delta", 0, FLT_MAX, {200, 30});

            if (ImGui::BeginPopupContextWindow())
            {
                if (ImGui::MenuItem("Custom",       NULL, location == -1)) location = -1;
                if (ImGui::MenuItem("Center",       NULL, location == -2)) location = -2;
                if (ImGui::MenuItem("Top-left",     NULL, location == 0)) location = 0;
                if (ImGui::MenuItem("Top-right",    NULL, location == 1)) location = 1;
                if (ImGui::MenuItem("Bottom-left",  NULL, location == 2)) location = 2;
                if (ImGui::MenuItem("Bottom-right", NULL, location == 3)) location = 3;
                ImGui::EndPopup();
            }
        }
        ImGui::End();
    }

    trackball_.DrawPostApply();
}