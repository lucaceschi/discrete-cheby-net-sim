#include "simulator_app.hpp"

#include <imgui.h>


#define DRAG_FLOAT_SPEED 1e-3


void initGUI()
{
    ImGui::GetIO().Fonts->AddFontFromFileTTF("DroidSans.ttf", 14.0);

    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsLight(&style);
    style.Colors[ImGuiCol_Text] = ImVec4(0.31f, 0.25f, 0.24f, 1.00f);
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 1.00f);
    style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.40f, 0.65f, 0.80f, 0.20f);
    style.Colors[ImGuiCol_Border] = ImVec4(0.68f, 0.68f, 0.68f, 0.60f);
    style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    style.Colors[ImGuiCol_TitleBg] = ImVec4(0.40f, 0.65f, 0.80f, 0.20f);
    style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.40f, 0.65f, 0.80f, 0.20f);
    style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.42f, 0.75f, 1.00f, 0.53f);
    style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.40f, 0.62f, 0.80f, 0.15f);
    style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.39f, 0.64f, 0.80f, 0.30f);
    style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.28f, 0.67f, 0.80f, 0.59f);
    style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.25f, 0.48f, 0.53f, 0.67f);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.62f, 0.70f, 0.72f, 0.56f);
    style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.95f, 0.33f, 0.14f, 0.47f);
    style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.97f, 0.31f, 0.13f, 0.81f);
    style.WindowRounding = 4;
    style.FrameRounding = 1;
    style.FrameBorderSize = 1;
}

void SimulatorApp::drawGUI()
{
    ImGui::ShowDemoWindow();
    
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
        
        ImGui::SeparatorText("Controls");

        static bool playSimGui;
        playSimGui = playSim_;
        if(ImGui::Checkbox("Play sim", &playSimGui))
            playSim_ = playSimGui;

        ImGui::SeparatorText("Scene parameters");
        
        static ConstantForce* forceConstant = dynamic_cast<ConstantForce*>(force_);
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

        static std::shared_ptr<SphereCollConstr> sphereCollision = std::dynamic_pointer_cast<SphereCollConstr>(collisionCs_);
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
            ImGui::Text("Number of iterations: %i", solver_->getNIters());
            ImGui::Text("Final mean delta: %f", solver_->getMeanDelta());
            ImGui::Text("Simple overlay\n" "(right-click to change position)");
            ImGui::Separator();
            static float arr[] = { 0.6f, 0.1f, 1.0f, 0.5f, 0.92f, 0.1f, 0.2f };
            //ImGui::PlotLines("##totDisplacements", solver_.getTotDisplacements(), solver_.getNIters());

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