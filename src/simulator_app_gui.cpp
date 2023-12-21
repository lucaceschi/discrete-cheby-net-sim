#include "simulator_app.hpp"

#include <imgui.h>


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
                    for(int n = 0; n < nNets_; n++)
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
        static bool playSimGui = playSim_;
        if(ImGui::Checkbox("Play sim", &playSimGui))
            playSim_ = playSimGui;
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
            ImGui::Text("Final max delta: %f", solver_->getMaxDelta());
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

    {
        ImGui::Begin("Force");
        static ConstantForce* forceConstant = dynamic_cast<ConstantForce*>(force_.get());
        if(forceConstant != NULL)
        {
            ImGui::SeparatorText("Constant force");

            static float vec[3];
            vec[0] = forceConstant->getVector()[0];
            vec[1] = forceConstant->getVector()[1];
            vec[2] = forceConstant->getVector()[2];

            if(ImGui::DragFloat3("Vector", vec, 1e-6, -1, 1, "%.6f"))
                forceConstant->setVector(Eigen::Vector3f(vec));
        }
        ImGui::End();
    }

    trackball_.DrawPostApply();
}