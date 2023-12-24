#include "simulator_app.hpp"

#include <tuple>

#include <Eigen/Dense>
#include <imgui.h>
#include <GL/glew.h>


const GLclampf SimulatorApp::PICKING_BG_COLOR[3] = {1, 1, 1};

std::tuple<GLubyte, GLubyte, GLubyte> node2color(int i)
{
    return {(GLubyte)(i), (GLubyte)(i >> 8), (GLubyte)(i >> 16)};
}

int color2node(GLubyte r, GLubyte g, GLubyte b)
{
    if(r == 0xff && g == 0xff && b == 0xff)
        return -1;
    
    int nodeIdx = 0;

    nodeIdx = (nodeIdx + b) << 8;
    nodeIdx = (nodeIdx + g) << 8;
    nodeIdx = (nodeIdx + r);

    return nodeIdx;
}

void SimulatorApp::drawNetPickingMode(int netIdx)
{
    Net& net = *nets_[netIdx];

    glClearColor(PICKING_BG_COLOR[0], PICKING_BG_COLOR[1], PICKING_BG_COLOR[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw grid nodes

    glPointSize(24.0f * trackball_.track.sca);

    glBegin(GL_POINTS);
    for(int nodeIdx = 0; nodeIdx < net.getNNodes(); nodeIdx++)
    {
        GLubyte r, g, b;
        std::tie(r, g, b) = node2color(nodeIdx);
        glColor3ub(r, g, b);

        const Eigen::Vector3f nodePos = net.nodePos(nodeIdx);
        glVertex3d(nodePos(0), nodePos(1), nodePos(2));
    }
    glEnd();
}

void SimulatorApp::handleMouseEvents()
{
    if(ImGui::GetIO().WantCaptureMouse)
        return;

    Eigen::Vector2d curPos = input_.getCursorPos();
    curPos(0) = curPos(0) * getWindowContentScale()(0);
    curPos(1) = getFramebufferSize()(1) - curPos(1) * getWindowContentScale()(1);

    if(input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT)
       || input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT))
    {
        GLubyte pickedColor[3];
        GLfloat pickedDepth;
        int pickedNodeIdx;

        for(int netIdx = 0; netIdx < nets_.size(); netIdx++)
        {
            drawNetPickingMode(netIdx);
            glReadPixels(curPos(0), curPos(1), 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &pickedColor);
            glReadPixels(curPos(0), curPos(1), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &pickedDepth);
            pickedNodeIdx = color2node(pickedColor[0], pickedColor[1], pickedColor[2]);

            if(pickedNodeIdx != -1 && input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT))
            {
                if(!fixedCs_->isNodeFixed(netIdx, pickedNodeIdx))
                    fixedCs_->fixNode(nets_, netIdx, pickedNodeIdx);
                else
                    fixedCs_->freeNode(netIdx, pickedNodeIdx);
                break;
            }
            else if(pickedNodeIdx != -1 && input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                pick_ = Pick(netIdx, pickedNodeIdx, pickedDepth);
                fixedCs_->fixNode(nets_, netIdx, pickedNodeIdx);
                break;
            }
        }

        if(pickedNodeIdx == -1)
            trackball_.MouseDown((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
    }

    if(input_.isMouseButtonHeld(GLFW_MOUSE_BUTTON_LEFT))
    {
        if(pick_.pick)
        {
            GLdouble modelview[16];
            GLdouble projection[16];
            glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
            glGetDoublev(GL_PROJECTION_MATRIX, projection);
            Eigen::Map<Eigen::Matrix4d> modelviewMat(modelview);
            Eigen::Map<Eigen::Matrix4d> projectionMat(projection);

            Eigen::Vector2d curPos = input_.getCursorPos();
            Eigen::Vector4d curPosHom = {
                ((curPos(0) / getFramebufferSize()(0)) - 0.5)*2,
                -((curPos(1) / getFramebufferSize()(1)) - 0.5)*2,
                pick_.depth * 2.0 - 1.0,
                1
            };

            Eigen::Vector4f worldPos = ((projectionMat * modelviewMat).inverse() * curPosHom).cast<float>();

            nets_[pick_.netIdx]->nodePos(pick_.nodeIdx) = Eigen::Vector3f{
                worldPos(0) / worldPos(3),
                worldPos(1) / worldPos(3),
                worldPos(2) / worldPos(3)
            };
            fixedCs_->fixNode(nets_, pick_.netIdx, pick_.nodeIdx);
        }
        else
            trackball_.MouseMove((int)curPos(0), (int)curPos(1));
    }

    if(input_.isMouseButtonReleased(GLFW_MOUSE_BUTTON_LEFT))
    {
        if(pick_.pick)
            fixedCs_->freeNode(pick_.netIdx, pick_.nodeIdx);
        pick_ = Pick();
        trackball_.MouseUp((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
    }

    if(input_.getMouseScrollOffset() != 0)
        trackball_.MouseWheel(input_.getMouseScrollOffset());
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