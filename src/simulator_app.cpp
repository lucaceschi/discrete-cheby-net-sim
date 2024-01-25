#include "simulator_app.hpp"
#include "sdfs.hpp"
#include "framework/debug.hpp"

#include <iostream>
#include <chrono>
#include <vector>
#include <map>

#include <Eigen/Dense>
#include <imgui.h>
#include <GL/glew.h>


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

        glPointSize(5.0f * trackball_.track.sca);

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

        // Draw contact points

        glColor3ub(0xff, 0x80, 0x00);

        glBegin(GL_POINTS);
        for(const ContactConstraint::Contact& c : contactCs_->getContacts())
        {
            Eigen::Vector3f contactPointA, contactPointB;
            std::tie(contactPointA, contactPointB) = c.getContactPoints(nets_);

            glVertex3d(contactPointA(0), contactPointA(1), contactPointA(2));
            glVertex3d(contactPointB(0), contactPointB(1), contactPointB(2));
        }
        glEnd();
    }
}

void SimulatorApp::simulate() {
    const std::chrono::duration TIME_STEP = std::chrono::nanoseconds(1000000000) / solverSpsCap_;

    while(!byebye_) {

        if(!playSim_)
        {
            std::this_thread::yield();
            continue;
        }

        auto startTime = std::chrono::high_resolution_clock::now();
        auto minEndTime = startTime + TIME_STEP;

        if(force_ != nullptr)
        {
            #pragma omp parallel for schedule(static)
            for(int i = 0; i < totNodes_; i++)
            {
                int netIdx = 0;
                int nodeIdx = i;

                while(nodeIdx >= nets_[netIdx]->getNNodes())
                {
                    nodeIdx -= nets_[netIdx]->getNNodes();
                    netIdx++;
                }

                force_->applyForce(nets_, netIdx, nodeIdx);
            }
        }

        if(solver_->solve() == ConstraintSolver::ReturnCode::REACHED_MAX_ITERS)
        {
            frmwrk::Debug::logWarning("Constraint solver reached max iters, stopping");
            playSim_ = false;
        }

        nItersHist_[histNextIndex_]    = solver_->getNIters();
        meanDeltaHist_[histNextIndex_] = solver_->getMeanDelta();
        histNextIndex_ = (histNextIndex_ + 1) % HISTORY_LENGTH;
        

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
    std::vector< std::unordered_map<int, int> > edgeIndexMaps   (nets_.size());
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
                newFixedNodes[netIdx].push_back(newNodes[netIdx].size() - 1);
            }

            newEdges[netIdx].emplace_back(p1Index, p2Index);
            newEdgeLengths[netIdx].push_back(edgeLength);
            edgeIndexMaps[netIdx][edgeIdx] = newEdges[netIdx].size() - 1;
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

    // recreate scissor constraints
    // WARN: exact parametric positions of each scissor are not preserved
    
    std::vector<ContactConstraint::Contact> oldContacts = contactCs_->getContacts();
    contactCs_->clearContacts();
    for(const ContactConstraint::Contact& c : oldContacts)
    {
        std::unordered_map<int, int>::iterator edgeAIt = edgeIndexMaps[c.netIdxA].find(c.edgeIdxA);
        std::unordered_map<int, int>::iterator edgeBIt = edgeIndexMaps[c.netIdxB].find(c.edgeIdxB);

        if(edgeAIt != edgeIndexMaps[c.netIdxA].end() &&
           edgeBIt != edgeIndexMaps[c.netIdxB].end())
        {
            ContactConstraint::Contact newContact(nets_, c.netIdxA, edgeAIt->second,
                                                         c.netIdxB, edgeBIt->second);
            contactCs_->addContact(newContact, nets_,
                                   maxEEDistRel_ * getMinEdgeLength(),
                                   minCNDistRel_ * getMinEdgeLength(),
                                   minCCDistRel_ * getMinEdgeLength());
        }
    }
}

float SimulatorApp::getMinEdgeLength()
{
    float minEdgeLength = std::numeric_limits<float>::max();

    for(int netIdx = 0; netIdx < nets_.size(); netIdx++)
        minEdgeLength = std::min(minEdgeLength, nets_[netIdx]->edgeLengths.maxCoeff());

    return minEdgeLength;
}

int SimulatorApp::addAllContactConstraints()
{
    int nAdded = 0;
    float minEdgeLen = getMinEdgeLength();

    for(int netIdxA = 0; netIdxA < nets_.size()-1; netIdxA++)
        for(int netIdxB = netIdxA+1; netIdxB < nets_.size(); netIdxB++)
            for(int edgeIdxA = 0; edgeIdxA < nets_[netIdxA]->getNEdges(); edgeIdxA++)
                for(int edgeIdxB = 0; edgeIdxB < nets_[netIdxB]->getNEdges(); edgeIdxB++)
                {
                    ContactConstraint::Contact c(nets_, netIdxA, edgeIdxA, netIdxB, edgeIdxB);

                    if(contactCs_->addContact(c, nets_,
                                              maxEEDistRel_ * getMinEdgeLength(),
                                              minCNDistRel_ * getMinEdgeLength(),
                                              minCCDistRel_ * getMinEdgeLength()))
                        nAdded++;
                }
    
    frmwrk::Debug::log("Added %d new contact constraints", nAdded);

    return nAdded;
}