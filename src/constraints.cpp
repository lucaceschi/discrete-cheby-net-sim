#include "constraints.hpp"

#include <iostream>


ConstraintTask::ConstraintTask()
    : active(true)
{}

float ConstraintTask::solve(std::vector<Net*>& nets) const
{
    return (active)? solve_(nets) : 0;
}


FixedNodeConstraint::FixedNodeConstraint(std::vector<Net*>& nets)
    : fixedPos_(nets.size())
{}

void FixedNodeConstraint::fixNode(std::vector<Net*>& nets, int netIndex, int nodeIndex)
{
    fixedPos_[netIndex][nodeIndex] = nets[netIndex]->nodePos(nodeIndex);
}

bool FixedNodeConstraint::isNodeFixed(int netIndex, int nodeIndex) const
{
    return fixedPos_[netIndex].find(nodeIndex) != fixedPos_[netIndex].cend();
}

void FixedNodeConstraint::freeNode(int netIndex, int nodeIndex)
{
    typedef std::unordered_map<int, Eigen::Vector3f>::iterator Iterator;
    Iterator it = fixedPos_[netIndex].find(nodeIndex);

    if(it != fixedPos_[netIndex].end())
        fixedPos_[netIndex].erase(it);
}

void FixedNodeConstraint::freeNodes(int netIndex)
{
    fixedPos_[netIndex].clear();
}

float FixedNodeConstraint::solve_(std::vector<Net*>& nets) const
{
    typedef std::unordered_map<int, Eigen::Vector3f>::const_iterator CIterator;
    float meanDelta;

    meanDelta = 0;
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
        for(CIterator it = fixedPos_[netIdx].cbegin(); it != fixedPos_[netIdx].cend(); it++)
        {
            meanDelta += (nets[netIdx]->nodePos(it->first) - it->second).squaredNorm();
            nets[netIdx]->nodePos(it->first) = it->second;
        }

    return meanDelta;
}

int FixedNodeConstraint::nConstraints(std::vector<Net*>& nets) const
{
    int n = 0;
    for(const auto& m : fixedPos_)
        n += m.size();

    return n;
}


EdgeLengthConstraint::EdgeLengthConstraint(std::vector<Net*>& nets, int netIndex)
    : netIdx_(netIndex)
{
    updateEdgeLengths(nets);
}

void EdgeLengthConstraint::updateEdgeLengths(std::vector<Net*>& nets)
{
    edgeLensSquared_ = Eigen::ArrayXf(nets[netIdx_]->edgeLengths.square());
}

float EdgeLengthConstraint::solve_(std::vector<Net*>& nets) const
{
    Net* n = nets[netIdx_];
    float meanDelta;

    meanDelta = 0;
    for(int e = 0; e < n->getNEdges(); e++)
    {
        Eigen::Vector3f v = n->nodePos(n->edge(e)[0]) - n->nodePos(n->edge(e)[1]);
        float dist = v.squaredNorm();
        float delta = (edgeLensSquared_[e] - dist);
        float s = delta / (4 * dist);
        Eigen::Vector3f deltaV = s * v;

        n->nodePos(n->edge(e)[0]) += deltaV;
        n->nodePos(n->edge(e)[1]) -= deltaV;

        meanDelta += std::abs(delta);
    }

    return meanDelta;
}

int EdgeLengthConstraint::nConstraints(std::vector<Net*>& nets) const
{
    return nets[netIdx_]->getNEdges();
}


ShearLimitConstr::ShearLimitConstr(int netIdx, float edgeLength, float minRadians)
    : len_(edgeLength),
      netIdx_(netIdx)
{
    setLimit(minRadians);
}

float ShearLimitConstr::solve_(std::vector<Net*>& nets) const
{
    Net* n = nets[netIdx_];
    float meanDelta;

    meanDelta = 0;    
    for(int d = 0; d < n->getNDiagonals(); d++)
    {
        Eigen::Vector3f v = n->nodePos(n->diag(d)[0]) - n->nodePos(n->diag(d)[1]);
        float dist = v.squaredNorm();

        if(dist > squaredLen_)
            continue;

        float delta = (squaredLen_ - dist);
        float s = delta / (4 * dist);
        Eigen::Vector3f deltaV = s * v;

        n->nodePos(n->diag(d)[0]) += deltaV;
        n->nodePos(n->diag(d)[1]) -= deltaV;

        meanDelta += std::abs(delta);
    }

    return meanDelta;
}

int ShearLimitConstr::nConstraints(std::vector<Net*>& nets) const
{
    return nets[netIdx_]->getNDiagonals();
}

float ShearLimitConstr::getLimit()
{
    return minRadians_;
}

void ShearLimitConstr::setLimit(float minRadians)
{
    minRadians_ = minRadians;
    
    if(minRadians <= 0)
    {
        squaredLen_ = 0;
        minRadians_ = 0;
    }
    else if(minRadians >= M_PI_2)
    {
        squaredLen_ = std::pow(len_, 2) * M_SQRT2;
        minRadians_ = M_PI_2;
    }
    else
    {
        float twoSqEdgeLen = 2 * std::pow(len_, 2);
        squaredLen_ = twoSqEdgeLen - twoSqEdgeLen * std::cos(minRadians);
    }
}


PlanarBoundaryConstr::PlanarBoundaryConstr(Eigen::Vector3f pointOnBoundary, Eigen::Vector3f normalVec)
    : pointOnBoundary(pointOnBoundary),
      normalVec(normalVec)
{}

int PlanarBoundaryConstr::nConstraints(std::vector<Net*>& nets) const
{
    int n = 0;
    for(const Net* net : nets)
        n += net->getNNodes();

    return n; 
}

float PlanarBoundaryConstr::solve_(std::vector<Net*>& nets) const
{
    float currDelta, meanDelta;

    meanDelta = 0;
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
    {
        Net* n = nets[netIdx];

        for(int nodeIdx = 0; nodeIdx < n->getNNodes(); nodeIdx++)
        {
            currDelta = (pointOnBoundary - n->nodePos(nodeIdx)).dot(normalVec);
            
            if(currDelta > 0)
            {
                meanDelta += std::pow(currDelta, 2);
                n->nodePos(nodeIdx) += (currDelta * normalVec);
            }
        }
    }

    return meanDelta;
}


SphereCollConstr::SphereCollConstr(Eigen::Vector3f centerPos, float radius)
    : centerPos(centerPos),
      radius(radius)
{}

float SphereCollConstr::solve_(std::vector<Net*>& nets) const
{
    float currDist, currDelta, meanDelta;

    meanDelta = 0;
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
    {
        Net* n = nets[netIdx];

        for(int i = 0; i < n->getNNodes(); i++)
        {            
            currDist = (n->nodePos(i) - centerPos).norm();
            currDelta = radius - currDist; 
            if(currDelta > 0)
            {
                meanDelta += std::pow(currDelta, 2);
                
                if((n->nodePos(i) - centerPos).isZero())
                    n->nodePos(i) += Eigen::Vector3f{0, radius, 0};
                else
                    n->nodePos(i) = (n->nodePos(i) - centerPos) * (radius / currDist) + centerPos;
            }
        }
    }

    return meanDelta;
}

int SphereCollConstr::nConstraints(std::vector<Net*>& nets) const
{
    int n = 0;
    for(const Net* net : nets)
        n += net->getNNodes();

    return n;
}