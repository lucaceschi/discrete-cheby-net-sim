#include "constraints.hpp"

#include <iostream>


EdgeLengthConstraint::EdgeLengthConstraint(int netIndex, float edgeLength)
    : netIdx_(netIndex),
      edgeLenSquared_(std::pow(edgeLength, 2))
{}

float EdgeLengthConstraint::solve(std::vector<Net*>& nets) const
{
    Net* n = nets[netIdx_];
    float meanDelta;

    meanDelta = 0;
    for(int e = 0; e < n->getNEdges(); e++)
    {
        Eigen::Vector3f v = n->nodePos(n->edge(e)[0]) - n->nodePos(n->edge(e)[1]);
        float dist = v.squaredNorm();
        float delta = (edgeLenSquared_ - dist);
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
    : netIdx_(netIdx)
{
    setLimit(edgeLength, minRadians);
}

float ShearLimitConstr::solve(std::vector<Net*>& nets) const
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

void ShearLimitConstr::setLimit(float edgeLength, float minRadians)
{
    if(minRadians <= 0)
        squaredLen_ = 0;
    else if(minRadians >= M_PI_2)
        squaredLen_ = std::pow(edgeLength, 2) * M_SQRT2;
    else
    {
        float twoSqEdgeLen = 2 * std::pow(edgeLength, 2);
        squaredLen_ = twoSqEdgeLen - twoSqEdgeLen * std::cos(minRadians);
    }
}


SphereCollConstr::SphereCollConstr(int netIdx, Eigen::Vector3f centerPos, float radius)
    : netIdx_(netIdx),
        centerPos_(centerPos),
        radius_(radius)
{}

float SphereCollConstr::solve(std::vector<Net*>& nets) const
{
    Net* n = nets[netIdx_];
    float currDist, currDelta, meanDelta;

    meanDelta = 0;
    for(int i = 0; i < n->getNNodes(); i++)
    {            
        currDist = (n->nodePos(i) - centerPos_).norm();
        currDelta = radius_ - currDist; 
        if(currDelta > 0)
        {
            meanDelta += std::pow(currDelta, 2);
            
            if((n->nodePos(i) - centerPos_).isZero())
                n->nodePos(i) += Eigen::Vector3f{0, radius_, 0};
            else
                n->nodePos(i) = (n->nodePos(i) - centerPos_) * (radius_ / currDist) + centerPos_;
        }
    }

    return meanDelta;
}

int SphereCollConstr::nConstraints(std::vector<Net*>& nets) const
{
    return nets[netIdx_]->getNNodes();
}