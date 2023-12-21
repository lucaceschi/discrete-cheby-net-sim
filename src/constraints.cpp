#include "constraints.hpp"

#include <iostream>


EdgeLengthConstraint::EdgeLengthConstraint(std::vector<std::shared_ptr<Net>>&  nets,
                                           int netIndex, int edgeIndex, float edgeLength)
    : net_(nets[netIndex]),
      netIdx_(netIndex),
      edgeIdx_(edgeIndex),
      edgeLenSquared_(std::pow(edgeLength, 2))
{}

float EdgeLengthConstraint::solve(std::vector<Eigen::Matrix3Xf>& newPos,
                                  std::vector<Eigen::ArrayXf>& updateCounts) const
{
    std::shared_ptr<Net> net = net_.lock();
    if(!net)
        return 0;
    
    int p0Indx = net->edge(edgeIdx_)[0];
    int p1Indx = net->edge(edgeIdx_)[1];

    Eigen::Vector3f v = net->nodePos(p0Indx) - net->nodePos(p1Indx);
    double dist = v.squaredNorm();
    double delta = (edgeLenSquared_ - dist);
    double s = delta / (4 * dist);
    Eigen::Vector3f deltaV = s * v;

    newPos[netIdx_].col(p0Indx) += (net->nodePos(p0Indx) + deltaV);
    newPos[netIdx_].col(p1Indx) += (net->nodePos(p1Indx) - deltaV);

    updateCounts[netIdx_][p0Indx]++;
    updateCounts[netIdx_][p1Indx]++;

    return delta;
}