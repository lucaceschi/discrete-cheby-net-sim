#include "constraints.hpp"

#include <iostream>


EdgeLengthConstraint::EdgeLengthConstraint(std::vector<Net*>& nets, int netIndex, float edgeLength)
    : netIdx_(netIndex),
      edgeLenSquared_(std::pow(edgeLength, 2))
{}

float EdgeLengthConstraint::solve(std::vector<Net*>& nets) const
{
    Net* n = nets[netIdx_];
    double maxDelta = 0.0;
    
    for(int e = 0; e < n->getNEdges(); e++)
    {
        Eigen::Vector3f v = n->nodePos(n->edge(e)[0]) - n->nodePos(n->edge(e)[1]);
        double dist = v.squaredNorm();
        double delta = (edgeLenSquared_ - dist);
        double s = delta / (4 * dist);
        Eigen::Vector3f deltaV = s * v;

        n->nodePos(n->edge(e)[0]) += deltaV;
        n->nodePos(n->edge(e)[1]) -= deltaV;

        maxDelta = std::max(maxDelta, std::abs(delta));
    }

    return maxDelta;
}