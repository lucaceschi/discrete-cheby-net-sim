#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include "net.hpp"

#include <memory>
#include <vector>

#include <Eigen/Dense>


class ConstraintTask
{
public:
    virtual float solve(std::vector<Net*>& nets) const = 0;
};


class EdgeLengthConstraint : public ConstraintTask
{
public:
    EdgeLengthConstraint(std::vector<Net*>& nets, int netIndex, float edgeLength);
    
    float solve(std::vector<Net*>& nets) const;

private:
    int netIdx_;
    float edgeLenSquared_;
};


#endif