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
    EdgeLengthConstraint(int netIndex, float edgeLength);
    
    float solve(std::vector<Net*>& nets) const;

private:
    int netIdx_;
    float edgeLenSquared_;
};


class SphereCollConstr : public ConstraintTask
{
public:
    SphereCollConstr(int netIdx, Eigen::Vector3f centerPos, float radius);

    float solve(std::vector<Net*>& nets) const;

private:
    Eigen::Vector3f centerPos_;
    int netIdx_;
    float radius_;
};


class ShearLimitConstr : public ConstraintTask
{
public:
    ShearLimitConstr(int netIdx, float edgeLength, float minRadians);

    virtual float solve(std::vector<Net*>& nets) const;
    void setLimit(float edgeLength, float minRadians);

private:
    int netIdx_;
    float squaredLen_;
};


#endif