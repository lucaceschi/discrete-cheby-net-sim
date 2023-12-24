#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include "net.hpp"

#include <memory>
#include <vector>
#include <map>

#include <Eigen/Dense>


class ConstraintTask
{
public:
    virtual float solve(std::vector<Net*>& nets) const = 0;
    virtual int nConstraints(std::vector<Net*>& nets) const = 0;
};


class FixedNodeConstraint : public ConstraintTask
{
public:
    FixedNodeConstraint(std::vector<Net*>& nets);

    void fixNode(std::vector<Net*>& nets, int netIndex, int nodeIndex);
    bool isNodeFixed(int netIndex, int nodeIndex) const;
    void freeNode(int netIndex, int nodeIndex);

    float solve(std::vector<Net*>& nets) const;
    int nConstraints(std::vector<Net*>& nets) const;

private:
    std::vector<std::unordered_map<int, Eigen::Vector3f>> fixedPos_;
};


class EdgeLengthConstraint : public ConstraintTask
{
public:
    EdgeLengthConstraint(int netIndex, float edgeLength);
    
    float solve(std::vector<Net*>& nets) const;
    int nConstraints(std::vector<Net*>& nets) const;

private:
    int netIdx_;
    float edgeLenSquared_;
};


class SphereCollConstr : public ConstraintTask
{
public:
    SphereCollConstr(int netIdx, Eigen::Vector3f centerPos, float radius);

    float solve(std::vector<Net*>& nets) const;
    int nConstraints(std::vector<Net*>& nets) const;

private:
    Eigen::Vector3f centerPos_;
    int netIdx_;
    float radius_;
};


class ShearLimitConstr : public ConstraintTask
{
public:
    ShearLimitConstr(int netIdx, float edgeLength, float minRadians);

    float solve(std::vector<Net*>& nets) const;
    int nConstraints(std::vector<Net*>& nets) const;
    void setLimit(float edgeLength, float minRadians);

private:
    int netIdx_;
    float squaredLen_;
};


#endif