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
    ConstraintTask();
    float solve(std::vector<Net*>& nets) const;
    virtual int nConstraints(std::vector<Net*>& nets) const = 0;

    bool active;

private:
    virtual float solve_(std::vector<Net*>& nets) const = 0;
};


class FixedNodeConstraint : public ConstraintTask
{
public:
    FixedNodeConstraint(std::vector<Net*>& nets);

    void fixNode(std::vector<Net*>& nets, int netIndex, int nodeIndex);
    bool isNodeFixed(int netIndex, int nodeIndex) const;
    void freeNode(int netIndex, int nodeIndex);
    void freeNodes(int netIndex);

    int nConstraints(std::vector<Net*>& nets) const;

private:
    float solve_(std::vector<Net*>& nets) const;

    std::vector<std::unordered_map<int, Eigen::Vector3f>> fixedPos_;
};


class EdgeLengthConstraint : public ConstraintTask
{
public:
    EdgeLengthConstraint(std::vector<Net*>& nets, int netIndex);
    
    void updateEdgeLengths(std::vector<Net*>& nets);

    int nConstraints(std::vector<Net*>& nets) const;

private:
    float solve_(std::vector<Net*>& nets) const;

    int netIdx_;
    Eigen::ArrayXf edgeLensSquared_;
};


class SphereCollConstr : public ConstraintTask
{
public:
    SphereCollConstr(Eigen::Vector3f centerPos, float radius);

    int nConstraints(std::vector<Net*>& nets) const;

private:
    float solve_(std::vector<Net*>& nets) const;

    Eigen::Vector3f centerPos_;
    float radius_;
};


class ShearLimitConstr : public ConstraintTask
{
public:
    ShearLimitConstr(int netIdx, float edgeLength, float minRadians);

    int nConstraints(std::vector<Net*>& nets) const;
    void setLimit(float edgeLength, float minRadians);

private:
    float solve_(std::vector<Net*>& nets) const;

    int netIdx_;
    float squaredLen_;
};


#endif