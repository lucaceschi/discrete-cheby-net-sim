#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include "net.hpp"

#include <memory>
#include <vector>
#include <map>

#include <Eigen/Dense>
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/GridOperators.h>


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


class PlanarBoundaryConstr : public ConstraintTask
{
public:
    PlanarBoundaryConstr(Eigen::Vector3f pointOnBoundary, Eigen::Vector3f normalVec);

    int nConstraints(std::vector<Net*>& nets) const;

    Eigen::Vector3f pointOnBoundary;
    Eigen::Vector3f normalVec;

private:
    float solve_(std::vector<Net*>& nets) const;
};


class SphereCollConstr : public ConstraintTask
{
public:
    SphereCollConstr(Eigen::Vector3f centerPos, float radius);

    int nConstraints(std::vector<Net*>& nets) const;

    Eigen::Vector3f centerPos;
    float radius;

private:
    float solve_(std::vector<Net*>& nets) const;
};


class ShearLimitConstr : public ConstraintTask
{
public:
    ShearLimitConstr(int netIdx, float edgeLength, float minRadians);

    int nConstraints(std::vector<Net*>& nets) const;
    float getLimit();
    void setLimit(float minRadians);

private:
    float solve_(std::vector<Net*>& nets) const;

    int netIdx_;
    float len_;
    float squaredLen_;
    float minRadians_;
};


template <typename F, typename DF>
class SDFCollConstr : public ConstraintTask
{
public:
    SDFCollConstr(F* sdf, DF* dsdf, bool exact);

    int nConstraints(std::vector<Net*>& nets) const;

private:
    float solve_(std::vector<Net*>& nets) const;

    F* sdf_;
    DF* dsdf_;
    bool exact_;
};


class DiscreteSDFCollConstr : public ConstraintTask
{
using Vec3d = openvdb::math::Vec3d;
using Coord = openvdb::Coord;
using Transform = openvdb::math::Transform;
using FloatGrid = openvdb::FloatGrid;
using FloatGradient = openvdb::tools::Gradient<FloatGrid>;

public:
    DiscreteSDFCollConstr(std::vector<Net*>& nets, FloatGrid::Ptr sdfGrid);

    int nConstraints(std::vector<Net*>& nets) const;

private:
    float solve_(std::vector<Net*>& nets) const;

    Transform::Ptr transform_;
    FloatGrid::Ptr sdfGrid_;
    FloatGradient::OutGridType::Ptr gradGrid_;

    std::vector<std::vector<FloatGrid::ConstAccessor>> sdfGridAccs_;
    std::vector<std::vector<FloatGradient::OutGridType::ConstAccessor>> gradGridAccs_;
};


#endif