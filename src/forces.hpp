#ifndef FORCES_HPP
#define FORCES_HPP

#include "net.hpp"

#include <Eigen/Dense>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/Interpolation.h>


class UnaryForce
{
public:
    virtual ~UnaryForce();
    virtual void applyForce(std::vector<Net*>& nets, int netIndex, int nodeIndex) const = 0;
};


class ConstantForce : public UnaryForce
{
public:
    ConstantForce(Eigen::Vector3f vec);
    virtual ~ConstantForce();

    virtual void applyForce(std::vector<Net*>& nets, int netIndex, int nodeIndex) const;

    Eigen::Vector3f vec;
    
};


class DiscreteSDFFittingForce : public UnaryForce
{
using Vec3f = openvdb::math::Vec3<float>;
using Coord = openvdb::Coord;
using Transform = openvdb::math::Transform;
using FloatGrid = openvdb::FloatGrid;
using FloatGradient = openvdb::tools::Gradient<FloatGrid>;
using BoxSampler = openvdb::tools::BoxSampler;
using FloatGridSampler = openvdb::tools::GridSampler<FloatGrid::ConstAccessor, BoxSampler>;
using FloatGradientGridSampler = openvdb::tools::GridSampler<FloatGradient::OutGridType::ConstAccessor, BoxSampler>;

public:
    DiscreteSDFFittingForce(std::vector<Net*>& nets,
                               FloatGrid::Ptr sdfGrid,
                               float smoothstepNearBound,
                               float smoothstepFarBound,
                               Eigen::Vector3f worldTranslationVec);
    virtual ~DiscreteSDFFittingForce();

    virtual void applyForce(std::vector<Net*>& nets, int netIndex, int nodeIndex) const;

    float getMaxFarBound() const;
    float getNearBound() const;
    float getFarBound() const;
    void setNearBound(float smoothstepNearBound);
    void setFarBound(float smoothstepFarBound);

    Eigen::Vector3f getWorldTranslationVec() const;
    void setWorldTranslationVec(Eigen::Vector3f worldTranslationVec);

private:
    float clamp(float x, float lowerlimit = 0.0f, float upperlimit = 1.0f) const;
    float smoothstep(float edge0, float edge1, float x) const;

    FloatGrid::Ptr sdfGrid_;
    Transform::Ptr transform_;
    FloatGradient::OutGridType::Ptr gradGrid_;

    float intBandWidth_;
    float extBandWidth_;

    float nearBound_;
    float farBound_;

    float maxFarBound_;

    Eigen::Vector3f worldTranslationVec_;
    float worldTranslationVecNorm_;

    std::vector<std::vector<FloatGrid::ConstAccessor>> sdfGridAccs_;
    std::vector<std::vector<FloatGridSampler>> sdfGridSamplers_;
    std::vector<std::vector<FloatGradient::OutGridType::ConstAccessor>> gradGridAccs_;
    std::vector<std::vector<FloatGradientGridSampler>> gradGridSamplers_;
};


#endif