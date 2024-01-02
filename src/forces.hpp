#ifndef FORCES_HPP
#define FORCES_HPP

#include "net.hpp"

#include <Eigen/Dense>
#include <openvdb/openvdb.h>
#include <openvdb/tools/GridOperators.h>


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


class DiscreteSDFAttractionForce : public UnaryForce
{
using Vec3d = openvdb::math::Vec3d;
using Coord = openvdb::Coord;
using Transform = openvdb::math::Transform;
using FloatGrid = openvdb::FloatGrid;
using FloatGradient = openvdb::tools::Gradient<FloatGrid>;

public:
    DiscreteSDFAttractionForce(std::vector<Net*>& nets,
                               FloatGrid::Ptr sdfGrid,
                               float smoothstepNearBound,
                               float smoothstepFarBound,
                               Eigen::Vector3f worldTranslationVec);
    virtual ~DiscreteSDFAttractionForce();

    virtual void applyForce(std::vector<Net*>& nets, int netIndex, int nodeIndex) const;

    float getMinNearBound() const;
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

    float minNearBound_;
    float maxFarBound_;

    Eigen::Vector3f worldTranslationVec_;
    float worldTranslationVecNorm_;

    std::vector<std::vector<FloatGrid::ConstAccessor>> sdfGridAccs_;
    std::vector<std::vector<FloatGradient::OutGridType::ConstAccessor>> gradGridAccs_;
};


#endif