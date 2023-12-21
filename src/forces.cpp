#include "forces.hpp"


Force::~Force() {}

ConstantForce::ConstantForce(Eigen::Vector3f vec)
    : vec_(vec)
{}

ConstantForce::~ConstantForce() {}

void ConstantForce::applyForce(Eigen::Ref<Eigen::Vector3f> pos) const
{
    pos += vec_;
}

Eigen::Vector3f ConstantForce::getVector()
{
    return vec_;
}

void ConstantForce::setVector(Eigen::Vector3f translationVec)
{
    vec_ = translationVec;
}