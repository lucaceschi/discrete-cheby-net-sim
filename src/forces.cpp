#include "forces.hpp"


Force::~Force() {}

ConstantForce::ConstantForce(Eigen::Vector3f vec)
    : vec(vec)
{}

ConstantForce::~ConstantForce() {}

void ConstantForce::applyForce(Eigen::Ref<Eigen::Vector3f> pos) const
{
    pos += vec;
}