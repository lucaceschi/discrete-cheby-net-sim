#include "forces.hpp"


UnaryForce::~UnaryForce() {}


ConstantForce::ConstantForce(Eigen::Vector3f vec)
    : vec(vec)
{}

ConstantForce::~ConstantForce() {}

void ConstantForce::applyForce(std::vector<Net*>& nets, int netIndex, int nodeIndex) const
{
    nets[netIndex]->nodePos(nodeIndex) += vec;
}