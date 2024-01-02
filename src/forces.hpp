#ifndef FORCES_HPP
#define FORCES_HPP

#include "net.hpp"

#include <Eigen/Dense>


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


#endif