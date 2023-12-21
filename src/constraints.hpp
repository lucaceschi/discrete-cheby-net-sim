#ifndef CONSTRAINTS_HPP
#define CONSTRAINTS_HPP

#include "net.hpp"

#include <memory>
#include <vector>

#include <Eigen/Dense>


class ConstraintTask
{
public:
    virtual float solve(std::vector<Eigen::Matrix3Xf>& newPos,
                        std::vector<Eigen::ArrayXf>& updateCounts) const = 0;
};


class EdgeLengthConstraint : public ConstraintTask
{
public:
    EdgeLengthConstraint(std::vector<std::shared_ptr<Net>>& nets,
                         int netIndex, int edgeIndex, float edgeLength);
    
    float solve(std::vector<Eigen::Matrix3Xf>& newPos,
                std::vector<Eigen::ArrayXf>& updateCounts) const;

private:
    std::weak_ptr<Net> net_;
    int netIdx_;
    int edgeIdx_;
    float edgeLenSquared_;
};


#endif