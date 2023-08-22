#ifndef CONSTRAINTS_HPP_
#define CONSTRAINTS_HPP_

#include "grid.hpp"
#include "param_distance3.hpp"

#include <Eigen/Dense>


class HardConstraint
{
public:
    virtual double resolve() const = 0;
};


class EdgeLenConstr : public HardConstraint
{
public:
    EdgeLenConstr(Grid* grid, double length)
        : grid_(grid),
          lens_(grid->getNEdges(), length)
    {}

    EdgeLenConstr(Grid* grid, std::vector<double> lengths)
        : grid_(grid),
          lens_(lengths)
    {}

    virtual double resolve() const
    {
        double maxDelta = 0.0;
        
        for(int e = 0; e < grid_->getNEdges(); e++)
        {
            Eigen::Vector3d v = grid_->nodePos(grid_->edge(e)[0]) - grid_->nodePos(grid_->edge(e)[1]);
            double dist = v.norm();
            v.normalize();
            double delta = (lens_[e] - dist);

            if(grid_->isNodeFixed(grid_->edge(e)[0]))
                grid_->nodePos(grid_->edge(e)[1]) -= delta * v;
            else if(grid_->isNodeFixed(grid_->edge(e)[1]))
                grid_->nodePos(grid_->edge(e)[0]) += delta * v;
            else
            {
                delta /= 2;
                grid_->nodePos(grid_->edge(e)[0]) += delta * v;
                grid_->nodePos(grid_->edge(e)[1]) -= delta * v;
            }

            maxDelta = std::max(maxDelta, std::abs(delta));
        }

        return maxDelta;
    }

    double getLength(int e) const { return lens_[e]; }

private:
    Grid* grid_;
    std::vector<double> lens_;
};


class SphereCollConstr : public HardConstraint
{
public:
    SphereCollConstr(Grid* grid, Eigen::Vector3d centerPos, double radius)
        : grid_(grid),
          centerPos_(centerPos),
          radius_(radius)
    {}

    virtual double resolve() const
    {
        double currDist, currDelta, maxDelta = 0;
        
        for(int n = 0; n < grid_->getNNodes(); n++)
        {
            if(grid_->isNodeFixed(n))
                continue;
            
            currDist = (grid_->nodePos(n) - centerPos_).norm();
            currDelta = radius_ - currDist; 
            if(currDelta > 0)
            {
                maxDelta = std::max(maxDelta, currDelta);
                
                if((grid_->nodePos(n) - centerPos_).isZero())
                    grid_->nodePos(n) += Eigen::Vector3d{0, radius_, 0};
                else
                    grid_->nodePos(n) = (grid_->nodePos(n) - centerPos_) * (radius_ / currDist) + centerPos_;
            }
        }

        return maxDelta;
    }

    Eigen::Vector3d centerPos_;
private:
    Grid* grid_;
    double radius_;
};


class ScissorConstr : public HardConstraint
{
public:
    ScissorConstr(Grid* gridA, int nodeA0Indx, int nodeA1Indx,
                  Grid* gridB, int nodeB0Indx, int nodeB1Indx)
        : gridA_(gridA),
          nodeA0Indx_(nodeA0Indx),
          nodeA1Indx_(nodeA1Indx),
          gridB_(gridB),
          nodeB0Indx_(nodeB0Indx),
          nodeB1Indx_(nodeB1Indx)
    {
        vcg::Segment3d segA = vcg::Segment3d(
            vcg::Point3d(gridA->nodePos(nodeA0Indx).data()),
            vcg::Point3d(gridA->nodePos(nodeA1Indx).data())
        );

        vcg::Segment3d segB = vcg::Segment3d(
            vcg::Point3d(gridB->nodePos(nodeB0Indx).data()),
            vcg::Point3d(gridB->nodePos(nodeB1Indx).data())
        );

        bool parallel;
        vcg::Point3d midpointA, midpointB;
        vcg::SegmentSegmentDistancePar(segA, segB, dist_, parallel, alpha_, beta_, midpointA, midpointB);
        omegaA_ = 1.0 / (std::pow(alpha_, 2) + std::pow(1 - alpha_, 2));
        omegaB_ = 1.0 / (std::pow(beta_,  2) + std::pow(1 - beta_,  2));

        if(parallel)
            frmwrk::Debug::logWarning("Found parallel edges while adding a new scissor constraint");
    }

    virtual double resolve() const
    {
        Eigen::Vector3d shiftDir = getMidpointB() - getMidpointA();
        double delta = (shiftDir.norm() - dist_);
        double deltaA = delta * omegaA_ / (omegaA_ + omegaB_);
        double deltaB = delta * omegaB_ / (omegaA_ + omegaB_);
        double deltaA0 = deltaA * (1 - alpha_) * omegaA_;
        double deltaA1 = deltaA * (alpha_)     * omegaA_;
        double deltaB0 = deltaB * (1 - beta_)  * omegaB_;
        double deltaB1 = deltaB * (beta_)      * omegaB_;
        shiftDir.normalize();

        gridA_->nodePos(nodeA0Indx_) += deltaA0 * shiftDir;
        gridA_->nodePos(nodeA1Indx_) += deltaA1 * shiftDir;
        gridB_->nodePos(nodeB0Indx_) -= deltaB0 * shiftDir;
        gridB_->nodePos(nodeB1Indx_) -= deltaB1 * shiftDir;

        return std::max({std::abs(deltaA0), std::abs(deltaA1), std::abs(deltaB0), std::abs(deltaB1)});
    }

    Grid* getGridA() const { return gridA_; }
    Grid* getGridB() const { return gridB_; }
    int getNodeA0Indx() const { return nodeA0Indx_; }
    int getNodeA1Indx() const { return nodeA1Indx_; }
    int getNodeB0Indx() const { return nodeB0Indx_; }
    int getNodeB1Indx() const { return nodeB1Indx_; }
    double getDist() const { return dist_; }
    double getAlpha() const { return alpha_; }
    double getBeta() const { return beta_; }

    inline Eigen::Vector3d getMidpointA() const
    {
        return gridA_->nodePos(nodeA0Indx_) * (1 - alpha_) + gridA_->nodePos(nodeA1Indx_) * alpha_;
    }

    inline Eigen::Vector3d getMidpointB() const
    {
        return gridB_->nodePos(nodeB0Indx_) * (1 - beta_) + gridB_->nodePos(nodeB1Indx_) * beta_;
    }

private:
    Grid* gridA_;
    int nodeA0Indx_, nodeA1Indx_;
    double alpha_, omegaA_;
    Grid* gridB_;
    int nodeB0Indx_, nodeB1Indx_;
    double beta_, omegaB_;
    double dist_;
};


#endif