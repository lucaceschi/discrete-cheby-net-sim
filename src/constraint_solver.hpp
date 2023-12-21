#ifndef CONSTR_SOLVER_H
#define CONSTR_SOLVER_H

#include "net.hpp"
#include "constraints.hpp"

#include <vector>
#include <memory>

#include <Eigen/Dense>


class ConstraintSolver
{
public:
    enum class ReturnCode { CONVERGENCE, REACHED_MAX_ITERS };
    
    ConstraintSolver();
    ConstraintSolver(std::vector<std::shared_ptr<Net>>& nets, int nNets,
                     float absoluteTolerance, float relativeTolerance, int maxIterations,
                     int nThreads);

    void addConstraint(std::unique_ptr<ConstraintTask> constraintTask);

    ReturnCode solve();

    const int getNIters();
    const float getTotDisplacement();
    const float getMaxDelta();

private:
    std::vector<std::shared_ptr<Net>>& nets_;
    int nNets_;
    int totNodes_;
    std::vector<std::vector<Eigen::Matrix3Xf>> newPosLocal_;
    std::vector<std::vector<Eigen::ArrayXf>> updateCountsLocal_;

    float absTol_, relTol_;
    int maxIters_;

    std::vector<std::unique_ptr<ConstraintTask>> tasks_;

    int nThreads_;

    volatile int nIters_;
    volatile float totDisplacement_;
    volatile float maxDelta_;
};


#endif