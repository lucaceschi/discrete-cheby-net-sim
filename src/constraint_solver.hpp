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
    ConstraintSolver(std::vector<Net*>& nets,
                     float absoluteTolerance, float relativeTolerance, int maxIterations);

    void addConstraint(std::shared_ptr<ConstraintTask> constraintTask);

    ReturnCode solve();

    const int getNIters();
    const float getMeanDelta();

private:
    std::vector<Net*>& nets_;

    float absTol_, relTol_;
    int maxIters_;

    std::vector<std::shared_ptr<ConstraintTask>> tasks_;

    volatile int nIters_;
    volatile float meanDelta_;
};


#endif