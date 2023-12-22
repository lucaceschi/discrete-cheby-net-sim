#include "constraint_solver.hpp"

#include <vector>
#include <iostream>

#include <omp.h>


ConstraintSolver::ConstraintSolver(std::vector<Net*>& nets, int nNets,
                        float absoluteTolerance, float relativeTolerance, int maxIterations)
    : nets_(nets),
      nNets_(nNets),
      absTol_(absoluteTolerance),
      relTol_(relativeTolerance),
      maxIters_(maxIterations),
      tasks_(),
      nIters_(0)
{}

void ConstraintSolver::addConstraint(std::unique_ptr<ConstraintTask> constraintTask)
{
    tasks_.push_back(std::move(constraintTask));
}

ConstraintSolver::ReturnCode ConstraintSolver::solve()
{
    static bool stop;
    static int nIters;
    static float meanDelta;
    static float firstMeanDelta;
    static float totPrevNorm;
    static ConstraintSolver::ReturnCode returnCode;

    stop = false;
    nIters = 0;
    firstMeanDelta = 0;
    while(!stop)
    {
        nIters++;
        meanDelta = 0;

        for(std::unique_ptr<ConstraintTask> const &tsk : tasks_)
            meanDelta += tsk->solve(nets_);


        if(nIters >= maxIters_)
        {
            stop = true;
            returnCode = ReturnCode::REACHED_MAX_ITERS;
        }
        else if(meanDelta <= absTol_ + firstMeanDelta * relTol_)
        {
            stop = true;
            returnCode = ReturnCode::CONVERGENCE;
        }

        if(firstMeanDelta == 0)
            firstMeanDelta = meanDelta;
    }

    nIters_ = nIters;
    totDisplacement_ = meanDelta;
    maxDelta_ = meanDelta;

    return returnCode;
}

const int ConstraintSolver::getNIters()
{
    return nIters_;
}

const float ConstraintSolver::getTotDisplacement()
{
    return totDisplacement_;
}

const float ConstraintSolver::getMaxDelta()
{
    return maxDelta_;
}