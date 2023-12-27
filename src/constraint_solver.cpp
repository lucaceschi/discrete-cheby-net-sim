#include "constraint_solver.hpp"

#include <vector>
#include <iostream>

#include <omp.h>


ConstraintSolver::ConstraintSolver(std::vector<Net*>& nets,
                                   float absoluteTolerance, float relativeTolerance, int maxIterations)
    : nets_(nets),
      absTol_(absoluteTolerance),
      relTol_(relativeTolerance),
      maxIters_(maxIterations),
      tasks_(),
      nIters_(0)
{}

void ConstraintSolver::addConstraint(std::shared_ptr<ConstraintTask> constraintTask)
{
    tasks_.push_back(std::move(constraintTask));
}

ConstraintSolver::ReturnCode ConstraintSolver::solve()
{
    static bool stop;
    static int nIters;
    static float meanDelta;
    static int totConstraints;
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
        totConstraints = 0;

        for(std::shared_ptr<ConstraintTask> const &tsk : tasks_)
        {
            meanDelta += tsk->solve(nets_);
            totConstraints += tsk->nConstraints(nets_);
        }

        meanDelta /= totConstraints;

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
    meanDelta_ = meanDelta;

    return returnCode;
}

const int ConstraintSolver::getNIters()
{
    return nIters_;
}

const float ConstraintSolver::getMeanDelta()
{
    return meanDelta_;
}