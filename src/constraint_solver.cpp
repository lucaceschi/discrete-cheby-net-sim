#include "constraint_solver.hpp"

#include <vector>
#include <iostream>

#include <omp.h>


ConstraintSolver::ConstraintSolver(std::vector<std::shared_ptr<Net>>& nets, int nNets,
                 float absoluteTolerance, float relativeTolerance, int maxIterations,
                 int nThreads)
    : nets_(nets),
      nNets_(nNets),
      totNodes_(0),
      newPosLocal_(nThreads, std::vector<Eigen::Matrix3Xf>(nNets)),
      updateCountsLocal_(nThreads, std::vector<Eigen::ArrayXf>(nNets)),
      absTol_(absoluteTolerance),
      relTol_(relativeTolerance),
      maxIters_(maxIterations),
      tasks_(),
      nThreads_(nThreads),
      nIters_(0)
{
    for(int t = 0; t < nThreads_; t++)
        for(int n = 0; n < nNets; n++)
        {
            newPosLocal_[t][n] = Eigen::Matrix3Xf(3, nets_[n]->getNNodes());
            updateCountsLocal_[t][n] = Eigen::ArrayXf(nets_[n]->getNNodes());
        }

    for(int n = 0; n < nNets; n++)
        totNodes_ += nets_[n]->getNNodes();
}

void ConstraintSolver::addConstraint(std::unique_ptr<ConstraintTask> constraintTask)
{
    tasks_.push_back(std::move(constraintTask));
}

ConstraintSolver::ReturnCode ConstraintSolver::solve()
{       
    static bool stop;
    static int nIters;
    static ConstraintSolver::ReturnCode returnCode;
    static float totDisplacement;
    static float maxDelta;
    static float totNorm;

    stop = false;
    nIters = 0;
    
    #pragma omp parallel
    {
        int thNum = omp_get_thread_num();
        float maxDeltaLocal;
        float totDisplacementLocal;
        float totNormLocal;

        while(!stop)
        {
            // azzera posizioni locali
            for(int n = 0; n < nNets_; n++)
            {
                newPosLocal_[thNum][n].setZero();
                updateCountsLocal_[thNum][n].setZero();
            }

            // parallel for dynamic risoluzione tasks
            maxDeltaLocal = 0;
            #pragma omp for schedule(dynamic)
            for(std::unique_ptr<ConstraintTask> const &tsk : tasks_)
                maxDeltaLocal = std::max(tsk->solve(newPosLocal_[thNum], updateCountsLocal_[thNum]),
                                         maxDeltaLocal);
            // (barriera implicita)
            
            // parallel for static su ogni particella
            totDisplacementLocal = 0;
            totNormLocal = 0;
            #pragma omp for schedule(static) nowait
            for(int i = 0; i < totNodes_; i++)
            {
                int netIdx = 0;
                int nodeIdx = i;

                while(nodeIdx >= nets_[netIdx]->getNNodes())
                {
                    nodeIdx -= nets_[netIdx]->getNNodes();
                    netIdx++;
                }

                // incrementa la norma quadrata delle attuali posizioni
                totNormLocal += nets_[netIdx]->nodePos(nodeIdx).squaredNorm();

                // fai la media di tutte le posizioni finali calcolate dai thread
                // e incrementa la norma quadrata dei displacement
                int totUpdateCounts = updateCountsLocal_[0][netIdx][nodeIdx];
                for(int t = 1; t < nThreads_; t++)
                {
                    newPosLocal_[0][netIdx].col(nodeIdx) += newPosLocal_[t][netIdx].col(nodeIdx);
                    totUpdateCounts += updateCountsLocal_[t][netIdx][nodeIdx];
                }
                if(totUpdateCounts != 0)
                {
                    newPosLocal_[0][netIdx].col(nodeIdx) /= totUpdateCounts;
                    totDisplacementLocal += (newPosLocal_[0][netIdx].col(nodeIdx)
                                             - nets_[netIdx]->nodePos(nodeIdx))
                                             .squaredNorm();

                    nets_[netIdx]->nodePos(nodeIdx) = newPosLocal_[0][netIdx].col(nodeIdx);
                }
            }
            // (no barriera implicita)

            // raduna displacement totale, delta massimo, norma totale
            #pragma omp single
            {
                totDisplacement = 0;
                maxDelta = 0;
                totNorm = 0;
            }
            // (barriera implicita)
            #pragma omp critical
            {
                totDisplacement += totDisplacementLocal;
                maxDelta = std::max(maxDelta, maxDeltaLocal);
                totNorm += totNormLocal;
            }

            // pusha info e verifica stopping criteria
            #pragma omp single
            {
                nIters++;

                if(totDisplacement <= absTol_ + totNorm * relTol_)
                {
                    stop = true;
                    returnCode = ReturnCode::CONVERGENCE;
                }
                else if(nIters >= maxIters_)
                {
                    stop = true;
                    returnCode = ReturnCode::REACHED_MAX_ITERS;
                }
            }
            // (barriera implicita)
        }
    }

    nIters_ = nIters;
    totDisplacement_ = totDisplacement;
    maxDelta_ = maxDelta;

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