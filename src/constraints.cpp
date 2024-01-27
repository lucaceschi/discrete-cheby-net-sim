#include "framework/debug.hpp"
#include "constraints.hpp"
#include "param_distance3.hpp"

#include <iostream>

#include <vcg/space/point3.h>
#include <vcg/space/segment3.h>


#define SDF_RAYCASTING_TOLERANCE 1e-6


ConstraintTask::ConstraintTask()
    : active(true)
{}

float ConstraintTask::solve(std::vector<Net*>& nets) const
{
    return (active)? solve_(nets) : 0;
}


FixedNodeConstraint::FixedNodeConstraint(std::vector<Net*>& nets)
    : fixedPos_(nets.size())
{}

void FixedNodeConstraint::fixNode(std::vector<Net*>& nets, int netIndex, int nodeIndex)
{
    fixedPos_[netIndex][nodeIndex] = nets[netIndex]->nodePos(nodeIndex);
}

bool FixedNodeConstraint::isNodeFixed(int netIndex, int nodeIndex) const
{
    return fixedPos_[netIndex].find(nodeIndex) != fixedPos_[netIndex].cend();
}

void FixedNodeConstraint::freeNode(int netIndex, int nodeIndex)
{
    fixedPos_[netIndex].erase(nodeIndex);
}

void FixedNodeConstraint::freeNodes(int netIndex)
{
    fixedPos_[netIndex].clear();
}

float FixedNodeConstraint::solve_(std::vector<Net*>& nets) const
{
    typedef std::unordered_map<int, Eigen::Vector3f>::const_iterator CIterator;
    float meanDelta;

    meanDelta = 0;
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
        for(CIterator it = fixedPos_[netIdx].cbegin(); it != fixedPos_[netIdx].cend(); it++)
        {
            meanDelta += (nets[netIdx]->nodePos(it->first) - it->second).squaredNorm();
            nets[netIdx]->nodePos(it->first) = it->second;
        }

    return meanDelta;
}

int FixedNodeConstraint::nConstraints(std::vector<Net*>& nets) const
{
    int n = 0;
    for(const auto& m : fixedPos_)
        n += m.size();

    return n;
}


EdgeLengthConstraint::EdgeLengthConstraint(std::vector<Net*>& nets, int netIndex)
    : netIdx_(netIndex)
{
    updateEdgeLengths(nets);
}

void EdgeLengthConstraint::updateEdgeLengths(std::vector<Net*>& nets)
{
    edgeLensSquared_ = Eigen::ArrayXf(nets[netIdx_]->edgeLengths.square());
}

float EdgeLengthConstraint::solve_(std::vector<Net*>& nets) const
{
    Net* n = nets[netIdx_];
    float meanDelta;

    meanDelta = 0;
    for(int e = 0; e < n->getNEdges(); e++)
    {
        Eigen::Vector3f v = n->nodePos(n->edge(e)[0]) - n->nodePos(n->edge(e)[1]);
        float dist = v.squaredNorm();
        float delta = (edgeLensSquared_[e] - dist);
        float s = delta / (4 * dist);
        Eigen::Vector3f deltaV = s * v;

        n->nodePos(n->edge(e)[0]) += deltaV;
        n->nodePos(n->edge(e)[1]) -= deltaV;

        meanDelta += std::abs(delta);
    }

    return meanDelta;
}

int EdgeLengthConstraint::nConstraints(std::vector<Net*>& nets) const
{
    return nets[netIdx_]->getNEdges();
}


ShearLimitConstr::ShearLimitConstr(int netIdx, float edgeLength, float minRadians)
    : len_(edgeLength),
      netIdx_(netIdx)
{
    setLimit(minRadians);
}

float ShearLimitConstr::solve_(std::vector<Net*>& nets) const
{
    Net* n = nets[netIdx_];
    float meanDelta;

    meanDelta = 0;    
    for(int d = 0; d < n->getNDiagonals(); d++)
    {
        Eigen::Vector3f v = n->nodePos(n->diag(d)[0]) - n->nodePos(n->diag(d)[1]);
        float dist = v.squaredNorm();

        if(dist > squaredLen_)
            continue;

        float delta = (squaredLen_ - dist);
        float s = delta / (4 * dist);
        Eigen::Vector3f deltaV = s * v;

        n->nodePos(n->diag(d)[0]) += deltaV;
        n->nodePos(n->diag(d)[1]) -= deltaV;

        meanDelta += std::abs(delta);
    }

    return meanDelta;
}

int ShearLimitConstr::nConstraints(std::vector<Net*>& nets) const
{
    return nets[netIdx_]->getNDiagonals();
}

float ShearLimitConstr::getLimit()
{
    return minRadians_;
}

void ShearLimitConstr::setLimit(float minRadians)
{
    minRadians_ = minRadians;
    
    if(minRadians <= 0)
    {
        squaredLen_ = 0;
        minRadians_ = 0;
    }
    else if(minRadians >= M_PI_2)
    {
        squaredLen_ = std::pow(len_, 2) * M_SQRT2;
        minRadians_ = M_PI_2;
    }
    else
    {
        float twoSqEdgeLen = 2 * std::pow(len_, 2);
        squaredLen_ = twoSqEdgeLen - twoSqEdgeLen * std::cos(minRadians);
    }
}


PlanarBoundaryConstr::PlanarBoundaryConstr(Eigen::Vector3f pointOnBoundary, Eigen::Vector3f normalVec)
    : pointOnBoundary(pointOnBoundary),
      normalVec(normalVec)
{}

int PlanarBoundaryConstr::nConstraints(std::vector<Net*>& nets) const
{
    int n = 0;
    for(const Net* net : nets)
        n += net->getNNodes();

    return n; 
}

float PlanarBoundaryConstr::solve_(std::vector<Net*>& nets) const
{
    float currDelta, meanDelta;

    meanDelta = 0;
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
    {
        Net* n = nets[netIdx];

        for(int nodeIdx = 0; nodeIdx < n->getNNodes(); nodeIdx++)
        {
            currDelta = (pointOnBoundary - n->nodePos(nodeIdx)).dot(normalVec);
            
            if(currDelta > 0)
            {
                meanDelta += std::pow(currDelta, 2);
                n->nodePos(nodeIdx) += (currDelta * normalVec);
            }
        }
    }

    return meanDelta;
}


SphereCollConstr::SphereCollConstr(Eigen::Vector3f centerPos, float radius)
    : centerPos(centerPos),
      radius(radius)
{}

float SphereCollConstr::solve_(std::vector<Net*>& nets) const
{
    float currDist, currDelta, meanDelta;

    meanDelta = 0;
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
    {
        Net* n = nets[netIdx];

        for(int i = 0; i < n->getNNodes(); i++)
        {            
            currDist = (n->nodePos(i) - centerPos).norm();
            currDelta = radius - currDist; 
            if(currDelta > 0)
            {
                meanDelta += std::pow(currDelta, 2);
                
                if((n->nodePos(i) - centerPos).isZero())
                    n->nodePos(i) += Eigen::Vector3f{0, radius, 0};
                else
                    n->nodePos(i) = (n->nodePos(i) - centerPos) * (radius / currDist) + centerPos;
            }
        }
    }

    return meanDelta;
}

int SphereCollConstr::nConstraints(std::vector<Net*>& nets) const
{
    int n = 0;
    for(const Net* net : nets)
        n += net->getNNodes();

    return n;
}


template <typename F, typename DF>
SDFCollConstr<F, DF>::SDFCollConstr(F* sdf, DF* dsdf, bool exact)
    : sdf_(sdf),
      dsdf_(dsdf),
      exact_(exact)
{}

template <typename F, typename DF>
int SDFCollConstr<F, DF>::nConstraints(std::vector<Net*>& nets) const
{
    int n = 0;
    for(const Net* net : nets)
        n += net->getNNodes();

    return n;
}

template <typename F, typename DF>
float SDFCollConstr<F, DF>::solve_(std::vector<Net*>& nets) const
{
    float currDist, currDelta, meanDelta;

    meanDelta = 0;
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
    {
        Net* n = nets[netIdx];

        for(int nodeIdx = 0; nodeIdx < n->getNNodes(); nodeIdx++)
        {
            currDelta = 0;
            
            if(exact_)
            {
                currDist = sdf_(n->nodePos(nodeIdx));
                if(currDist < 0)
                {
                    n->nodePos(nodeIdx) -= (currDist * dsdf_(n->nodePos(nodeIdx)));
                    currDelta = currDist;
                }
            }
            else
            {
                while(true)
                {
                    currDist = sdf_(n->nodePos(nodeIdx));
                    if(currDist < -SDF_RAYCASTING_TOLERANCE)
                    {
                        n->nodePos(nodeIdx) -= (currDist * dsdf_(n->nodePos(nodeIdx)));
                        currDelta += currDist;
                    }
                    else
                        break;
                }
            }

            meanDelta += std::pow(currDelta, 2);
        }
    }

    return meanDelta;
}

template class SDFCollConstr<float(Eigen::Vector3f), Eigen::Vector3f(Eigen::Vector3f)>;


DiscreteSDFCollConstr::DiscreteSDFCollConstr(std::vector<Net*>& nets, FloatGrid::Ptr sdfGrid)
    : sdfGrid_(sdfGrid),
      transform_(sdfGrid->transformPtr()),
      sdfGridAccs_(nets.size()),
      gradGridAccs_(nets.size()),
      sdfGridSamplers_(nets.size()),
      gradGridSamplers_(nets.size())
{   
    // compute gradient grid
    FloatGradient grad(*sdfGrid_);
    gradGrid_ = grad.process();

    // initialize grid accessors
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
    {
        Net* n = nets[netIdx];

        sdfGridAccs_[netIdx].reserve(n->getNNodes());
        gradGridAccs_[netIdx].reserve(n->getNNodes());
        for(int nodeIdx = 0; nodeIdx < n->getNNodes(); nodeIdx++)
        {
            sdfGridAccs_[netIdx].emplace_back(sdfGrid_->getConstAccessor());
            gradGridAccs_[netIdx].emplace_back(gradGrid_->getConstAccessor());
            sdfGridSamplers_[netIdx].emplace_back(sdfGridAccs_[netIdx].back(), sdfGrid_->transform());
            gradGridSamplers_[netIdx].emplace_back(gradGridAccs_[netIdx].back(), sdfGrid_->transform());
        }
    }
}

int DiscreteSDFCollConstr::nConstraints(std::vector<Net*>& nets) const
{
    int n = 0;
    for(const Net* net : nets)
        n += net->getNNodes();

    return n;
}

float DiscreteSDFCollConstr::solve_(std::vector<Net*>& nets) const
{
    float currDist, meanDelta;
    Vec3f currNodeWorldPos;

    meanDelta = 0;
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
    {
        Net* n = nets[netIdx];

        for(int nodeIdx = 0; nodeIdx < n->getNNodes(); nodeIdx++)
        {
            currNodeWorldPos = Vec3f(n->nodePos(nodeIdx).data());
            currDist = sdfGridSamplers_[netIdx][nodeIdx].wsSample(currNodeWorldPos);

            if(currDist < 0)
            {
                Eigen::Vector3f grad = Eigen::Vector3f(gradGridSamplers_[netIdx][nodeIdx].wsSample(currNodeWorldPos).asPointer());
                n->nodePos(nodeIdx) -= (currDist * grad);
                meanDelta += std::pow(currDist, 2);
            }
        }
    }

    return meanDelta;
}


ContactConstraint::Contact::Contact(std::vector<Net*>& nets,
                                    int netIndexA, int edgeIndexA,
                                    int netIndexB, int edgeIndexB)
    : netIdxA(netIndexA),
      edgeIdxA(edgeIndexA),
      netIdxB(netIndexB),
      edgeIdxB(edgeIndexB)
{
    Net* netA = nets[netIndexA];
    Net* netB = nets[netIndexB];

    vcg::Segment3f segmentA = vcg::Segment3f(
        vcg::Point3f(netA->nodePos(netA->edge(edgeIndexA)[0]).data()),
        vcg::Point3f(netA->nodePos(netA->edge(edgeIndexA)[1]).data())
    );

    vcg::Segment3f segmentB = vcg::Segment3f(
        vcg::Point3f(netB->nodePos(netB->edge(edgeIndexB)[0]).data()),
        vcg::Point3f(netB->nodePos(netB->edge(edgeIndexB)[1]).data())
    );

    bool parallel;
    vcg::Point3f contactPointA, contactPointB;
    vcg::SegmentSegmentDistancePar(segmentA, segmentB,
                                   distance,
                                   parallel,
                                   alpha, beta,
                                   contactPointA, contactPointB);

    if(parallel)
        frmwrk::Debug::logWarning("Found parallel edges while computing a new contact");

    omegaA = 1.0 / (std::pow(alpha, 2) + std::pow(1 - alpha, 2));
    omegaB = 1.0 / (std::pow(beta,  2) + std::pow(1 - beta,  2));
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> ContactConstraint::Contact::getContactPoints(std::vector<Net*>& nets) const
{
    Net* netA = nets[netIdxA];
    Net* netB = nets[netIdxB];

    Eigen::Vector3f nodeA1 = netA->nodePos(netA->edge(edgeIdxA)[0]);
    Eigen::Vector3f nodeA2 = netA->nodePos(netA->edge(edgeIdxA)[1]);
    Eigen::Vector3f nodeB1 = netB->nodePos(netB->edge(edgeIdxB)[0]);
    Eigen::Vector3f nodeB2 = netB->nodePos(netB->edge(edgeIdxB)[1]);

    return std::make_pair(
        nodeA1 * (1 - alpha) + nodeA2 * (alpha),
        nodeB1 * (1 - beta)  + nodeB2 * (beta)
    );
}

ContactConstraint::ContactConstraint() {}

float ContactConstraint::solve_(std::vector<Net*>& nets) const
{
    float deltaA, deltaB, deltaA1, deltaA2, deltaB1, deltaB2, meanDelta;
    Eigen::Vector3f contactPointA;
    Eigen::Vector3f contactPointB;
    Eigen::Vector3f shiftVec;
    float currDist;

    meanDelta = 0;    
    for(const Contact& c : contacts_)
    {
        Net* netA = nets[c.netIdxA];
        Net* netB = nets[c.netIdxB];
        
        std::tie(contactPointA, contactPointB) = c.getContactPoints(nets);

        shiftVec = contactPointB - contactPointA;
        currDist = shiftVec.norm();

        if(currDist == 0)
            continue;

        deltaA = currDist * c.omegaB / (c.omegaA + c.omegaB);
        deltaB = currDist * c.omegaA / (c.omegaA + c.omegaB);
        deltaA1 = deltaA * (1 - c.alpha) * c.omegaA;
        deltaA2 = deltaA * (c.alpha)     * c.omegaA;
        deltaB1 = deltaB * (1 - c.beta)  * c.omegaB;
        deltaB2 = deltaB * (c.beta)      * c.omegaB;
        shiftVec /= currDist;

        netA->nodePos(netA->edge(c.edgeIdxA)[0]) += deltaA1 * shiftVec;
        netA->nodePos(netA->edge(c.edgeIdxA)[1]) += deltaA2 * shiftVec;
        netB->nodePos(netB->edge(c.edgeIdxB)[0]) -= deltaB1 * shiftVec;
        netB->nodePos(netB->edge(c.edgeIdxB)[1]) -= deltaB2 * shiftVec;

        meanDelta += (std::pow(deltaA1, 2) +
                      std::pow(deltaA2, 2) +
                      std::pow(deltaB1, 2) +
                      std::pow(deltaB2, 2));
    }

    return meanDelta;
}

int ContactConstraint::nConstraints(std::vector<Net*>& nets) const
{
    return contacts_.size() * 4;
}

bool ContactConstraint::addContact(const Contact& contact, std::vector<Net*>& nets,
                                   float maxEdgeEdgeDistance,
                                   float minContactNodeDistance,
                                   float minContactContactDistance)
{
    if(contact.distance > maxEdgeEdgeDistance ||
       std::abs(contact.alpha - 0.5) > (0.5 - minContactNodeDistance) ||
       std::abs(contact.beta  - 0.5) > (0.5 - minContactNodeDistance))
        return false;

    Eigen::Vector3f contactPointA, otherContactPointA;
    Eigen::Vector3f contactPointB, otherContactPointB;
    Eigen::Vector3f meanContactPoint, otherMeanContactPoint;

    std::tie(contactPointA, contactPointB) = contact.getContactPoints(nets);
    meanContactPoint = (contactPointA + contactPointB) / 2.0f;
    
    for(const Contact& otherContact : contacts_)
    {
        std::tie(otherContactPointA, otherContactPointB) = otherContact.getContactPoints(nets);
        otherMeanContactPoint = (otherContactPointA + otherContactPointB) / 2.0f;

        if((meanContactPoint - otherMeanContactPoint).norm() < minContactContactDistance)
            return false;
    }

    contacts_.push_back(contact);
    return true;
}

const std::vector<ContactConstraint::Contact>& ContactConstraint::getContacts() const
{
    return contacts_;
}

void ContactConstraint::clearContacts()
{
    contacts_.clear();
}