#include "forces.hpp"

#include "converter_defs.hpp"


UnaryForce::~UnaryForce() {}


ConstantForce::ConstantForce(Eigen::Vector3f vec)
    : vec(vec)
{}

ConstantForce::~ConstantForce() {}

void ConstantForce::applyForce(std::vector<Net*>& nets, int netIndex, int nodeIndex) const
{
    nets[netIndex]->nodePos(nodeIndex) += vec;
}


DiscreteSDFFittingForce::DiscreteSDFFittingForce(std::vector<Net*>& nets,
                                                       FloatGrid::Ptr sdfGrid,
                                                       float smoothstepNearBound,
                                                       float smoothstepFarBound,
                                                       Eigen::Vector3f worldTranslationVec)
    : sdfGrid_(sdfGrid),
      transform_(sdfGrid->transformPtr()),
      sdfGridAccs_(nets.size()),
      gradGridAccs_(nets.size()),
      sdfGridSamplers_(nets.size()),
      gradGridSamplers_(nets.size())
{
    setWorldTranslationVec(worldTranslationVec);
    
    float voxelSize = sdfGrid->metaValue<float>(VDB_VOXEL_SIZE_METADATA_NAME);
    intBandWidth_ = sdfGrid->metaValue<float>(VDB_INTERIOR_BANDWIDTH_METADATA_NAME) * voxelSize;
    extBandWidth_ = sdfGrid->metaValue<float>(VDB_EXTERIOR_BANDWIDTH_METADATA_NAME) * voxelSize;
    maxFarBound_ = extBandWidth_;

    setNearBound(smoothstepNearBound);
    setFarBound(smoothstepFarBound);

    // compute gradient grid
    FloatGradient grad(*sdfGrid_);
    gradGrid_ = grad.process();
  
    // initialize grid accessors
    for(int netIdx = 0; netIdx < nets.size(); netIdx++)
    {
        Net* n = nets[netIdx];

        sdfGridAccs_[netIdx].reserve(n->getNNodes());
        gradGridAccs_[netIdx].reserve(n->getNNodes());
        sdfGridSamplers_[netIdx].reserve(n->getNNodes());
        for(int nodeIdx = 0; nodeIdx < n->getNNodes(); nodeIdx++)
        {
            sdfGridAccs_[netIdx].emplace_back(sdfGrid_->getConstAccessor());
            gradGridAccs_[netIdx].emplace_back(gradGrid_->getConstAccessor());
            sdfGridSamplers_[netIdx].emplace_back(sdfGridAccs_[netIdx].back(), sdfGrid_->transform());
            gradGridSamplers_[netIdx].emplace_back(gradGridAccs_[netIdx].back(), sdfGrid_->transform());
        }
    }
}

DiscreteSDFFittingForce::~DiscreteSDFFittingForce() {}

void DiscreteSDFFittingForce::applyForce(std::vector<Net*>& nets, int netIndex, int nodeIndex) const
{
    Vec3f worldPos = Vec3f(nets[netIndex]->nodePos(nodeIndex).data());
    float distFromSurface = sdfGridSamplers_[netIndex][nodeIndex].wsSample(worldPos);

    Eigen::Vector3f attractionVec = Eigen::Vector3f::Zero();
    if(std::abs(distFromSurface) < extBandWidth_)
    {
        attractionVec = -Eigen::Vector3f(gradGridSamplers_[netIndex][nodeIndex].wsSample(worldPos).asPointer());
        attractionVec *= std::min(worldTranslationVecNorm_, distFromSurface);
    }

    float s = smoothstep(nearBound_, farBound_, distFromSurface);
    nets[netIndex]->nodePos(nodeIndex) += (worldTranslationVec_ * s) + (attractionVec * (1.0 - s));
}

float DiscreteSDFFittingForce::getMaxFarBound() const
{
    return maxFarBound_;
}

float DiscreteSDFFittingForce::getNearBound() const
{
    return nearBound_;
}

float DiscreteSDFFittingForce::getFarBound() const
{
    return farBound_;
}

void DiscreteSDFFittingForce::setNearBound(float smoothstepNearBound)
{
    nearBound_ = smoothstepNearBound;
    if(nearBound_ < 0)
        nearBound_ = 0;
    else if(nearBound_ > maxFarBound_)
        nearBound_ = maxFarBound_;
}

void DiscreteSDFFittingForce::setFarBound(float smoothstepFarBound)
{
    farBound_ = smoothstepFarBound;
    if(farBound_ > maxFarBound_)
        farBound_ = maxFarBound_;
    else if(farBound_ < 0)
        farBound_ = 0;
}

Eigen::Vector3f DiscreteSDFFittingForce::getWorldTranslationVec() const
{
    return worldTranslationVec_;
}

void DiscreteSDFFittingForce::setWorldTranslationVec(Eigen::Vector3f worldTranslationVec)
{
    worldTranslationVec_ = worldTranslationVec;
    worldTranslationVecNorm_ = worldTranslationVec_.norm();
}

float DiscreteSDFFittingForce::clamp(float x, float lowerlimit, float upperlimit) const
{
    if (x < lowerlimit) return lowerlimit;
    if (x > upperlimit) return upperlimit;
    return x;
}

float DiscreteSDFFittingForce::smoothstep(float edge0, float edge1, float x) const
{
    x = clamp((x - edge0) / (edge1 - edge0));
    return x * x * (3.0f - 2.0f * x);
}