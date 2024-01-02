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


DiscreteSDFAttractionForce::DiscreteSDFAttractionForce(std::vector<Net*>& nets,
                                                       FloatGrid::Ptr sdfGrid,
                                                       float smoothstepNearBound,
                                                       float smoothstepFarBound,
                                                       Eigen::Vector3f worldTranslationVec)
    : sdfGrid_(sdfGrid),
      transform_(sdfGrid->transformPtr()),
      sdfGridAccs_(nets.size()),
      gradGridAccs_(nets.size())
{
    setWorldTranslationVec(worldTranslationVec);
    
    float voxelSize = sdfGrid->metaValue<float>(VDB_VOXEL_SIZE_METADATA_NAME);
    intBandWidth_ = sdfGrid->metaValue<float>(VDB_INTERIOR_BANDWIDTH_METADATA_NAME) * voxelSize;
    extBandWidth_ = sdfGrid->metaValue<float>(VDB_EXTERIOR_BANDWIDTH_METADATA_NAME) * voxelSize;

    minNearBound_ = voxelSize / 2.0;
    setNearBound(smoothstepNearBound);

    maxFarBound_ = extBandWidth_ - minNearBound_;
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
        for(int nodeIdx = 0; nodeIdx < n->getNNodes(); nodeIdx++)
        {
            sdfGridAccs_[netIdx].emplace_back(sdfGrid_->getConstAccessor());
            gradGridAccs_[netIdx].emplace_back(gradGrid_->getConstAccessor());
        }
    }
}

DiscreteSDFAttractionForce::~DiscreteSDFAttractionForce() {}

void DiscreteSDFAttractionForce::applyForce(std::vector<Net*>& nets, int netIndex, int nodeIndex) const
{
    Coord c = transform_->worldToIndexCellCentered(Vec3d(nets[netIndex]->nodePos(nodeIndex).data()));
    float distFromSurface = sdfGridAccs_[netIndex][nodeIndex].getValue(c);

    if(std::abs(distFromSurface) < minNearBound_)
        return;

    Eigen::Vector3f attractionVec = Eigen::Vector3f::Zero();
    if(std::abs(distFromSurface) > minNearBound_ && std::abs(distFromSurface) < maxFarBound_)
    {
        attractionVec = -Eigen::Vector3f(gradGridAccs_[netIndex][nodeIndex].getValue(c).asPointer());
        attractionVec *= std::min(worldTranslationVecNorm_, distFromSurface);
    }

    float s = smoothstep(nearBound_, farBound_, distFromSurface);
    nets[netIndex]->nodePos(nodeIndex) += (worldTranslationVec_ * s) + (attractionVec * (1.0 - s));
}

float DiscreteSDFAttractionForce::getMinNearBound() const
{
    return minNearBound_;
}

float DiscreteSDFAttractionForce::getMaxFarBound() const
{
    return maxFarBound_;
}

float DiscreteSDFAttractionForce::getNearBound() const
{
    return nearBound_;
}

float DiscreteSDFAttractionForce::getFarBound() const
{
    return farBound_;
}

void DiscreteSDFAttractionForce::setNearBound(float smoothstepNearBound)
{
    nearBound_ = smoothstepNearBound;
    if(nearBound_ < minNearBound_)
        nearBound_ = minNearBound_;
}

void DiscreteSDFAttractionForce::setFarBound(float smoothstepFarBound)
{
    farBound_ = smoothstepFarBound;
    if(farBound_ > maxFarBound_)
        farBound_ = maxFarBound_;  
}

Eigen::Vector3f DiscreteSDFAttractionForce::getWorldTranslationVec() const
{
    return worldTranslationVec_;
}

void DiscreteSDFAttractionForce::setWorldTranslationVec(Eigen::Vector3f worldTranslationVec)
{
    worldTranslationVec_ = worldTranslationVec;
    worldTranslationVecNorm_ = worldTranslationVec_.norm();
}

float DiscreteSDFAttractionForce::clamp(float x, float lowerlimit, float upperlimit) const
{
    if (x < lowerlimit) return lowerlimit;
    if (x > upperlimit) return upperlimit;
    return x;
}

float DiscreteSDFAttractionForce::smoothstep(float edge0, float edge1, float x) const
{
    x = clamp((x - edge0) / (edge1 - edge0));
    return x * x * (3.0f - 2.0f * x);
}