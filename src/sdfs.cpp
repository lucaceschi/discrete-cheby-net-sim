#include "sdfs.hpp"

#include <Eigen/Dense>


#define SDF_TORUS_TX 0.5
#define SDF_TORUS_TY 0.2

float SDFs::Torus::sdf(Eigen::Vector3f pos)
{   
    Eigen::Vector2f posXZ(pos.x(), pos.z());
    Eigen::Vector2f q(posXZ.norm() - SDF_TORUS_TX, pos.y());

    return q.norm() - SDF_TORUS_TY;
}

Eigen::Vector3f SDFs::Torus::dsdf(Eigen::Vector3f pos)
{
    Eigen::Vector2f posXZ(pos.x(), pos.z());
    float posXZNorm = posXZ.norm();
    Eigen::Vector2f q(posXZNorm - SDF_TORUS_TX, pos.y());
    float qNorm = q.norm();


    return Eigen::Vector3f{
        (pos.x() * q.x()) / (qNorm * posXZNorm),
        pos.y() / qNorm,
        (pos.z() * q.x()) / (qNorm * posXZNorm),
    };
}


#define SDF_NUT_RADIUS 0.5

float SDFs::Nut::sdf(Eigen::Vector3f pos)
{      
    float smooth = 0.1;
    
    float sph1 = (pos - Eigen::Vector3f{-0.4, 0, 0}).norm() - SDF_NUT_RADIUS;
    float sph2 = (pos - Eigen::Vector3f{+0.4, 0, 0}).norm() - SDF_NUT_RADIUS;

    float h = std::max(smooth - std::abs(sph1 - sph2), 0.0f);
    return std::min(sph1, sph2) - (0.25 * h * h / smooth);
}

Eigen::Vector3f SDFs::Nut::dsdf(Eigen::Vector3f pos)
{
    float smooth = 0.1;
    
    Eigen::Vector3f sph1 = (pos - Eigen::Vector3f{-0.4, 0, 0});
    Eigen::Vector3f sph2 = (pos - Eigen::Vector3f{+0.4, 0, 0});
    float sph1n = sph1.norm();
    float sph2n = sph2.norm();
    sph1 /= sph1n;
    sph2 /= sph2n;

    float h = std::max(smooth - std::abs(sph1n - sph2n), 0.0f);
    float n = 0.50 * h / smooth;

    return sph1 + (sph1n < sph2n? n : 1.0-n) * (sph2 - sph1);
}