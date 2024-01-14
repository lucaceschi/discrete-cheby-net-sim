#ifndef SDFS_HPP
#define SDFS_HPP

#include <Eigen/Dense>


namespace SDFs
{
    namespace Torus
    {
        float sdf(Eigen::Vector3f pos);
        Eigen::Vector3f dsdf(Eigen::Vector3f pos);
    }
    
    namespace Peanut
    {
        float sdf(Eigen::Vector3f pos);
        Eigen::Vector3f dsdf(Eigen::Vector3f pos);
    }
}


#endif