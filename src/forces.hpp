#ifndef FORCES_HPP
#define FORCES_HPP

#include <Eigen/Dense>


class Force
{
public:
    virtual ~Force();
    virtual void applyForce(Eigen::Ref<Eigen::Vector3f> pos) const = 0;
};

class ConstantForce : public Force
{
public:
    ConstantForce(Eigen::Vector3f vec);
    ~ConstantForce();

    virtual void applyForce(Eigen::Ref<Eigen::Vector3f> pos) const;

    Eigen::Vector3f getVector();
    void setVector(Eigen::Vector3f vec);

private:
    Eigen::Vector3f vec_;
    
};


#endif