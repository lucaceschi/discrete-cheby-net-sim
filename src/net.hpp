#ifndef NET_H
#define NET_H

#include <string>

#include <Eigen/Dense>
#include <GL/glew.h>


struct Net
{
    Net();

    Net(Eigen::Vector2i size,
        float edgeLength,
        Eigen::Vector3f center,
        Eigen::Vector3f xTangVec,
        Eigen::Vector3f yTangVec,
        GLubyte* color);

    inline int getNNodes() const     { return pos.cols(); }
    inline int getNEdges() const     { return edges.cols(); }
    inline int getNDiagonals() const { return diagonals.cols(); }

    inline Eigen::Block<Eigen::Matrix3Xf, 3, 1, true> nodePos(int idx) { return pos.col(idx); }
    inline Eigen::Block<Eigen::Array2Xi, 2, 1, true>  edge(int idx)    { return edges.col(idx); }
    inline Eigen::Block<Eigen::Array2Xi, 2, 1, true>  diag(int idx)    { return diagonals.col(idx); }

    void exportPly(std::string filename) const;

    Eigen::Matrix3Xf pos;
    Eigen::Array2Xi edges;
    Eigen::Array2Xi diagonals;
    Eigen::ArrayXf edgeLengths;
    GLubyte color[3];
};


#endif