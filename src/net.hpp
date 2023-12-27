#ifndef NET_H
#define NET_H

#include <string>

#include <Eigen/Dense>
#include <GL/glew.h>


struct Net
{
    struct Quad
    {
        Quad() {}
        
        Quad(int node1Index, int node2Index, int node3Index, int node4Index)
            : node1Index(node1Index),
              node2Index(node2Index),
              node3Index(node3Index),
              node4Index(node4Index)
        {}
        
        int node1Index;
        int node2Index;
        int node3Index;
        int node4Index;
    };
    
    Net(Eigen::Matrix3Xf& nodePos,
        Eigen::Array2Xi& edges,
        Eigen::ArrayXf& edgeLengths,
        Eigen::Array2Xi& diagonals,
        std::vector<Quad>& quads,
        GLubyte* renderColor);

    Net(Eigen::Vector2i size,
        float edgeLength,
        Eigen::Vector3f center,
        Eigen::Vector3f xTangVec,
        Eigen::Vector3f yTangVec,
        GLubyte* renderColor);

    Net();

    inline int getNNodes() const     { return pos.cols(); }
    inline int getNEdges() const     { return edges.cols(); }
    inline int getNDiagonals() const { return diagonals.cols(); }

    inline Eigen::Block<Eigen::Matrix3Xf, 3, 1, true> nodePos(int idx) { return pos.col(idx); }
    inline Eigen::Block<Eigen::Array2Xi, 2, 1, true>  edge(int idx)    { return edges.col(idx); }
    inline Eigen::Block<Eigen::Array2Xi, 2, 1, true>  diag(int idx)    { return diagonals.col(idx); }

    void exportPly(std::string filename) const;

    Eigen::Matrix3Xf pos;
    Eigen::Array2Xi edges;
    Eigen::ArrayXf edgeLengths;
    Eigen::Array2Xi diagonals;
    std::vector<Quad> quads;
    GLubyte renderColor[3];
};


#endif