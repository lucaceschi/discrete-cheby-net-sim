#include "net.hpp"

#include <fstream>


Net::Net() {}

Net::Net(Eigen::Vector2i size,
         float edgeLength,
         Eigen::Vector3f center,
         Eigen::Vector3f xTangVec,
         Eigen::Vector3f yTangVec,
         GLubyte* renderColor)
    : pos(3, size[0] * size[1]),
      edges(2, 2 * size[0] * size[1] - size[0] - size[1]),
      diagonals(2, 2 * (size[1] - 1) * (size[0] - 1)),
      color{renderColor[0], renderColor[1], renderColor[2]}
{
    float width  = (float)(size[1] -  1) * edgeLength;
    float height = (float)(size[0] -  1) * edgeLength;
    xTangVec.normalize();
    yTangVec.normalize();
    Eigen::Vector3f gridOrigin = center - (width/2.0 * xTangVec) - (height/2.0 * yTangVec);

    for(int n = 0, e = 0, d = 0; n < (size[0] * size[1]); n++)
    {
        int row = n / size[1];
        int col = n - (row * size[1]);

        pos.col(n) = gridOrigin + (row * edgeLength) * yTangVec + (col * edgeLength) * xTangVec;

        if(col != size[1] - 1)
            edges.col(e++) = Eigen::Array2i{n, n+1};
        if(row != size[0] - 1)
            edges.col(e++) = Eigen::Array2i{n, n+size[1]};

        if(row != size[0] - 1 && col != size[1] - 1)
            diagonals.col(d++) = Eigen::Array2i(n, n+size[1]+1);
        if(row != 0 && col != size[1] - 1)
            diagonals.col(d++) = Eigen::Array2i(n, n-size[1]+1);
    }
}

void Net::exportPly(std::string filename) const
{
    Eigen::IOFormat eigenFmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ");
    std::ofstream f(filename, std::ios::out | std::ios::trunc);

    // Header
    f << "ply" << std::endl << "format ascii 1.0" << std::endl;
    f << "element vertex " << getNNodes() << std::endl;
    f << "property double x" << std::endl;
    f << "property double y" << std::endl;
    f << "property double z" << std::endl;
    f << "element edge " << getNEdges() << std::endl;
    f << "property int vertex1" << std::endl;
    f << "property int vertex2" << std::endl;
    f << "end_header" << std::endl;

    // Vertex list
    for(int n = 0; n < getNNodes(); n++)
        f << pos.col(n).format(eigenFmt) << std::endl;

    // Edge list
    for(int e = 0; e < getNEdges(); e++)
        f << edges.col(e).format(eigenFmt) << std::endl;

    f.close();
}
