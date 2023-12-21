#include <iostream>
#include <vector>

#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/GridOperators.h>


class Vertex;
class Face;
struct MeshUsedTypes : public vcg::UsedTypes<vcg::Use<Vertex>::AsVertexType,
                                             vcg::Use<Face>::AsFaceType> {};

class Vertex : public vcg::Vertex<MeshUsedTypes,
                                  vcg::vertex::Coord3d,
                                  vcg::vertex::BitFlags> {};

class Face : public vcg::Face<MeshUsedTypes,
                              vcg::face::VertexRef> {};

class Mesh : public vcg::tri::TriMesh<std::vector<Vertex>,
                                      std::vector<Face>> {};


int main(int argc, char* argv[])
{
    char *meshPath, *outPath;
    float voxelSize;
    float extBandWidth, intBandWidth;
    {
        if(argc != 6)
        {
            std::cout << "mesh2vdb: generates a discrete SDF of a closed triangular mesh for the simulator." << std::endl
                      << "Usage: ./mesh2vdb path_to_mesh voxel_size ext_band_width int_band_width path_to_output" << std::endl
                      << "\t path_to_mesh: path to the mesh to be converted" << std::endl
                      << "\t voxel_size: size of VDB voxels in world space units" << std::endl
                      << "\t ext_band_width: the exterior narrow-band width in voxel units" << std::endl
                      << "\t int_band_width: the interior narrow-band width in voxel units" << std::endl
                      << "\t path_to_output: path to the vdb output" << std::endl;
            
            return EXIT_FAILURE;
        }

        meshPath = argv[1];

        try { voxelSize = std::stof(argv[2]); }
        catch(std::exception &e) { std::cerr << "An error occured parsing the voxel size" << std::endl; return EXIT_FAILURE;}
        
        try { extBandWidth = std::stof(argv[3]); }
        catch(std::exception &e) { std::cerr << "An error occured parsing the exterior narrow-band width" << std::endl; return EXIT_FAILURE;}

        try { intBandWidth = std::stof(argv[4]); }
        catch(std::exception &e) { std::cerr << "An error occured parsing the interior narrow-band width" << std::endl; return EXIT_FAILURE;}

        outPath = argv[5];
    }

    // - - - - - -

    Mesh mesh;
    {
        std::cout << "Importing mesh..." << std::endl;
        
        using Importer = vcg::tri::io::Importer<Mesh>;
        int loadErr, loadMask;

        loadErr = Importer::Open(mesh, meshPath, loadMask);
        if(loadErr != 0)
        {
            std::cerr << "Error loading mesh \"" << meshPath << "\": " << Importer::ErrorMsg(loadErr) << std::endl;
            return EXIT_FAILURE;
        }

        vcg::tri::UpdateFlags<Mesh>::VertexBorderFromNone(mesh);
        if(vcg::tri::UpdateSelection<Mesh>::VertexFromBorderFlag(mesh) != 0)
        {
            std::cerr << "The mesh \"" << meshPath << "\" is not closed" << std::endl;
            return EXIT_FAILURE;
        }
    }

    // - - - - - -

    openvdb::FloatGrid::Ptr sdfGrid;
    {
        std::cout << "Computing signed distance field..." << std::endl;
        
        openvdb::initialize();
        using Vec3s = openvdb::math::Vec3s;
        using Vec3I = openvdb::Vec3I;
        using Vec4I = openvdb::Vec4I;
        using Transform = openvdb::math::Transform;

        std::vector<Vec3s> meshVerts;
        std::vector<Vec3I> meshFaceIs;
        std::vector<Vec4I> emptyVecOfVec4I = std::vector<Vec4I>();

        meshVerts.reserve(mesh.VN());
        for(const Vertex& v : mesh.vert)
            meshVerts.emplace_back(v.cP().X(),
                                   v.cP().Y(),
                                   v.cP().Z());

        meshFaceIs.reserve(mesh.FN());
        for(const Face& f : mesh.face)
            meshFaceIs.emplace_back(vcg::tri::Index(mesh, f.cV(0)),
                                    vcg::tri::Index(mesh, f.cV(1)),
                                    vcg::tri::Index(mesh, f.cV(2)));

        Transform::Ptr transform = Transform::createLinearTransform(voxelSize);
        sdfGrid = openvdb::tools::meshToSignedDistanceField<openvdb::FloatGrid>(*transform,
                                                                                 meshVerts,
                                                                                 meshFaceIs,
                                                                                 emptyVecOfVec4I,
                                                                                 extBandWidth,
                                                                                 intBandWidth);
    }

    // - - - - - -

    {
        std::cout << "Exporting VDB..." << std::endl;
        
        sdfGrid->insertMeta("extBandWidthWorld", openvdb::FloatMetadata(extBandWidth));
        sdfGrid->insertMeta("intBandWidthWorld", openvdb::FloatMetadata(intBandWidth));
        
        openvdb::io::File vdbFile(outPath);
        vdbFile.write({sdfGrid});
        vdbFile.close();
    }

    // - - - - - -

    std::cout << "Bye" << std::endl;

    return EXIT_SUCCESS;
}