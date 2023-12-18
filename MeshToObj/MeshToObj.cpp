#include <iostream>
#include <fstream>
#include <string>

std::ifstream inputFile;
std::ofstream outputFile;

struct InternalData
{
    uint32_t flags;
    uint32_t priority;
    uint32_t minPriority;
    uint32_t collisionDataTypeFilter;
    uint32_t xSize;
    uint32_t zSize;
    int xBegin;
    int zBegin;
};

struct AutoGenerationDataFile
{
    char magic[4];
    uint32_t version;
    uint32_t cellInfoCount;
    uint32_t layerMeshCount;
};

struct CellInfoFile
{
    uint64_t hash;
    int nextRemeshLayer;
    int nextMeshLayer;
    int xIndex;
    int zIndex; 
};

struct LayerMeshFile
{
    uint64_t meshHash;
    uint64_t remeshHash;
};

struct Vertex
{
    float points[3];
};

struct dtMeshHeader
{
    char magic[4]; // Identifier
    int version; // Tile data format version number
    int xIndex; // x-position of the tile in the dtNavMesh tile grid
    int zIndex; // y-position of the tile in the dtNavMesh tile grid
    int layer; // Layer of the tile in the dtNavMesh tile grid
    int tileIndex; 
    int polyCount;
    int vertCount;
    int maxLinkCount;
    int detailMeshCount;
    int detailVertCount;
    int detailTriCount;
    int bvNodeCount;
    int offMeshConnectCount;
    int offMeshBase;
    float walkableHeight;
    float walkableRadius;
    float walkableClimb;
    float bmin[3]; // Minimum bound vertex
    float bmax[3]; // Maximum bound vertex
    float bvQuantFactor;
};

struct dtPoly
{
    unsigned int firstLink; // Index of first dtLink, defines what edges the polygon shares with others
    uint16_t verts[7]; // Indices of up to 7 vertices the Polygon uses
    uint16_t neis[7]; // References to Neighboring Polygons and Flags for each Edge... with a really stupid name
    uint16_t flags; // User Defined Flags for the Polygon itself
    char vertCount; // Number of Vertices the Polygon has (max here is 7)
    char areaAndType; // Packed data representing the Area ID and the Polygon Type
};

struct NavMesh
{
    char magic[4];
    int version;
    uint32_t size;
    uint32_t reserved[5];
    dtMeshHeader meshHeader;


};



struct NavMeshFile
{
    InternalData* pInternalData;
    AutoGenerationDataFile* pAutoGenDatFile;
    CellInfoFile* pCellInfoFiles;
    LayerMeshFile* pLayerMeshFiles;
    uint32_t* pOffsetMap;

    NavMeshFile(char* filebuffer);
};

NavMeshFile::NavMeshFile(char* filebuffer)
{
    this->pInternalData = (InternalData*)(filebuffer + 0x180);
    this->pAutoGenDatFile = (AutoGenerationDataFile*)(&this->pInternalData[1]);
    this->pCellInfoFiles = (CellInfoFile*)(&this->pAutoGenDatFile[1]);
    this->pLayerMeshFiles = (LayerMeshFile*)(&this->pCellInfoFiles[this->pAutoGenDatFile->cellInfoCount]);
    this->pOffsetMap = (uint32_t*)(&this->pLayerMeshFiles[this->pAutoGenDatFile->layerMeshCount]);
}

void WriteObj(NavMeshFile meshFile, char* fileBuffer)
{
    NavMesh* pNavMesh = nullptr;
    Vertex* pVertices = nullptr;
    dtPoly* pPolys = nullptr;
    Vertex bBoxDelta;
    Vertex vertDelta;
    Vertex startingPoint;
    Vertex cellDelta;

    pNavMesh = (NavMesh*)(fileBuffer + meshFile.pOffsetMap[0]);

    startingPoint.points[0] = meshFile.pInternalData->xBegin * 16;
    startingPoint.points[1] = 0;
    startingPoint.points[2] = meshFile.pInternalData->zBegin * 16;

    int vertexThing = 0;

    outputFile.open("C:/Users/Mongo/source/repos/MeshToObj/MeshToObj/Output/output.obj", std::ios::out);



    for (int i = 0; i < meshFile.pAutoGenDatFile->layerMeshCount; i++) //meshFile.pAutoGenDatFile->layerMeshCount
    {
        
        pNavMesh = (NavMesh*)(fileBuffer + meshFile.pOffsetMap[i]);
        printf("Mesh %u: Identifiers = %.4s, %.4s\n", i, pNavMesh->magic, pNavMesh->meshHeader.magic);

        //outputFile << "g NavMesh_" << i << "\n";
        outputFile << "o BoundingBox_" << pNavMesh->meshHeader.xIndex << "_" << pNavMesh->meshHeader.zIndex << "\n";

        pVertices = (Vertex*) &pNavMesh[1];
        pPolys = (dtPoly*)&pVertices[pNavMesh->meshHeader.vertCount];

        bBoxDelta.points[0] = pNavMesh->meshHeader.bmax[0] - pNavMesh->meshHeader.bmin[0];
        bBoxDelta.points[1] = pNavMesh->meshHeader.bmax[1] - pNavMesh->meshHeader.bmin[1];
        bBoxDelta.points[2] = pNavMesh->meshHeader.bmax[2] - pNavMesh->meshHeader.bmin[2];

        cellDelta.points[0] = pNavMesh->meshHeader.bmin[0] - startingPoint.points[0];
        
        cellDelta.points[1] = pNavMesh->meshHeader.bmin[1] - startingPoint.points[1];
        cellDelta.points[2] = pNavMesh->meshHeader.bmin[2] - startingPoint.points[2];

        printf("Deltas: x = %f, y = %f, z = %f\n", cellDelta.points[0], cellDelta.points[1], cellDelta.points[2]);

        printf("Bounding Box Deltas: X = %f, Y = %f, Z = %f\n\n", bBoxDelta.points[0], bBoxDelta.points[1], bBoxDelta.points[2]);
        // Vertices are written to the .obj as v X Y Z
        // Lower four Vertices
        outputFile << "v " << cellDelta.points[0] << " " << cellDelta.points[1] << " " << cellDelta.points[2] << "\n";
        outputFile << "v " << std::fixed << bBoxDelta.points[0] + cellDelta.points[0] << " " << cellDelta.points[1] << " " << cellDelta.points[2] << "\n";
        outputFile << "v " << std::fixed << bBoxDelta.points[0] + cellDelta.points[0] << " " << cellDelta.points[1] << " " << std::fixed << bBoxDelta.points[2] + cellDelta.points[2] << "\n";
        outputFile << "v " << cellDelta.points[0] << " " << cellDelta.points[1] << " " << std::fixed << bBoxDelta.points[2] + cellDelta.points[2] << "\n";

        // Upper four Vertices
        outputFile << "v " << cellDelta.points[0] << " " << std::fixed << bBoxDelta.points[1] + cellDelta.points[1] << " " << cellDelta.points[2] << "\n";
        outputFile << "v " << std::fixed << bBoxDelta.points[0] + cellDelta.points[0] << " " << std::fixed << bBoxDelta.points[1] + cellDelta.points[1] << " " << cellDelta.points[2] << "\n";
        outputFile << "v " << std::fixed << bBoxDelta.points[0] + cellDelta.points[0] << " " << std::fixed << bBoxDelta.points[1] + cellDelta.points[1] << " " << std::fixed << bBoxDelta.points[2] + cellDelta.points[2] << "\n";
        outputFile << "v "  << cellDelta.points[0] << " " << std::fixed << bBoxDelta.points[1] + cellDelta.points[1] << " " << std::fixed << bBoxDelta.points[2] + cellDelta.points[2] << "\n";

        // Edges are written to the .obj as l pointA pointB
        // .obj Vertices start at index 1
        // Edges connecting the lower four Vertices

        outputFile << "l " << vertexThing + 1 << " " << vertexThing + 2 << "\n";
        outputFile << "l " << vertexThing + 2 << " " << vertexThing + 3 << "\n";
        outputFile << "l " << vertexThing + 3 << " " << vertexThing + 4 << "\n";
        outputFile << "l " << vertexThing + 4 << " " << vertexThing + 1 << "\n";

        /*outputFile << "l 1 2\n";
        outputFile << "l 2 3\n";
        outputFile << "l 3 4\n";
        outputFile << "l 4 1\n";*/

        // Edges connecting the upper four Vertices

        outputFile << "l " << vertexThing + 5 << " " << vertexThing + 6 << "\n";
        outputFile << "l " << vertexThing + 6 << " " << vertexThing + 7 << "\n";
        outputFile << "l " << vertexThing + 7 << " " << vertexThing + 8 << "\n";
        outputFile << "l " << vertexThing + 8 << " " << vertexThing + 5 << "\n";

        /*outputFile << "l 5 6\n";
        outputFile << "l 6 7\n";
        outputFile << "l 7 8\n";
        outputFile << "l 8 5\n";*/

        // Vertical Edges connecting the lower four with the upper four

        outputFile << "l " << vertexThing + 1 << " " << vertexThing + 5 << "\n";
        outputFile << "l " << vertexThing + 2 << " " << vertexThing + 6 << "\n";
        outputFile << "l " << vertexThing + 3 << " " << vertexThing + 7 << "\n";
        outputFile << "l " << vertexThing + 4 << " " << vertexThing + 8 << "\n";

        /*outputFile << "l 1 5\n";
        outputFile << "l 2 6\n";
        outputFile << "l 3 7\n";
        outputFile << "l 4 8\n";*/

        vertexThing += 8;

        //// The actual Navigation Mesh
        //outputFile << "o Mesh_" << i << "\n";

        // The vertices are normally in world coordinates, so we find the distance between them and the minimum bound
        // We then use that distance as the new coordinates so the mesh wont be 15,000 units away from the origin
        for (int i = 0; i < pNavMesh->meshHeader.vertCount; i++)
        {
            vertDelta.points[0] = pVertices[i].points[0] - pNavMesh->meshHeader.bmin[0] + cellDelta.points[0];
            vertDelta.points[1] = pVertices[i].points[1] - pNavMesh->meshHeader.bmin[1] + cellDelta.points[1];
            vertDelta.points[2] = pVertices[i].points[2] - pNavMesh->meshHeader.bmin[2] + cellDelta.points[2];
            outputFile << "v " << std::fixed << vertDelta.points[0] << " " << std::fixed << vertDelta.points[1] << " " << std::fixed << vertDelta.points[2] << "\n";
        }

        // Creates the Edges needed to make the polygons in the data
        for (int i = 0; i < pNavMesh->meshHeader.polyCount; i++)
        {
            for (int j = 0; j < pPolys[i].vertCount; j++)
            {
                if (j + 1 >= pPolys[i].vertCount)
                {
                    // The last vertex has no n+1 vertex to connect to, so it is connected to the first to finish the Polygon
                    // Because .obj Vertex Indices start from 1, we add 1 to the Polygon's Vertex index value
                    // We also add 8 to the value to account for the 8 Vertices we used to create the Bounding Box
                    // 1 + 8 = 9
                    outputFile << "l " << pPolys[i].verts[j] + vertexThing + 1 << " " << pPolys[i].verts[0] + vertexThing + 1 << "\n";
                }
                else
                {
                    // Connects a Polygon's vertex n with its vertex n+1
                    // Because .obj Vertex Indices start from 1, we add 1 to the Polygon's Vertex index value
                    // We also add 8 to the value to account for the 8 Vertices we used to create the Bounding Box
                    // 1 + 8 = 9
                    outputFile << "l " << pPolys[i].verts[j] + vertexThing + 1 << " " << pPolys[i].verts[j + 1] + vertexThing + 1 << "\n";
                }
            }
        }

        // Creates Faces using each Polygon's vertex indices
        for (int i = 0; i < pNavMesh->meshHeader.polyCount; i++)
        {
            outputFile << "f";
            for (int j = 0; j < pPolys[i].vertCount; j++)
            {
                // Because .obj Vertex Indices start from 1, we add 1 to the Polygon's Vertex index value
                // We also add 8 to the value to account for the 8 Vertices we used to create the Bounding Box
                // 1 + 8 = 9
                outputFile << " " << pPolys[i].verts[j] + vertexThing + 1;
            }
            outputFile << "\n";
        }

        vertexThing += pNavMesh->meshHeader.vertCount;

    }

    

    outputFile.close();
}


int main(int argc, char* args[])
{
    inputFile.open(args[1], std::ios::binary | std::ios::ate); // Open input file
    int fileSize = inputFile.tellg();   // Get input file size

    inputFile.seekg(0); // Return to start of file
    char* fileBuffer = new char[fileSize]; // Create buffer of the same size
    inputFile.read(fileBuffer, fileSize); // Read file into buffer

    inputFile.close();

    NavMeshFile navMeshFile(fileBuffer);
    
    WriteObj(navMeshFile, fileBuffer);
   
    
    //unsigned int meshAddress;
    //std::cout << "Type Mesh Address: "; // Any address in the file where mesh file information is found, starting with 0x6873654D ("Mesh")
    //std::cin >> meshAddress;
    //
    //try
    //{
    //    if (*(int*)(fileBuffer + meshAddress) != 1752393037) // Check if the int found at the current address matches the magic number 0x6873654D
    //    {
    //        throw(meshAddress);
    //    }
    //}
    //catch (unsigned int address)
    //{
    //    std::cout << "Invalid Mesh Address: " << address << std::endl;
    //    return 0;
    //}

    //// At Offset 0x8 of the Mesh File Information is the size of the mesh data itself
    //unsigned int meshDataSize = *(unsigned int*)(fileBuffer + meshAddress + 0x8);



    //dtMeshHeader dNav; // Detour Navigation Mesh Object
    //dNav = *(dtMeshHeader*)(fileBuffer + meshAddress + 0x20); // The Detour Navmesh Data is offset 0x20 bytes from the Mesh File Information
    
    NavMesh* pNavMesh = nullptr;
    pNavMesh = (NavMesh*)(fileBuffer + navMeshFile.pOffsetMap[0]);
    if (pNavMesh != nullptr)
    {
        printf("Quick check: %.4s, %.4s\n", pNavMesh->magic, pNavMesh->meshHeader.magic);
    }
    
    Vertex* pVertices = (Vertex*) &pNavMesh[1];
    printf("Vertex: X = %f, Y = %f, Z = %f\n", pVertices[0].points[0], pVertices[0].points[1], pVertices[0].points[2]);

    dtPoly* pPolys = (dtPoly*)&pVertices[pNavMesh->meshHeader.vertCount];
    printf("Polygon: First Link = %d, Vert Count = %hd\n", pPolys[0].firstLink, pPolys[0].vertCount);




    //std::cin.get();
    //std::cin.get();

    //char* utilPointer = (fileBuffer + meshAddress + 0x84); // Pointer used to iterate through the data since Polygon data begins where Vertex data ends
    //
    //// VERTICES
    //// New array to easily access Vertices later on
    //Vertex* vertices = new Vertex[dNav.vertCount];

    //// Vertices are always 0xC bytes, we can use a loop to populate vertices with the data straight from the file
    //for (int i = 0; i < dNav.vertCount; i++)
    //{
    //    vertices[i] = *(Vertex*)(utilPointer);
    //    utilPointer += 0xC;
    //}

    //// POLYGONS
    //// New array to easily access Polygon data later on
    //dtPoly* polys = new dtPoly[dNav.polyCount];

    //// Polygon data is always 0x24 bytes, so we use a loop to populate the array with the Polygon data from the file
    //for (int i = 0; i < dNav.polyCount; i++)
    //{
    //    polys[i] = *(dtPoly*)(utilPointer);
    //    utilPointer += 0x24; 
    //}

    //// Open Output .obj file
    //outputFile.open("C:/Users/Mongo/source/repos/MeshToObj/MeshToObj/Output/output.obj", std::ios::out);

    //// The Bounding Box around the Navigation Mesh
    //outputFile << "o BoundingBox" << "\n";

    //// We treat the minimum bound of the Bounding Box as the origin
    //Vertex bBoxDelta;
    //Vertex vertDelta;

    //// Calculate the difference between the minimum and maximum bounds and use those as coordinates for the maximum bound vertexes
    //bBoxDelta.points[0] = dNav.bmax[0] - dNav.bmin[0];
    //bBoxDelta.points[1] = dNav.bmax[1] - dNav.bmin[1];
    //bBoxDelta.points[2] = dNav.bmax[2] - dNav.bmin[2];

    //// Vertices are written to the .obj as v X Y Z
    //// Lower four Vertices
    //outputFile << "v 0.000000 0.000000 0.000000\n";
    //outputFile << "v " << std::fixed << bBoxDelta.points[0] << " 0.000000 0.000000\n";
    //outputFile << "v " << std::fixed << bBoxDelta.points[0] << " 0.000000 " << std::fixed << bBoxDelta.points[2] << "\n";
    //outputFile << "v 0.000000 0.000000 " << std::fixed << bBoxDelta.points[2] << "\n";

    //// Upper four Vertices
    //outputFile << "v 0.000000 " << std::fixed << bBoxDelta.points[1] << " 0.000000\n";
    //outputFile << "v " << std::fixed  << bBoxDelta.points[0] << " " << std::fixed << bBoxDelta.points[1] << " 0.000000\n";
    //outputFile << "v " << std::fixed << bBoxDelta.points[0] << " " << std::fixed << bBoxDelta.points[1] << " " << std::fixed << bBoxDelta.points[2] << "\n";
    //outputFile << "v 0.000000 " << std::fixed << bBoxDelta.points[1] << " " << std::fixed << bBoxDelta.points[2] << "\n";

    //// Edges are written to the .obj as l pointA pointB
    //// .obj Vertices start at index 1
    //// Edges connecting the lower four Vertices
    //outputFile << "l 1 2\n";
    //outputFile << "l 2 3\n";
    //outputFile << "l 3 4\n";
    //outputFile << "l 4 1\n";

    //// Edges connecting the upper four Vertices
    //outputFile << "l 5 6\n";
    //outputFile << "l 6 7\n";
    //outputFile << "l 7 8\n";
    //outputFile << "l 8 5\n";

    //// Vertical Edges connecting the lower four with the upper four
    //outputFile << "l 1 5\n";
    //outputFile << "l 2 6\n";
    //outputFile << "l 3 7\n";
    //outputFile << "l 4 8\n";

    //// The actual Navigation Mesh
    //outputFile << "o Mesh\n";

    // The vertices are normally in world coordinates, so we find the distance between them and the minimum bound
    // We then use that distance as the new coordinates so the mesh wont be 15,000 units away from the origin
    /*for (int i = 0; i < dNav.vertCount; i++)
    {
        vertDelta.points[0] = vertices[i].points[0] - dNav.bmin[0];
        vertDelta.points[1] = vertices[i].points[1] - dNav.bmin[1];
        vertDelta.points[2] = vertices[i].points[2] - dNav.bmin[2];
        outputFile << "v " << std::fixed << vertDelta.points[0] << " " << std::fixed << vertDelta.points[1] << " " << std::fixed << vertDelta.points[2] << "\n";
    }*/

    // Creates the Edges needed to make the polygons in the data
    //for (int i = 0; i < dNav.polyCount; i++)
    //{
    //    for (int j = 0; j < polys[i].vertCount; j++)
    //    {
    //        if (j + 1 >= polys[i].vertCount)
    //        {
    //            // The last vertex has no n+1 vertex to connect to, so it is connected to the first to finish the Polygon
    //            // Because .obj Vertex Indices start from 1, we add 1 to the Polygon's Vertex index value
    //            // We also add 8 to the value to account for the 8 Vertices we used to create the Bounding Box
    //            // 1 + 8 = 9
    //            outputFile << "l " << polys[i].verts[j] + 9 << " " << polys[i].verts[0] + 9 << "\n";
    //        }
    //        else
    //        {
    //            // Connects a Polygon's vertex n with its vertex n+1
    //            // Because .obj Vertex Indices start from 1, we add 1 to the Polygon's Vertex index value
    //            // We also add 8 to the value to account for the 8 Vertices we used to create the Bounding Box
    //            // 1 + 8 = 9
    //            outputFile << "l " << polys[i].verts[j] + 9 << " " << polys[i].verts[j + 1] + 9 << "\n";
    //        }
    //    }
    //}

    // Creates Faces using each Polygon's vertex indices
    //for (int i = 0; i < dNav.polyCount; i++)
    //{
    //    outputFile << "f";
    //    for (int j = 0; j < polys[i].vertCount; j++)
    //    {
    //        // Because .obj Vertex Indices start from 1, we add 1 to the Polygon's Vertex index value
    //        // We also add 8 to the value to account for the 8 Vertices we used to create the Bounding Box
    //        // 1 + 8 = 9
    //        outputFile << " " << polys[i].verts[j] + 9;
    //    }
    //    outputFile << "\n";
    //}


    //outputFile.close();

    return 0;
}

