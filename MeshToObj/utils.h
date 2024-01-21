#pragma once
#include <iostream>

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

