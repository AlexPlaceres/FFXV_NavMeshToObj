#include <iostream>
#include <fstream>
#include <string>
#include <utils.h>

std::ifstream inputFile;
std::ofstream outputFile;

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

    int vertexIndex = 0;


    outputFile.open("../../MeshToObj/Output/output.obj", std::ios::out);



    for (int i = 0; i < meshFile.pAutoGenDatFile->layerMeshCount; i++)
    {
        
        pNavMesh = (NavMesh*)(fileBuffer + meshFile.pOffsetMap[i]);
        printf("Mesh %u: Identifiers = %.4s, %.4s\n", i, pNavMesh->magic, pNavMesh->meshHeader.magic);
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
        outputFile << "l " << vertexIndex + 1 << " " << vertexIndex + 2 << "\n";
        outputFile << "l " << vertexIndex + 2 << " " << vertexIndex + 3 << "\n";
        outputFile << "l " << vertexIndex + 3 << " " << vertexIndex + 4 << "\n";
        outputFile << "l " << vertexIndex + 4 << " " << vertexIndex + 1 << "\n";

        // Edges connecting the upper four Vertices
        outputFile << "l " << vertexIndex + 5 << " " << vertexIndex + 6 << "\n";
        outputFile << "l " << vertexIndex + 6 << " " << vertexIndex + 7 << "\n";
        outputFile << "l " << vertexIndex + 7 << " " << vertexIndex + 8 << "\n";
        outputFile << "l " << vertexIndex + 8 << " " << vertexIndex + 5 << "\n";


        // Vertical Edges connecting the lower four with the upper four
        outputFile << "l " << vertexIndex + 1 << " " << vertexIndex + 5 << "\n";
        outputFile << "l " << vertexIndex + 2 << " " << vertexIndex + 6 << "\n";
        outputFile << "l " << vertexIndex + 3 << " " << vertexIndex + 7 << "\n";
        outputFile << "l " << vertexIndex + 4 << " " << vertexIndex + 8 << "\n";


        vertexIndex += 8;

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
                    outputFile << "l " << pPolys[i].verts[j] + vertexIndex + 1 << " " << pPolys[i].verts[0] + vertexIndex + 1 << "\n";
                }
                else
                {
                    // Connects a Polygon's vertex n with its vertex n+1
                    // Because .obj Vertex Indices start from 1, we add 1 to the Polygon's Vertex index value
                    // We also add 8 to the value to account for the 8 Vertices we used to create the Bounding Box
                    // 1 + 8 = 9
                    outputFile << "l " << pPolys[i].verts[j] + vertexIndex + 1 << " " << pPolys[i].verts[j + 1] + vertexIndex + 1 << "\n";
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
                outputFile << " " << pPolys[i].verts[j] + vertexIndex + 1;
            }
            outputFile << "\n";
        }

        vertexIndex += pNavMesh->meshHeader.vertCount;

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
   
    return 0;
}

