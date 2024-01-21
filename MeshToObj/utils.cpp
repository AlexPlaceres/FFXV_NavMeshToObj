#include "utils.h"

NavMeshFile::NavMeshFile(char* filebuffer)
{
    this->pInternalData = (InternalData*)(filebuffer + 0x180);
    this->pAutoGenDatFile = (AutoGenerationDataFile*)(&this->pInternalData[1]);
    this->pCellInfoFiles = (CellInfoFile*)(&this->pAutoGenDatFile[1]);
    this->pLayerMeshFiles = (LayerMeshFile*)(&this->pCellInfoFiles[this->pAutoGenDatFile->cellInfoCount]);
    this->pOffsetMap = (uint32_t*)(&this->pLayerMeshFiles[this->pAutoGenDatFile->layerMeshCount]);
}
