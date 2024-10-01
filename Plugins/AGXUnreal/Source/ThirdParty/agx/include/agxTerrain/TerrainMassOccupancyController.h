/*
Copyright 2007-2024. Algoryx Simulation AB.

All AGX source code, intellectual property, documentation, sample code,
tutorials, scene files and technical white papers, are copyrighted, proprietary
and confidential material of Algoryx Simulation AB. You may not download, read,
store, distribute, publish, copy or otherwise disseminate, use or expose this
material unless having a written signed agreement with Algoryx Simulation AB, or having been
advised so by Algoryx Simulation AB for a time limited evaluation, or having purchased a
valid commercial license from Algoryx Simulation AB.

Algoryx Simulation AB disclaims all responsibilities for loss or damage caused
from using this software, unless otherwise stated in written agreements with
Algoryx Simulation AB.
*/

#pragma once

#include <agx/config/AGX_USE_AGXTERRAIN.h>
#include <agxTerrain/export.h>
#include <agxTerrain/TerrainDataAtlas.h>
#include <agx/Referenced.h>
#include <agx/Vec3.h>

namespace agxTerrain
{
  class Terrain;

  AGX_DECLARE_POINTER_TYPES(TerrainMassOccupancyController);
  AGX_DECLARE_VECTOR_TYPES(TerrainMassOccupancyController);

  /**
  Class that handles and sanity checks the mass logic in the terrain. Every mass operation on the terrain grid should go through here.
  */
  class AGXTERRAIN_EXPORT TerrainMassOccupancyController : public agx::Referenced
  {
  public:
    /**
    Construct given terrain instance.
    */
    TerrainMassOccupancyController(Terrain* terrain);

    /**
    \internal

    Default constructor used in serialization.
    */
    TerrainMassOccupancyController();

    /*
    Set fluid mass in a voxel given an index
    \param voxelIndex - The specified voxel index
    \param fluidMass - the amount of fluid mass to set
    */
    void setFluidOccupancy(const agx::Vec3i& voxelIndex, float fluidMass);

    /**
    Add fluid mass in a voxel and return the rest mass if the voxel was filled
    \param voxelIndex - The voxel index
    \param fluidMass - the amount of fluid mass to add
    \return the rest fluid mass that could not fit in the voxel
    */
    float addFluidOccupancy(const agx::Vec3i& voxelIndex, float fluidMass);

    /**
    Add fluid mass in a voxel and return the rest mass if the voxel was filled.
    \param voxelIndex - The voxel index.
    \param fluidMass - the amount of fluid mass to remove.
    \return the rest fluid mass that could not be removed from the voxel.
    */
    float removeFluidOccupancy(const agx::Vec3i& voxelIndex, float fluidMass);

    /**
    Get fluid mass in a voxel given an index
    \param voxelIndex - The specified voxel index
    \return the fluid mass in the voxel
    */
    float getFluidOccupancy(const agx::Vec3i& voxelIndex) const;

    /**
    Add solid mass with a given compaction to a voxel and return the rest mass that could not fit.
    \note - The added solid mass will modify the voxel compaction given it's own compaction.
    \param voxelIndex - The specified voxel index
    \param solidOccupancy - The specified solid mass occupancy to add to a voxel
    \param compaction - The compaction of the mass to add
    \param occupancyLimit - The nominal occupancy limit in to fill in the voxel.
                            NOTE - cannot be higher than the voxel max nominal occupancy
    \return The rest mass that could not fit inside the voxel
    */
    float addSolidOccupancy(const agx::Vec3i& voxelIndex, float solidOccupancy, float compaction, float occupancyLimit = 1.0);

    /**
    Remove solid mass from a voxel and return the amount of mass that could not be removed, i.e the
    rest if the voxel was emptied.
    \param voxelIndex - The specified voxel index
    \param solidMass - The specified solid mass to remove from a voxel
    \return The rest mass that could not be removed from the voxel before it was emptied
    */
    float removeSolidOccupancy(const agx::Vec3i& voxelIndex, float solidMass);

    /**
    Get the solid mass from a voxel
    \param voxelIndex - The specified voxel index
    \return the solid mass in the voxel
    */
    float getSolidOccupancy(const agx::Vec3i& voxelIndex) const;

    /**
    Set compaction on a voxel
    \param voxelIndex - The specified voxel index
    \param compaction- The compaction to set
    \param scaleMass - True if the existing occupancy in the voxel should be scaled so that the
                       compacted solid occupancy is the same as before compaction was set. This
                       will effectively increase the total mass in the terrain.
    */
    void setCompaction(const agx::Vec3i& voxelIndex, float compaction, bool scaleMass=false);

    /**
    Get compaction in a voxel
    \param voxelIndex - The specified voxel index
    \return the compaction in the voxel
    */
    float getCompaction(const agx::Vec3i& voxelIndex) const;

    float getOccupancyModifier(agx::Vec3i voxelIndex) const;

    /**
    Get particle mass in a voxel
    \param voxelIndex - The specified voxel index
    \return the particle mass in a voxel
    */
    float getParticleOccupancy(const agx::Vec3i& voxelIndex) const;

    /**
    Set particle mass in a voxel
    \param voxelIndex - The specified voxel index
    \param particleMass - The particle mass to set
    */
    void setParticleOccupancy(const agx::Vec3i& voxelIndex, float particleMass);

    /**
    Get the compacted mass in a voxel. This is the solid mass divided by the compaction in the specified voxel.
    This will normalize the solid mass to nominal occupancy, i.e how much space the compacted solid mass takes in the voxel (0.0 - 1.0).
    \param voxelIndex - The specified voxel index
    \return the compacted solid mass in the voxel
    */
    float getCompactedSolidOccupancy(const agx::Vec3i& voxelIndex) const;

    /**
    Add fluid mass column wise on the surface of the terrain
    \param terrainIndex - The specified 2D voxel index
    \param mass - Amount of mass to be added
    \return the fluid mass that was added to the column.
    */
    float addFluidOccupancyToColumn(const agx::Vec2i& terrainIndex, float mass);

    /**
    Remove fluid mass column wise from the surface of the terrain.
    \param terrainIndex - The specified 2D voxel index.
    \param mass - Amount of mass to be removed.
    \return the amount of fluid mass that was removed from the column.
    */
    float removeFluidOccupancyFromColumn(const agx::Vec2i& terrainIndex, float mass);

    /**
    Add solid mass with given compaction column wise on the surface of the terrain and return the amount of mass
    that was successfully added.
    \param terrainIndex - The specified 2D voxel index
    \param occupancy - the solid mass occupancy to add
    \param compaction - the compaction of the mass
    \return the amount of mass that was successfully added.
    */
    float addSolidOccupancyToColumn(const agx::Vec2i& terrainIndex, float occupancy, float compaction);

    /**
    Add solid mass with given compaction column wise on the surface of the terrain and return the amount of mass
    that was successfully added. Unless needed, prefer to use the addSolidMassToColumn that uses just (x,y) to index
    the voxel instead of this version that has (x,y,z).
    \param surfaceVoxelIndex - Current surface voxel in column
    \param occupancy - the solid mass occupancy to add
    \param compaction - the compaction of the mass
    \return the amount of mass that was successfully added.
    */
    float addSolidOccupancyToColumn(const agx::Vec3i& surfaceVoxelIndex, float occupancy, float compaction);

    /**
    Remove solid mass column wise from surface of the terrain and return the amount of mass
    that was successfully added.
    \param terrainIndex - The specified 2D voxel index
    \param mass - the solid mass to remove
    \return the amount of mass that was successfully removed
    */
    float removeSolidOccupancyFromColumn(const agx::Vec2i& terrainIndex, float mass);

    /**
    Remove solid mass column wise from surface of the terrain and return the amount of mass
    that was successfully added. Unless needed, prefer to use the removeSolidMassFromColumn
    that uses (x,y) to index the voxel instead of this version with (x,y,z).
    \param surfaceVoxelIndex - Current surface voxel index
    \param mass - the solid mass to remove
    \return the amount of mass that was successfully removed
    */
    float removeSolidOccupancyFromColumn(const agx::Vec3i& surfaceVoxelIndex, float mass);

    /**
    Dilate material in a column from a bottom index up to surface. This is used when the compaction data in the grid
    has been updated (eg. through compaction) and the compacted mass needs redistributed to account for the changed volume.
    \param bottomVoxel - The bottom voxel index where dilation will begin
    \param newSurfaceVoxel - Will give the new surface voxel index after all the mass has been dilated
    \return true if the dilation was successfully, false otherwise
    */
    bool dilateOccupancyInColumn(const agx::Vec3i& bottomVoxel, agx::Vec3i& newSurfaceVoxel);

    /**
    Converts compacted mass to regular non-compacted mass given the specified voxel compaction
    \param voxelIndex - The voxel index where the compaction value will be used for conversion
    \param compactedOccupancy - The compacted mass to convert
    \return The converted mass given the specified voxel compaction and compacted occupancy
    */
    float convertCompactedSolidOccupancyToNominal(const agx::Vec3i& voxelIndex, float compactedOccupancy) const;

    /**
    Checks if the total mass in the specified voxel exceeds the allowed nominal threshold
    \param voxelIndex - The specified voxel index to check
    \return True if the mass is over the nominal threshold, false otherwise
    */
    bool isTotalOccupancyOverNominalThreshold(const agx::Vec3i& voxelIndex) const;

    /**
    Verify that a mass column in the terrain grid is properly filled, i.e that all submerged voxels under the surface holds the maximum
    allowed mass given by the voxel compaction. There should not be any "holes" in the terrain.
    \note - If the check fails, it will trigger an agxVerify and crash.
    \param terrainIndex - The specified voxel index to check.
    \return true if the terrain solid column satisfies mass integrity conditions, false otherwise.
    */
    bool verifyOccupancyColumnIntegrity(const agx::Vec2i& terrainIndex ) const;

    /**
    Set the which voxel is the surface voxel and what the occupancy is in that voxel
    \param terrainIndex - Terrain index of the column.
    \param voxelZ - Z value of the surface voxel in the voxel grid structure.
    \param occupancy - Occupancy in the surface voxel.
    */
    void setGridColumnOccupancy(const agx::Vec2i terrainIndex, int voxelZ, float occupancy);

    /**
    Set occupancy in surface voxel
    \param terrainIndex - Terrain index of the column.
    \param occupancy - Occupancy in the surface voxel.
    */
    void setOccupancyInSurfaceVoxel(const agx::Vec2i terrainIndex, float occupancy);

    /**
    Get Z value of the surface voxel in the voxel grid structure.
    \param terrainIndex - Terrain index of the column.
    \return Z value of the surface voxel in the voxel grid structure.
    */
    int getSurfaceHeightIndex(const agx::Vec2i terrainIndex) const;

    /**
    Get the total mass in the terrain of a specific mass type.
    \param massType - The mass type.
    \return Total mass of that specific mass type in terrain.
    */
    agx::Real getOccupancyInTerrain( TerrainDataAtlas::MassType massType ) const;


    std::pair<int, float> getVoxelZAndOccupancyFromLocalHeight(agx::Real height, agx::Real voxelSize);

  public:
    DOXYGEN_START_INTERNAL_BLOCK()
    AGXTERRAIN_STORE_RESTORE_INTERFACE;
    DOXYGEN_END_INTERNAL_BLOCK()

  protected:

    virtual ~TerrainMassOccupancyController();

    TerrainDataAtlas* getData() const;

  private:
    Terrain*                          m_terrain;
    agx::HashTable<agx::Vec3i, float> m_occupancyModifiers;
  };
}
