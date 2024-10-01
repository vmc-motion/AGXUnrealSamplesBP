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
#include <agx/Referenced.h>
#include <agx/Vec3.h>
#include <agxTerrain/TerrainMaterial.h>
#include <agxTerrain/TerrainDataAtlas.h>


namespace agxTerrain
{
  class Terrain;
  class TerrainDataAtlas;
  DOXYGEN_START_INTERNAL_BLOCK()

  AGX_DECLARE_POINTER_TYPES(TerrainMaterialController);
  AGX_DECLARE_VECTOR_TYPES(TerrainMaterialController);

  typedef std::pair<agx::Vec3iVector, agx::UInt32Vector> TerrainMaterialsInVoxels;
  typedef agx::HashTable<TerrainMaterial*, agx::Vec3iVector> TerrainMaterialSort;


  /**
  Class that handles and sanity checks the material logic in the terrain. Every terrain material operation on the terrain grid should go through here.

  To support a multimaterial inhomogeneous terrain model, the controller needs to keep track of several TerrainMaterial objects and their corresponding
  agx::Material objects. The TerrainMaterial objects govern internal terrain behavior such as avalanching and failure zone generation, whilst the
  agx::Material objects govern interactions between the terrain and other AGX objects, such as the shovel or other rigid bodies.

  The material information is stored in a sparse grid with material pairs of <TerrainMaterial*, agx::Material*>. There is a default material pair that is
  assumed to include every non assigned voxel in the sparse grid. The underlying grid is stored in the TerrainDataAtlas class - but access is given through the TerrainMaterialController.

  The controller also implement some utility functions:
  - A function to get weights of the different material pairs in a set of voxels.
  - Functions that average TerrainMaterial properties, such as density, using said weights.
  - Functions that average ContactMaterial properties using said weights between the terrain and an external shovel material
  */
  class AGXTERRAIN_EXPORT TerrainMaterialController : public agx::Referenced
  {
  public:
    /**
    Construct given terrain instance.
    */
    TerrainMaterialController(Terrain* terrain);

    /**
    Default constructor used in serialization.
    */
    TerrainMaterialController();

    /*
    Set the default terrain material and it's corresponding default material.
    */
    void setDefaultTerrainMaterial(TerrainMaterial* defaultTerrainMaterial, agx::Material* defaultMaterial = nullptr);
    /*
    Get the default terrain material.
    */
    TerrainMaterial* getDefaultTerrainMaterial();
    /*
    Set only the default material.
    */
    void setDefaultMaterial(agx::Material* defaultMaterial);
    /*
    Get the default material.
    */
    agx::Material* getDefaultMaterial();

    /*
    Add a TerrainMaterial to the specified vector with voxel indices
    \param terrainMaterial - The TerrainMaterial to set.
    \param coords - The vector of voxel indices to set the terrain material in. NOTE: GridCoord is not terrainIndex and depth! It is voxel index.
    \return num voxels assigned with new material.
    */
    int addTerrainMaterial(TerrainMaterial* terrainMaterial, const Grid::GridCoordVector& coords);

    /*
    Add a TerrainMaterial to the specified voxel index
    \param terrainMaterial - The TerrainMaterial to set.
    \param coords - Voxel index to set the terrain material in. NOTE: GridCoord is not terrainIndex and depth! It is voxel index.
    \return true if the material was assigned, false otherwise.
    */
    bool addTerrainMaterial(TerrainMaterial* terrainMaterial, const Grid::GridCoord& coord);

    /*
    Add a TerrainMaterial to the data atlas. A new associated material will be created for it by default.
    It is not assigned to any parts of the terrain.
    \param terrainMaterial - The TerrainMaterial to add.
    \return true if the material was added, false otherwise. If false, perhaps the terrainMaterial was already added?
    */
    bool addTerrainMaterial(TerrainMaterial* terrainMaterial);

    /*
    Associate a Material with an added TerrainMaterial. The given material is used to configure contact materials between
    the terrain and shovel.
    \param material - The material
    \param terrainMaterial - The added TerrainMaterial the material should be associated with.
    */
    bool associateMaterialToTerrainMaterial(agx::Material* material, TerrainMaterial* terrainMaterial);

    /*
    Get the Terrainmaterial in the specified voxel index
    */
    TerrainMaterial* getTerrainMaterial(const Grid::GridCoord& coord) const;

    /*
    \return the Terrainmaterial in the terrain voxel given a world position.
    */
    TerrainMaterial* getTerrainMaterial(const agx::Vec3 worldPosition);

    /*
    Get the added terrain materials
    */
    agx::Vector<TerrainMaterial*> getTerrainMaterials() const;
    /*
    Get the terrainMaterialIndex for the specified terrainMaterial.
    \param terrainMaterial - the TerrainMaterial you wish to get the index for.
    \returns - the index for the given TerrainMaterial or the number of added terrain materials if not added.
    */
    agx::UInt32 getTerrainMaterialIndex(TerrainMaterial* terrainMaterial);

    /*
    Get the solid voxels that intersects with the given geometry.
    */
    agx::Vec3iVector getIntersectingVoxels(agxCollide::Geometry* geometry);

    /*
    Set the added terrain materials. Clears previously added terrain materials.
    */
    bool setTerrainMaterials(agx::Vector<TerrainMaterial*> terrainMaterials);

    /*
    Exchange an already added TerrainMaterial with another TerrainMaterial without changing its associated agx::Material or assigned domain.
    */
    bool exchangeTerrainMaterial(TerrainMaterial* oldTerrainMaterial, TerrainMaterial* newTerrainMaterial);

    /*
    Remove an added terrain material, its assigned domain will also be removed along with it.
    \note This method can not remove the default terrain material, just set the default terrain material instead!
    */
    bool removeTerrainMaterial(TerrainMaterial* terrainMaterialToRemove);


    /*
    Clear the voxel grid of added materials.
    \param clearAddedMaterials - true to also clear the list of added materials, false to only clear the assigned domains.
    */
    void clearTerrainMaterialGrid(bool clearAddedMaterials);


    /*
    \return The corresponding agx::Material for the specified TerrainMaterial or nullptr if the TerrainMaterial isn't included in the terrain.
    */
    agx::Material* getMaterial(TerrainMaterial* terrainMaterial) const;

    /*
    Set the default agx::Material which corresponds to the default terrain material.
    */
    void setMaterial(agx::Material* defaultMaterial);

    /**
    Get a pre-computed debug color given a material index. Used to for static correlation of
    specific material index to color for debugging purposes. [Max: 8]
    \param index - the specified material index.
    \return the debug color for the specific index
    */
    agxRender::Color getMaterialDebugColor(size_t index);

    /*
    Render the added terrain materials using the getMaterialDebugColors
    /note Very slow function, take a look at TerrainVoxelRenderer for faster rendering.
    */
    void debugRenderMaterials();

    /*
    \return Pairs of vectors containing the voxel indices and corresponding terrain material indices.
    */
    TerrainMaterialsInVoxels getTerrainMaterialsInVoxels();


  public:

    MaterialPairVector getTerrainMaterialPairs();
    void setTerrainMaterialPairs(MaterialPairVector materialPairs);

    /*
    Function used by pager to set terrain materials at specific voxel indices
    \param terrainMaterialsInVoxels
    \param resetTerrainMaterials - true to clear the grid before assignment, false to keep already assigned materials.
    */
    void pagerSetTerrainMaterialsInVoxels(TerrainMaterialsInVoxels terrainMaterialsInVoxels, bool resetTerrainMaterials);

    /*
    Function used by pager to set one terrain material at specific voxel indices
    \param voxelIndices - The voxel indices to set at
    \param terrainMaterialIndex - The terrain material index
    \param resetTerrainMaterials - true to clear the grid before assignment, false to keep already assigned materials.
    */
    void pagerSetTerrainMaterialInVoxels(agx::Vec3iVector voxelIndices, agx::UInt32 terrainMaterialIndex, bool resetTerrainMaterials = true);

    /*
    Function used to remove empty voxels from the terrain material grid.
    If there is no occupancy left, the information that it contains another material is also removed.
    */
    void removeEmptyVoxelsFromTable(Grid::GridCoordVector& coords, std::function<float(const agx::Vec3i&)> getOccupancy);

    /*
    Calculates the weigths of terrain materials in the specified voxel indices.
    The weights are used to calculate an average property from the terrain materials, see for example TerrainMaterialController::getDensity(weights) etc.
    \internal
    Index 0 is defaultTerrainMaterial, the following indices map such that weights[i+1] <-> getData()->getTerrainMaterials[i].
    \endinternal
    */
    agx::RealVector getWeights(const Grid::GridCoordVector& coords);

    /**
    Return the material weights for the voxels along the failure plane of the specified active
    zone. This is used to computed the weights contact material properties use in the
    terrain <-> aggregate contacts.
    \param activeZone - The specfied active zone where the failure plane will be used for
                        voxel extraction.
    \return the vector contianing the material weights for the voxels touched by the points.
    */
    agx::RealVector getMaterialWeightsInFailureZone(ActiveZone* activeZone);

    /**
    Return the material weights for the voxels touched by the specified points.
    \param points - the points to used when extracting voxels for computing
                    material weights.
    \return the vector contianing the material weights for the voxels touched by the points.
    */
    agx::RealVector getMaterialWeightsInPoints(const agx::Vec3Vector& points);

    /// Averaging properties for TerrainMaterial properties
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getDensity(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getYoungsModulus(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getSwellFactor(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getCohesion(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getHardeningConstantKE(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getHardeningConstantNE(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getAggregateStiffnessMultiplier(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getMaximumAggregateNormalForce(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getDepthAngleThreshold(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getMaximumDepth(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getDepthDecayFactor(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getDepthIncreaseFactor(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getFrictionAngle(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getDilatancyAngleScalingFactor(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getDilatancyAngle(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getPoissonsRatio(const agx::RealVector& weights);
    /*
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    */
    agx::Real getExcavationStiffnessMultiplier(const agx::RealVector& weights);

    // CONTACT MATERIAL AVERAGING

    /**
    Get the weighted friction coefficient of the contact materials between the shovel material and the relevant materials in the terrain.
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    \param shovelMaterial - The material of the shovel, used to get the contact material between the shovel material and the relevant materials in the terrain.
    */
    agx::Real getSoilToolFrictionCoefficient(const agx::RealVector& weights, agx::Material* shovelMaterial);

    /**
    Get the weighted adhesion of the contact materials between the shovel material and the relevant materials in the terrain.
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    \param shovelMaterial - The material of the shovel, used to get the contact material between the shovel material and the relevant materials in the terrain.
    */
    agx::Real getSoilToolAdhesion(const agx::RealVector& weights, agx::Material* shovelMaterial);

    /**
    Get the weighted Young's modulus of the contact materials between the shovel material and the relevant materials in the terrain.
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    \param shovelMaterial - The material of the shovel, used to get the contact material between the shovel material and the relevant materials in the terrain.
    */
    agx::Real getSoilToolYoungsModulus(const agx::RealVector& weights, agx::Material* shovelMaterial);

    /**
    Get the weighted adhesive overlap of the contact materials between the shovel material and the relevant materials in the terrain.
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    \param shovelMaterial - The material of the shovel, used to get the contact material between the shovel material and the relevant materials in the terrain.
    */
    agx::Real getSoilToolAdhesiveOverlap(const agx::RealVector& weights, agx::Material* shovelMaterial);

    /**
    Get the weighted damping of the contact materials between the shovel material and the relevant materials in the terrain.
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    \param shovelMaterial - The material of the shovel, used to get the contact material between the shovel material and the relevant materials in the terrain.
    */
    agx::Real getSoilToolDamping(const agx::RealVector& weights, agx::Material* shovelMaterial);

    /**
    Get the weighted rolling resistance of the contact materials between the shovel material and the relevant materials in the terrain.
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    \param shovelMaterial - The material of the shovel, used to get the contact material between the shovel material and the relevant materials in the terrain.
    */
    agx::Real getSoilToolRollingResistanceCoefficient(const agx::RealVector& weights, agx::Material* shovelMaterial);

    /**
    Get the weighted twisting resistance of the contact materials between the shovel material and the relevant materials in the terrain.
    \param weights - Weight vector for the parameter used to perform averaging. Use getWeights to get the correct weights.
    \param shovelMaterial - The material of the shovel, used to get the contact material between the shovel material and the relevant materials in the terrain.
    */
    agx::Real getSoilToolTwistingResistanceCoefficient(const agx::RealVector& weights, agx::Material* shovelMaterial);



    AGXTERRAIN_STORE_RESTORE_INTERFACE;

  protected:

    virtual ~TerrainMaterialController();

    TerrainDataAtlas* getData() const;

    Grid::GridCoordVector voxelIndicesAlongLine(agx::Line line);

    /*
    /internal
    Utility function used to average a property from the terrain materials in the terrain instance.
    \param getPropertyFunc - a lambda that takes a TerrainMaterial and returns the property that should be averaged.
    \param weights - weights for the different terrain materials. See getWeights(const Grid::GridCoordVector& coords).
    /endinternal
    */
    agx::Real averageProperty(std::function<agx::Real(TerrainMaterial* terrainMaterial)> getPropertyFunc, const agx::RealVector& weights);

    /*
    /internal
    Utility function used to average a property from the contact materials in the terrain instance.
    Contact materials are retrieved or calculated implicitly between the given shovelMaterial and the
    agx::Materials corresponding to the TerrainMaterials.
    \param getPropertyFunc - a lambda that takes a ContactMaterial and returns the averaged property.
    \param weights - weights for the different terrain materials. See getWeights(const Grid::GridCoordVector& coords).
    \param shovelMaterial - the agx::Material of the shovel.
    /endinternal
    */
    agx::Real averageTerrainToolProperty(std::function<agx::Real(agx::ContactMaterial* contactMaterial)> getPropertyFunc, const agx::RealVector& weights, agx::Material* shovelMaterial);


  private:
    Terrain*                      m_terrain;
  };
  DOXYGEN_END_INTERNAL_BLOCK()
}