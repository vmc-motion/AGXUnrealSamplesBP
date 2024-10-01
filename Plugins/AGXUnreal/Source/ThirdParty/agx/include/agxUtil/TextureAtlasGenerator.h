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

#include <agx/agxPhysics_export.h>
#include <agx/Math.h>
#include <agx/Referenced.h>
#include <agxCollide/RenderData.h>

namespace agxUtil
{

  /**
  This class will take an input mesh and recompute texture coordinates to fit on a texture atlas.
  It might change the number of vertices/normals and texture coordinates.
  */
  class AGXPHYSICS_EXPORT TextureAtlasGenerator
  {
    public:

      /**
      From the input data, it will build a texture atlas, recompute texture coordinates to the output data.
      The number of vertices, normals and texCoordinates must be larger than zero and equal in size.
      The input vectors (vertices, normals, indices, texCoordinates) must be separate from the output vectors.

      \param vertices - Input vertex positions
      \param normals - Input normals
      \param indices - Input indices
      \param texCoordinates - Input texture cordinates
      \param out_vertices - Output vertex positions
      \param out_normals - Output normals
      \param out_indices - Output indices
      \param out_texCoordinates - Output texture cordinates
      \return true if the indata and outdata is valid.
      */
      static bool process(
        const agx::Vec3Vector& vertices,
        const agx::Vec3Vector& normals,
        const agx::UInt32Vector& indices,
        const agx::Vec2Vector& texCoordinates,
        
        agx::Vec3Vector& out_vertices,
        agx::Vec3Vector& out_normals,
        agx::UInt32Vector& out_indices,
        agx::Vec2Vector& out_texCoordinates
        );

      /**
      From the input data, it will build a texture atlas, recompute texture coordinates to the output data.
      The number of vertices, normals and texCoordinates must be larger than zero and equal in size.

      \param renderData - The data in the object will be read and the recomputed data will be written back into renderData.
      \return true if the indata and outdata is valid.
      */
      static bool process(agxCollide::RenderData* renderData);
  };
}
