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

#ifndef AGXMODEL_PRESSUREFIELDRENDERER_H
#define AGXMODEL_PRESSUREFIELDRENDERER_H

#include <agxModel/WindAndWaterUtils.h>

#include <agx/Referenced.h>
#include <agx/AffineMatrix4x4.h>

namespace agxModel
{
  class AGXMODEL_EXPORT PressureFieldRenderer : public agx::Referenced
  {
    public:
      /**
      On add triangle.
      \param tData - triangle data with three vertices, also containing data
      \param worldToShape - world to shape transform, matches as updateTransform( worldToShape.inverse() )
      */
      virtual void addTriangle( const agxModel::TriangleData& tData, const agx::AffineMatrix4x4& worldToShape ) = 0;

      /**
      On clear. Usually clear() -> addTriangle -> addPressure -> done.
      */
      virtual void clear() = 0;

      /**
      On done. Usually clear -> addTriangle -> addPressure -> done.
      */
      virtual void done() = 0;

      /**
      Called when the shape transform may have been updated.
      */
      virtual void updateTransform( const agx::AffineMatrix4x4& /*shapeToWorld*/ ) {}
  };

  typedef agx::ref_ptr<PressureFieldRenderer> PressureFieldRendererRef;
}

#endif
