// Copyright 2024, Algoryx Simulation AB.

#pragma once

#include "Shapes/ShapeBarrier.h"

#include <memory>

class AGXUNREALBARRIER_API FHeightFieldShapeBarrier : public FShapeBarrier
{
public:
	FHeightFieldShapeBarrier();
	FHeightFieldShapeBarrier(FHeightFieldShapeBarrier&& Other) = default;
	FHeightFieldShapeBarrier(std::unique_ptr<FGeometryAndShapeRef> Native);
	virtual ~FHeightFieldShapeBarrier() override;

	FHeightFieldShapeBarrier& operator=(FHeightFieldShapeBarrier&& Other) = default;

	void AllocateNative(int32 NumVerticesX, int32 NumVerticesY, double SizeX, double SizeY);

	void SetHeights(const TArray<float>& Heights);

private:
	virtual void AllocateNativeShape() override;
	virtual void ReleaseNativeShape() override;

	void AllocateNativeHeightField(
		int32 NumVerticesX, int32 NumVerticesY, double SizeX, double SizeY);

private:
	FHeightFieldShapeBarrier(const FHeightFieldShapeBarrier&) = delete;
	void operator=(const FHeightFieldShapeBarrier&) = delete;
};
