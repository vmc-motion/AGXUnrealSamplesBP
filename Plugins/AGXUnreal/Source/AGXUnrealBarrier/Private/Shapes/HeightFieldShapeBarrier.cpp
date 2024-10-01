// Copyright 2024, Algoryx Simulation AB.

#include "Shapes/HeightFieldShapeBarrier.h"

#include "AGXRefs.h"
#include "ShapeBarrierImpl.h"
#include "TypeConversions.h"

namespace
{
	agxCollide::HeightField* NativeHeightField(FHeightFieldShapeBarrier* Barrier)
	{
		return Barrier->GetNative()->NativeShape->as<agxCollide::HeightField>();
	}

	const agxCollide::HeightField* NativeHeightField(const FHeightFieldShapeBarrier* Barrier)
	{
		return Barrier->GetNative()->NativeShape->as<agxCollide::HeightField>();
	}
}

FHeightFieldShapeBarrier::FHeightFieldShapeBarrier()
	: FShapeBarrier()
{
}

FHeightFieldShapeBarrier::FHeightFieldShapeBarrier(std::unique_ptr<FGeometryAndShapeRef> Native)
	: FShapeBarrier(std::move(Native))
{
	check(NativeRef->NativeShape->is<agxCollide::HeightField>());
}

FHeightFieldShapeBarrier::~FHeightFieldShapeBarrier()
{
	// Must provide a destructor implementation in the .cpp file because the
	// std::unique_ptr NativeRef's destructor must be able to see the definition,
	// not just the forward declaration, of FHeightFieldShapeRef.
}

void FHeightFieldShapeBarrier::AllocateNative(
	int32 NumVerticesX, int32 NumVerticesY, double SizeX, double SizeY)
{
	FShapeBarrier::AllocateNative(
		[this, NumVerticesX, NumVerticesY, SizeX, SizeY]()
		{ this->AllocateNativeHeightField(NumVerticesX, NumVerticesY, SizeX, SizeY); });
}

void FHeightFieldShapeBarrier::SetHeights(const TArray<float>& Heights)
{
	check(HasNative());

	agx::VectorPOD<agx::Real> HeightsAGX;
	HeightsAGX.reserve(static_cast<size_t>(Heights.Num()));

	for (auto& Height : Heights)
	{
		HeightsAGX.push_back(ConvertDistanceToAGX(Height));
	}

	NativeHeightField(this)->setHeights(HeightsAGX);
}

void FHeightFieldShapeBarrier::AllocateNativeHeightField(
	int32 NumVerticesX, int32 NumVerticesY, double SizeX, double SizeY)
{
	check(!HasNative());

	agx::Real SizeXAGX = ConvertDistanceToAGX(SizeX);
	agx::Real SizeYAGX = ConvertDistanceToAGX(SizeY);

	NativeRef->NativeShape = new agxCollide::HeightField(
		static_cast<size_t>(NumVerticesX), static_cast<size_t>(NumVerticesY), SizeXAGX, SizeYAGX);
}

void FHeightFieldShapeBarrier::AllocateNativeShape()
{
	checkNoEntry();
}

void FHeightFieldShapeBarrier::ReleaseNativeShape()
{
	NativeRef->NativeShape = nullptr;
}
