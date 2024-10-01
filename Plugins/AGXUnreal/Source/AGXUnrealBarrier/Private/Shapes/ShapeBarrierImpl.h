// Copyright 2024, Algoryx Simulation AB.

// This header file should only be included from source files built as part of AGXUnrealBarrier.

#pragma once

#include "Shapes/ShapeBarrier.h"

#include "AGXRefs.h"

#include <utility>

template <typename TFunc, typename... TPack>
void FShapeBarrier::AllocateNative(TFunc Factory, TPack... Params)
{
	/// \todo This almost copy/paste from the non-templated version. Find a way to
	/// call one from the other, or some other way to share implementation.
	check(!HasNative());
	NativeRef->NativeGeometry = new agxCollide::Geometry();
	Factory(std::forward<TPack>(Params)...);
	NativeRef->NativeGeometry->add(NativeRef->NativeShape);
}

template <typename T>
T* FShapeBarrier::GetNativeShape()
{
	check(HasNative());
	check(NativeRef->NativeShape->is<T>());
	return NativeRef->NativeShape->asSafe<T>();
}

template <typename T>
const T* FShapeBarrier::GetNativeShape() const
{
	check(HasNative());
	check(NativeRef->NativeShape->is<T>());
	return NativeRef->NativeShape->asSafe<T>();
}
