// Copyright 2024, Algoryx Simulation AB.

#pragma once

// Unreal Engine includes.
#include "CoreMinimal.h"

class FRigidBodyBarrier;

class AGXUNREALBARRIER_API FAGXUtilities
{
public:

	/**
	* Body will get the parents velocity added to its velocity/angular velocity as if it was
	* rigidly attached.
	*/
	static void AddParentVelocity(const FRigidBodyBarrier& Parent, FRigidBodyBarrier& Body);
};
