// Copyright 2024, Algoryx Simulation AB.

#pragma once

#include "BeginAGXIncludes.h"
#include <agxWire/WireWinchController.h>
#include "EndAGXIncludes.h"

struct FWireWinchRef
{
	using NativeType = agxWire::WireWinchController;
	agxWire::WireWinchControllerRef Native;

	FWireWinchRef() = default;
	FWireWinchRef(agxWire::WireWinchController* InNative)
		: Native(InNative)
	{
	}
};
