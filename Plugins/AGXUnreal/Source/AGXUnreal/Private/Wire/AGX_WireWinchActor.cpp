// Copyright 2024, Algoryx Simulation AB.

#include "Wire/AGX_WireWinchActor.h"

// AGX Dynamics for Unreal include.s
#include "Wire/AGX_WireWinchComponent.h"

AAGX_WireWinchActor::AAGX_WireWinchActor()
{
	PrimaryActorTick.bCanEverTick = false;

	WireWinchComponent =
		CreateDefaultSubobject<UAGX_WireWinchComponent>(TEXT("WireWinchComponent"));
	SetRootComponent(WireWinchComponent);
}
