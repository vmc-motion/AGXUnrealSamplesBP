// Copyright 2024, Algoryx Simulation AB.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FAGXCommonModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
