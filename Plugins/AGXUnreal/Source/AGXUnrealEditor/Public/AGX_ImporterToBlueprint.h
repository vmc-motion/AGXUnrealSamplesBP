// Copyright 2024, Algoryx Simulation AB.

#pragma once

// Unreal Engine includes.
#include "CoreMinimal.h"

class UBlueprint;
class FString;
struct FAGX_ImportSettings;
struct FAGX_SynchronizeModelSettings;

namespace AGX_ImporterToBlueprint
{
	/**
	 * Read simulation objects from a .agx archive or .urdf file and create an Actor
	 * Blueprint populated with corresponding AGXUnreal objects.
	 *
	 * @param ImportSettings - Struct containing all information needed to perform the import.
	 * @return An Actor Blueprint containing the read objects.
	 */
	AGXUNREALEDITOR_API UBlueprint* Import(const FAGX_ImportSettings& ImportSettings);

	/*
	 * Synchronize the model (imported Blueprint) with the source file on disk.
	 * Assets and Components are updated according to the source file.
	 * Any child Blueprint of the passed Blueprint will also be updated but properties will only be
	 * updated for Components in child Blueprints if they have not been changed by a user.
	 *
	 * @param BaseBP - The Blueprint (base) containing the ModelSourceComponent.
	 * @param Settings - Struct containing all information needed to perform the
	 * synchronization.
	 * @param OpenBlueprint (Optional) - If this function was called from the details panel of a
	 * component inside a Blueprint, this Blueprint should be passed here. Can be a child of BaseBP.
	 * @return True if the Model synchronization was successful, false otherwise.
	 */
	AGXUNREALEDITOR_API bool SynchronizeModel(
		UBlueprint& BaseBP, const FAGX_SynchronizeModelSettings& Settings,
		UBlueprint* OpenBlueprint = nullptr);
}
