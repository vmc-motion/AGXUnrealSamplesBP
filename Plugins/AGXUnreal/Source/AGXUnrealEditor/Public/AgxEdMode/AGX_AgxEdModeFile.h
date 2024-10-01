// Copyright 2024, Algoryx Simulation AB.

#pragma once

// AGX Dynamics for Unreal includes.
#include "AgxEdMode/AGX_AgxEdModeSubMode.h"

// Unreal Engine includes.
#include "CoreMinimal.h"

#include "AGX_AgxEdModeFile.generated.h"

/**
 * Sub-mode for AgxEdMode. Used to import/export .agx and .urdf files.
 */
UCLASS(ClassGroup = "AGX", Category = "AGX", config = EditorPerProjectUserSettings)
class AGXUNREALEDITOR_API UAGX_AgxEdModeFile : public UAGX_AgxEdModeSubMode
{
	GENERATED_BODY()

public:
	static UAGX_AgxEdModeFile* GetInstance();

	virtual FText GetDisplayName() const override;
	virtual FText GetTooltip() const override;
	virtual FSlateIcon GetIcon() const override;

	static void ImportToBlueprint();
	static void ExportAgxArchive();

	UFUNCTION(BlueprintCallable, Category = "AGX Dynamics")
	static void SynchronizeModel_BP(UObject* Blueprint);
};
