// Copyright 2024, Algoryx Simulation AB.

#include "AGXUnreal.h"

// AGX Dynamics for Unreal includes.
#include "AGX_Environment.h"
#include "AGX_LogCategory.h"
#include "AGX_RuntimeStyle.h"
#include "Materials/AGX_ShapeMaterial.h"
#include "Materials/AGX_TerrainMaterial.h"

// Unreal Engine includes.
#include "UObject/CoreRedirects.h"

#define LOCTEXT_NAMESPACE "FAGXUnrealModule"

namespace AGXUnrealModule_helpers
{
	void PrintVersion()
	{
		const FString Version = FAGX_Environment::GetPluginVersion();
		FString Revision = FAGX_Environment::GetPluginRevision();
		if (!Revision.IsEmpty())
		{
			Revision = FString::Printf(TEXT(", revision %s"), *Revision);
		}
		UE_LOG(
			LogAGX, Log, TEXT("AGX Dynamics for Unreal (AGXUnreal) version %s%s."), *Version,
			*Revision);
	}
}

void FAGXUnrealModule::StartupModule()
{
	using namespace AGXUnrealModule_helpers;
	PrintVersion();
	RegisterCoreRedirects();
	FAGX_RuntimeStyle::Initialize();
	FAGX_RuntimeStyle::ReloadTextures();
}

void FAGXUnrealModule::ShutdownModule()
{
	FAGX_RuntimeStyle::Shutdown();
}

void FAGXUnrealModule::RegisterCoreRedirects()
{
	// This is used to handle name changes of UFUNCTIONs, UPROPERTYs and UObjects (backward
	// compatibility). See
	// https://docs.unrealengine.com/en-US/core-redirects-in-unreal-engine/

	TArray<FCoreRedirect> Redirects;

	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_ContactMaterialBase"),
		TEXT("AGX_ContactMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_ContactMaterialAsset"),
		TEXT("AGX_ContactMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_ContactMaterialInstance"),
		TEXT("AGX_ContactMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_ShapeMaterialBase"), TEXT("AGX_ShapeMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_ShapeMaterialAsset"), TEXT("AGX_ShapeMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_ShapeMaterialInstance"),
		TEXT("AGX_ShapeMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_TerrainMaterialBase"),
		TEXT("AGX_TerrainMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_TerrainMaterialAsset"),
		TEXT("AGX_TerrainMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_TerrainMaterialInstance"),
		TEXT("AGX_TerrainMaterial"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Class, TEXT("AGX_MaterialBase"),
		TEXT("AGX_ShapeMaterial"));

	// The Shovel Refactor effort, the addition of Shovel Component, also introduced
	// FAGX_ComponentReference and replaced the FAGX_RigidBodyComponentReference and
	// FAGX_SceneComponentReference implementations with ones based on the new reference class. For
	// a while we had both FAGX_BodyReference and FAGX_RigidBodyReference used in different parts of
	// the code. Eventually the old one was removed and the new took the old ones' name. This
	// redirect makes it so that scenes created during that interim time can find the new name even
	// though the asset was saved with the old. I think this was only ever used by the Shovel
	// Component and never released to users.
	//
	// In the switch the name of the property naming the referenced Component was changed from
	// BodyName to just Name. Add a redirect for that as well, so that assets using the old
	// FAGX_RigidBodyReference can be restored into the the new FAGX_ComponentReference based
	// FAGX_RigidBodyReference. Also the same for SceneComponentName used by the old
	// FAGX_SceneComponentReference.
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Struct, TEXT("AGX_BodyReference"), TEXT("AGX_RigidBodyReference"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Property,
		TEXT("/Script/AGXUnreal.AGX_RigidBodyReference.BodyName"),
		TEXT("/Script/AGXUnreal.AGX_RigidBodyReference.Name"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Property,
		TEXT("/Script/AGXUnreal.AGX_SceneComponentReference.SceneComponentName"),
		TEXT("/Script/AGXUnreal.AGX_SceneComponentReference.Name"));

	// Properties renamed in AGX Dynamics with the introduction of AGX Dynamics 2.37.
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Property, TEXT("AGX_Shovel.BottomContactThreshold"),
		TEXT("ContactRegionThreshold"));
	Redirects.Emplace(
		ECoreRedirectFlags::Type_Property, TEXT("AGX_Shovel.bOverrideBottomContactThreshold"),
		TEXT("bOverrideContactRegionThreshold"));

	// ShapeMaterial UFUNCTIONs with _BP postfix removed.
	// This should be feasable to removed in AGXUnreal 1.14 or later.
	for (TFieldIterator<UFunction> FuncIt(UAGX_ShapeMaterial::StaticClass()); FuncIt; ++FuncIt)
	{
		UFunction* Function = *FuncIt;
		const FString BpName = FString::Printf(TEXT("AGX_ShapeMaterial.%s_BP"), *Function->GetName());
		Redirects.Emplace(ECoreRedirectFlags::Type_Function, *BpName, *Function->GetName());
	}

	// TerrainMaterial UFUNCTIONs with _BP postfix removed.
	// This should be feasable to removed in AGXUnreal 1.14 or later.
	for (TFieldIterator<UFunction> FuncIt(UAGX_TerrainMaterial::StaticClass()); FuncIt; ++FuncIt)
	{
		UFunction* Function = *FuncIt;
		const FString BpName =
			FString::Printf(TEXT("AGX_TerrainMaterial.%s_BP"), *Function->GetName());
		Redirects.Emplace(ECoreRedirectFlags::Type_Function, *BpName, *Function->GetName());
	}

	FCoreRedirects::AddRedirectList(Redirects, TEXT("AGXUnreal"));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FAGXUnrealModule, AGXUnreal)
