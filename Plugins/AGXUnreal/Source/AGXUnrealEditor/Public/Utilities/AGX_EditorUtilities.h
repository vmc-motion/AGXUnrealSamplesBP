// Copyright 2024, Algoryx Simulation AB.

#pragma once

// AGX Dynamics for Unreal includes.
#include "AGX_LogCategory.h"
#include "Utilities/AGX_ObjectUtilities.h"

// Unreal Engine includes.
#include "AssetRegistry/AssetData.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "CoreMinimal.h"
#include "DetailLayoutBuilder.h"
#include "GameFramework/Actor.h"
#include "Misc/EngineVersionComparison.h"
#include "RawMesh.h"

#if UE_VERSION_OLDER_THAN(5, 0, 0)
#include "EditorStyle.h"
#else
#include "Styling/AppStyle.h"
#endif

// Standard library includes.
#include <tuple>

// Shape classes.
class FRenderDataBarrier;
class FShapeMaterialBarrier;
class FTrimeshShapeBarrier;
class UAGX_BoxShapeComponent;
class UAGX_CapsuleShapeComponent;
class UAGX_CylinderShapeComponent;
class UAGX_ShapeComponent;
class UAGX_SphereShapeComponent;
class UAGX_TrimeshShapeComponent;

// Constraint classes.
class AAGX_ConstraintActor;
class AAGX_ConstraintFrameActor;
class UAGX_ConstraintComponent;
class UAGX_HingeConstraintComponent;
class UAGX_PrismaticConstraintComponent;

// Other AGXUnreal classes.
class FContactMaterialBarrier;
class UAGX_RigidBodyComponent;
struct FAssetToDiskInfo;

// Unreal Engine classes.
class AActor;
class FText;
class IDetailLayoutBuilder;
class UClass;
class USceneComponent;
class UStaticMesh;
class UStaticMeshComponent;
class UWorld;

/**
 * A collection of helper functions that can only be compiled in editor builds.
 */
class AGXUNREALEDITOR_API FAGX_EditorUtilities
{
public:
	/**
	 * Utility function for starting a model synchronization, with GUI Window etc.
	 */
	static void SynchronizeModel(UBlueprint& Blueprint);

	/**
	 * Renames an existing and previously saved asset. WantedName will be sanitized and if there
	 * are name conflicts, a unique name is generated by appending a digit.
	 * If the WantedName is the same as the current Asset name, nothing is done.
	 * Note that this functions does not re-save the asset. This should be done by the
	 * caller at some point.
	 */
	static bool RenameAsset(UObject& Asset, const FString& WantedName, const FString& AssetType);

	static bool DeleteAsset(UObject& Asset);

	static int32 DeleteImportedAssets(const TArray<UObject*>& Assets);

	static void NullReferencesToObject(UObject* ToDelete);

	/**
	 * Create a new Actor with an empty USceneComponent as its RootComponent.
	 */
	static std::tuple<AActor*, USceneComponent*> CreateEmptyActor(
		const FTransform& Transform, UWorld* World);

	/**
	 * Create a new AGX Rigid Body Component as a child of the given Actor.
	 */
	static UAGX_RigidBodyComponent* CreateRigidBody(AActor* Owner);

	/**
	 * Create a new AGX Sphere Shape as child of the given Actor.
	 * The Shape will be attached to the given USceneComponent.
	 */
	static UAGX_SphereShapeComponent* CreateSphereShape(AActor* Owner, USceneComponent* Outer);

	/**
	 * Create a new AGX Box Shape as child of the given Actor.
	 * The Shape will be attached to the given USceneComponent.
	 */
	static UAGX_BoxShapeComponent* CreateBoxShape(AActor* Owner, USceneComponent* Outer);

	/**
	 * Create a new AGX Cylinder Shape as a child of the given Actor.
	 * The Shape will be attached to the given USceneComponent.
	 * @param Owner - The Actor that should own the new Cylinder Shape.
	 * @param Outer - The SceneComponent that the new Cylinder Shape should be attached to.
	 * @return A newly created Cylinder Shape.
	 */
	static UAGX_CylinderShapeComponent* CreateCylinderShape(AActor* Owner, USceneComponent* Outer);

	/**
	 * Create a new AGX Capsule shape as a child of the given Actor.
	 * The Shape will be attached to the given USceneComponent.
	 * @param Owner - The Actor that should own the new Capsule Shape.
	 * @param Outer - The SceneComponent that the new Capsule Shape should be attached to.
	 * @return A newly created Capsule Shape.
	 */
	static UAGX_CapsuleShapeComponent* CreateCapsuleShape(AActor* Owner, USceneComponent* Outer);

	/**
	 * Create a new AGX Trimesh Shape as a child of the given actor.
	 * The Shape will be attached to the given USceneComponent.
	 * A StaticMeshComponent is neither selected nor created.
	 * @see CreateStaticMesh.
	 * @param Owner The Actor that should own the newly created Component.
	 * @param Outer The USceneComponent that the newly created Component should be a attached to.
	 * @param bRegister True if AActor::registerComponent should be called. Must be called later if
	 * false.
	 */
	static UAGX_TrimeshShapeComponent* CreateTrimeshShape(
		AActor* Owner, USceneComponent* Outer, bool bRegister);

	/**
	 * Create an FRawMesh from the collision triangles in the given Trimesh Shape Barrier.
	 *
	 * The mesh created will be limited to the information stored in the Trimesh, so no texture
	 * coordinates and only one normal per triangle.
	 *
	 * @param Trimesh The Trimesh holding the triangles to convert.
	 * @return An FRawMesh containing the same triangles as the Trimesh.
	 */
	static FRawMesh CreateRawMeshFromTrimesh(const FTrimeshShapeBarrier& Trimesh);

	/**
	 * Create an FRawMesh from the render triangles in the given Render Data Barrier.
	 *
	 * Will return an empty FRawMesh if the Render Data doesn't have a mesh.
	 *
	 * @param RenderData Render Data holding the triangles to convert.
	 * @return An FRawMesh containing the same triangles as the Render Data.
	 */
	static FRawMesh CreateRawMeshFromRenderData(const FRenderDataBarrier& RenderData);

	/**
	 * Apply the RawMesh data to the StaticMesh.
	 *
	 * @param RawMesh - The RawMesh holding the mesh data.
	 * @param StaticMesh - The StaticMesh that should receive the mesh data.
	 */
	static void AddRawMeshToStaticMesh(FRawMesh& RawMesh, UStaticMesh* StaticMesh);

	/**
	 * Remove characters that are unsafe to use in object names, content
	 * references or file names. Unsupported characters are dropped, so check the
	 * returned string for emptyness.
	 *
	 * May remove more characters than necessary, and the set of allowed characters may be extended
	 * in the future.
	 *
	 * @param Name The name to sanitize.
	 * @return The name with all dangerous characters removed.
	 */
	static FString SanitizeName(const FString& Name);

	/**
	 * Remove characters that are unsafe to use in object names, content references or file names.
	 * Unsupported characters are dropped. If no characters remain then the fallback is returned.
	 *
	 * May remove more characters than necessary.
	 *
	 * @param Name The name to sanitize.
	 * @param Fallback Returned if none of the original characters remain.
	 * @return The name with all dangerous characters removed, or the fallback if all characters are
	 * dangerous.
	 */
	static FString SanitizeName(const FString& Name, const FString& Fallback);

	/**
	 * Remove characters that are unsafe to use in object names, content references or file names.
	 * Unsupported characters are dropped. If no characters remain then the fallback is returned.
	 *
	 * May remove more characters than necessary.
	 *
	 * @param Name The name to sanitize.
	 * @param Fallback Returned if none of the original characters remain.
	 * @return The name with all dangerous characters removed, or the fallback if all characters are
	 * dangerous.
	 */
	static FString SanitizeName(const FString& Name, const TCHAR* Fallback);

	static FString CreateAssetName(FString SourceName, FString ActorName, FString DefaultName);

	static void MakePackageAndAssetNameUnique(FString& PackageName, FString& AssetName);

	/**
	 * Write the given static mesh assets to disk in bulk, which utilizes multi-threaded mesh build.
	 * Returns true if all of the Writes was successful, false otherwise.
	 */
	static bool SaveStaticMeshAssetsInBulk(const TArray<UStaticMesh*>& StaticMeshAssets);

	/**
	 * Create a new UStaticMesh asset from the given mesh data. The StaticMesh asset is saved to
	 * /Game/ImportedAgxMeshes/'AssetFolderName' with the source name that the native
	 * agxCollide::Trimesh has. If it does not have a source name then 'ImportedAgxMesh' is used
	 * instead.
	 *
	 * @param Trimesh - AGX Dynamics trimesh data to convert to a StaticMesh.
	 * @param AssetFolderName - The name of the folder within /Game/ImportedAgxMeshes' that the
	 * asset should be stored to.
	 * @param FallbackName - Name used for the new Mesh Asset in case Trimesh does not have a
	 * source name.
	 */
	static UStaticMesh* CreateStaticMeshAsset(
		const FTrimeshShapeBarrier& Trimesh, const FString& AssetFolderName,
		const FString& FallbackName);

	/**
	 * Create a new constraint of the specified type.
	 */
	static AAGX_ConstraintActor* CreateConstraintActor(
		UClass* ConstraintType, UAGX_RigidBodyComponent* RigidBody1,
		UAGX_RigidBodyComponent* RigidBody2, bool bSelect, bool bShowNotification,
		bool bInPlayingWorldIfAvailable);

	template <typename T>
	static T* CreateConstraintActor(
		UAGX_RigidBodyComponent* RigidBody1, UAGX_RigidBodyComponent* RigidBody2, bool bSelect,
		bool bShowNotification, bool bInPlayingWorldIfAvailable, UClass* ConstraintType = nullptr);

	/**
	 * Create a new AGX Constraint Frame Actor. Set as child to specified Rigid Body, if available.
	 */
	static AAGX_ConstraintFrameActor* CreateConstraintFrameActor(
		AActor* ParentActor, bool bSelect, bool bShowNotification, bool bInPlayingWorldIfAvailable);

	/**
	 * Change current selection to the specific actor.
	 */
	static void SelectActor(AActor* Actor, bool DeselectPrevious = true);

	/**
	 * Check if the given Component is selected anywhere. This includes both in the Level Viewport
	 * or in the Components panel of any Blueprint Editor.
	 */
	static bool IsSelected(const UActorComponent& Component);

	/**
	 * Return the Editor world, i.e. not the one that is potentially currently playing.
	 */
	static UWorld* GetEditorWorld();

	/**
	 * Return the currently playing world, or null if game is not playing.
	 *
	 * @remarks Playing world can be access directly from actors, using the GetWorld()
	 * function which returns the world they belong in. Therefore this function should
	 * typically only be used by Editor code that does not have direct access to an Actor.
	 */
	static UWorld* GetPlayingWorld();

	/**
	 * Return GetPlayingWorld if game is playing, else returns GetEditorWorld.
	 *
	 * @remarks Current world can be access directly from actors, using the GetWorld()
	 * function which returns the world they belong in. Therefore this function should
	 * typically only be used by Editor code that does not have direct access to an Actor.
	 */
	static UWorld* GetCurrentWorld();

	/**
	 * Find the first two found actors that has a UAGX_RigidBodyComponent in the current selection,
	 * with options to search children and parents in case the user has selected a graphics-only
	 * actor instead of the actual rigid body actor.
	 *
	 * @param bSearchSubtrees If true, each selected actor's subtree will also be searched. But,
	 * once a Rigid Body is found, the search of the corresponding selected actor's subtree will be
	 * finished, such that only one Rigid Body actor may be found per selected actor.
	 *
	 * @param bSearchAncestors If true, all ancestors of each selected actor's subtree will also be
	 * searched.
	 *
	 */
	static void GetRigidBodyActorsFromSelection(
		AActor** OutActor1, AActor** OutActor2, bool bSearchSubtrees, bool bSearchAncestors);

	static AActor* GetRigidBodyActorFromSubtree(AActor* SubtreeRoot, const AActor* IgnoreActor);

	static AActor* GetRigidBodyActorFromAncestors(AActor* Actor, const AActor* IgnoreActor);

	static void GetAllClassesOfType(
		TArray<UClass*>& OutMatches, UClass* BaseClass, bool bIncludeAbstract);

	/**
	 * Returns single object being customized from DetailBuilder if found.
	 *
	 * @param FailIfMultiple If true, nullptr is returned if multiple objects are found.
	 * If False, the first found object is returned, even if multiple objects are found.
	 */
	template <typename T>
	static T* GetSingleObjectBeingCustomized(
		const IDetailLayoutBuilder& DetailBuilder, bool FailIfMultiple = true);

	/**
	 * Convert a bool to a Slate visibility flag. True means visible, false means Collapsed (not
	 * Hidden).
	 */
	static EVisibility VisibleIf(bool bVisible);

	static FString SelectExistingFileDialog(
		const FString& FileDescription, const FString& FileExtension,
		const FString& InStartDir = "");

	static FString SelectExistingDirectoryDialog(
		const FString& DialogTitle, const FString& InStartDir = "", bool AllowNoneSelected = false);

	static FString SelectNewFileDialog(
		const FString& DialogTitle, const FString& FileTypes, const FString& DefaultFile = "",
		const FString& InStartDir = "");

	static FString SelectNewAssetDialog(
		UClass* SavedClass, const FString& InDefaultPath, const FString& AssetNameSuggestion,
		const FString& DialogTitle);

	static const ISlateStyle& GetStyle()
	{
#if UE_VERSION_OLDER_THAN(5, 1, 0)
		return FEditorStyle::Get();
#else
		return FAppStyle::Get();
#endif
	}

	static const FSlateBrush* GetBrush(FName PropertyName)
	{
#if UE_VERSION_OLDER_THAN(5, 1, 0)
		return FEditorStyle::GetBrush(PropertyName);
#else
		return FAppStyle::GetBrush(PropertyName);
#endif
	}

	/**
	 * Find (and loads) all assets of the type specified by ClassName that resides in AssetDirPath.
	 * Does not search recursively, so the AssetDirPath must be the directory which the Asset
	 * resides in, for example "/Game/ImportedAGXModels/MyModel/ShapeMaterial". Any returned asset
	 * is guaranteed not to be nullptr.
	 */
	template <typename T>
	static TArray<T*> FindAssets(const FString& AssetDirPath);

	/**
	 * Save and Compile the passed Blueprint. This function cannot reside in AGX_BlueprintUtilities
	 * since its implementation depends on an Editor module function.
	 */
	static void SaveAndCompile(UBlueprint& Blueprint);

	/**
	 * Returns relative path given a full path and base path. The base path must be part of the full
	 * path, with matching path delimiters.
	 */
	static FString GetRelativePath(const FString& BasePath, FString FullPath);
};

template <typename T>
T* FAGX_EditorUtilities::CreateConstraintActor(
	UAGX_RigidBodyComponent* RigidBody1, UAGX_RigidBodyComponent* RigidBody2, bool bSelect,
	bool bShowNotification, bool bInPlayingWorldIfAvailable, UClass* ConstraintType)
{
	if (ConstraintType == nullptr)
	{
		ConstraintType = T::StaticClass();
	}
	check(ConstraintType->IsChildOf<T>());
	return Cast<T>(CreateConstraintActor(
		ConstraintType, RigidBody1, RigidBody2, bSelect, bShowNotification,
		bInPlayingWorldIfAvailable));
}

template <typename T>
T* FAGX_EditorUtilities::GetSingleObjectBeingCustomized(
	const IDetailLayoutBuilder& DetailBuilder, bool FailIfMultiple)
{
	static_assert(std::is_base_of<UObject, T>::value, "T must inherit from UObject");

	TArray<TWeakObjectPtr<UObject>> Objects;
	DetailBuilder.GetObjectsBeingCustomized(Objects);

	if (Objects.Num() == 1 || (!FailIfMultiple && Objects.Num() > 1))
	{
		if (T* Component = Cast<T>(Objects[0].Get()))
		{
			return Component;
		}

		// This is a special case where the Component we want is a default subobject to an actor,
		// and the Component's details panel can be seen while the selected Component is actually
		// the actors root Component, not the Componetn we are after. This happens for our AGXUnreal
		// Actor classes, which have their corresponding Component type as a default subobject. We
		// simply look for a Component inside the actor of the correct type and return that. If
		// multiple Components of type T is a default subobject to the actor, then we currently
		// don't have a good way to detect that, and return the first found, but log a warning.
		if (AActor* Actor = Cast<AActor>(Objects[0].Get()))
		{
			TArray<UObject*> DefaultSubObjects;
#if UE_VERSION_OLDER_THAN(5, 4, 0)
			Actor->CollectDefaultSubobjects(DefaultSubObjects);
#else
			// Is this sufficient, or do we need to use ForEachObjectWithOuter?
			// See CollectDefaultSubobjects deprecation message in UObject/Object.h.
			Actor->GetDefaultSubobjects(DefaultSubObjects);
#endif
			TArray<T*> ComponentsOfInterest =
				FAGX_ObjectUtilities::Filter<T, TArray<UObject*>>(DefaultSubObjects);
			if (ComponentsOfInterest.Num() == 0)
				return nullptr;
			if (ComponentsOfInterest.Num() > 1)
			{
				UE_LOG(
					LogAGX, Warning,
					TEXT("Multiple Component candidates was found when looking for the Component "
						 "being Customized, likely due to multiple default subobjects of the same "
						 "type in a single Actor. The first Component will be returned, but the "
						 "result may not be the expected one."));
			}

			return ComponentsOfInterest[0];
		}
	}

	return nullptr;
}

template <typename T>
TArray<T*> FAGX_EditorUtilities::FindAssets(const FString& AssetDirPath)
{
	TArray<T*> Assets;
	FAssetRegistryModule& AssetRegistryModule =
		FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
	TArray<FAssetData> AssetData;
	FARFilter Filter;
	Filter.PackagePaths.Add(FName(AssetDirPath));
	AssetRegistryModule.Get().GetAssets(Filter, AssetData);

	for (const FAssetData& Data : AssetData)
	{
		// It is possible to instead use the ClassPath/ClassName in the FARFilter to filter out the
		// asset types of interest, however that approach does not support filtering using base
		// classes of assets which is an annoying limitation. So instead, we get all assets (above)
		// in the directory passed to this function and then filter them ourselves to support
		// finding assets that derives from a certain base class as well.
		UObject* FoundAsset = Data.GetAsset();
		if (FoundAsset != nullptr && FoundAsset->IsA<T>())
		{
			if (T* Asset = Cast<T>(FoundAsset))
				Assets.Add(Asset);
		}
	}

	return Assets;
}
