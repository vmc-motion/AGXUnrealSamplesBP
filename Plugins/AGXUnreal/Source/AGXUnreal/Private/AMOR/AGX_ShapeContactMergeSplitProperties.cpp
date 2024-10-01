// Copyright 2024, Algoryx Simulation AB.

#include "AMOR/AGX_ShapeContactMergeSplitProperties.h"

// AGX Dynamics for Unreal includes.
#include "AGX_Check.h"
#include "AGX_LogCategory.h"
#include "AGX_RigidBodyComponent.h"
#include "AMOR/AGX_ShapeContactMergeSplitThresholds.h"
#include "Shapes/AGX_ShapeComponent.h"
#include "Utilities/AGX_NotificationUtilities.h"

template <typename T>
void FAGX_ShapeContactMergeSplitProperties::OnBeginPlayInternal(T& Owner)
{
	if (!Owner.HasNative())
	{
		return;
	}

	AGX_CHECK(!HasNative());

	// Only allocate native if either EnableMerge or EnableSplit is true.
	// Not having a native is a perfectly valid and regular thing for this class.
	if (bEnableMerge || bEnableSplit)
	{
		FAGX_NotificationUtilities::LogWarningIfAmorDisabled("Body or Shape");
		CreateNative(Owner);
		CreateNativeThresholds(Owner.GetWorld());
		UpdateNativeProperties();
	}
}

void FAGX_ShapeContactMergeSplitProperties::OnBeginPlay(UAGX_RigidBodyComponent& Owner)
{
	OnBeginPlayInternal(Owner);
}

void FAGX_ShapeContactMergeSplitProperties::OnBeginPlay(UAGX_ShapeComponent& Owner)
{
	OnBeginPlayInternal(Owner);
}

#if WITH_EDITOR
template <typename T>
void FAGX_ShapeContactMergeSplitProperties::OnPostEditChangePropertyInternal(T& Owner)
{
	if (bEnableMerge || bEnableSplit)
	{
		FAGX_NotificationUtilities::LogWarningIfAmorDisabled("Body or Shape");

		// If we have not yet allocated a native, and we are in Play, and EnableMerge or EnableSplit
		// is true, then we should now allocate a Native.
		if (Owner.HasNative() && !HasNative())
		{
			CreateNative(Owner);
			CreateNativeThresholds(Owner.GetWorld());
		}
	}

	if (HasNative())
	{
		UpdateNativeProperties();
	}
}

void FAGX_ShapeContactMergeSplitProperties::OnPostEditChangeProperty(UAGX_RigidBodyComponent& Owner)
{
	OnPostEditChangePropertyInternal(Owner);
}

void FAGX_ShapeContactMergeSplitProperties::OnPostEditChangeProperty(UAGX_ShapeComponent& Owner)
{
	OnPostEditChangePropertyInternal(Owner);
}
#endif

template <typename T>
void FAGX_ShapeContactMergeSplitProperties::CreateNativeInternal(T& Owner)
{
	AGX_CHECK(Owner.HasNative());
	AGX_CHECK(!HasNative());

	NativeBarrier.AllocateNative(*Owner.GetNative());
}

void FAGX_ShapeContactMergeSplitProperties::CreateNative(UAGX_RigidBodyComponent& Owner)
{
	CreateNativeInternal(Owner);
}

void FAGX_ShapeContactMergeSplitProperties::CreateNative(UAGX_ShapeComponent& Owner)
{
	CreateNativeInternal(Owner);
}

void FAGX_ShapeContactMergeSplitProperties::UpdateNativeProperties()
{
	AGX_CHECK(HasNative());
	NativeBarrier.SetEnableMerge(bEnableMerge);
	NativeBarrier.SetEnableSplit(bEnableSplit);

	UpdateNativeThresholds();
}

void FAGX_ShapeContactMergeSplitProperties::CreateNativeThresholds(UWorld* PlayingWorld)
{
	if (Thresholds == nullptr)
	{
		return;
	}

	UAGX_ShapeContactMergeSplitThresholds* ThresholdsInstance =
		Thresholds->GetOrCreateInstance(PlayingWorld);
	if (ThresholdsInstance == nullptr)
	{
		UE_LOG(
			LogAGX, Warning,
			TEXT("Unable to create a Merge Split Thresholds instance from the "
				 "asset '%s'."),
			*Thresholds->GetName());
		return;
	}

	if (!ThresholdsInstance->HasNative())
	{
		UE_LOG(
			LogAGX, Warning,
			TEXT("Unable to create a Merge Split Thresholds Native from the "
				 "instance '%s'."),
			*ThresholdsInstance->GetName());
	}
}

void FAGX_ShapeContactMergeSplitProperties::UpdateNativeThresholds()
{
	AGX_CHECK(HasNative());
	if (Thresholds == nullptr)
	{
		NativeBarrier.SetShapeContactMergeSplitThresholds(nullptr);
		return;
	}

	UAGX_ShapeContactMergeSplitThresholds* ThresholdsInstance = Thresholds->GetInstance();
	if (ThresholdsInstance == nullptr)
	{
		UE_LOG(
			LogAGX, Warning,
			TEXT("UpdateNativeThresholds called on Thresholds '%s' but it does not have an "
				 "instance. Has CreateNativeThresholds been called?"),
			*Thresholds->GetName());
		return;
	}

	if (Thresholds != ThresholdsInstance)
	{
		Thresholds = ThresholdsInstance;
	}

	FShapeContactMergeSplitThresholdsBarrier* Barrier = ThresholdsInstance->GetNative();
	AGX_CHECK(Barrier);
	NativeBarrier.SetShapeContactMergeSplitThresholds(Barrier);
}

void FAGX_ShapeContactMergeSplitProperties::BindBarrierToOwner(FRigidBodyBarrier& NewOwner)
{
	if (!NewOwner.HasNative())
	{
		NativeBarrier.ReleaseNative();
		return;
	}

	NativeBarrier.BindToNewOwner(NewOwner);
}

void FAGX_ShapeContactMergeSplitProperties::BindBarrierToOwner(FShapeBarrier& NewOwner)
{
	if (!NewOwner.HasNative())
	{
		NativeBarrier.ReleaseNative();
		return;
	}

	NativeBarrier.BindToNewOwner(NewOwner);
}

UAGX_MergeSplitThresholdsBase* FAGX_ShapeContactMergeSplitProperties::GetThresholds()
{
	return Thresholds;
}
