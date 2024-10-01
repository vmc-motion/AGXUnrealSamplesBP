// Copyright 2024, Algoryx Simulation AB.

#include "AMOR/AGX_WireMergeSplitProperties.h"

// AGX Dynamics for Unreal includes.
#include "AGX_Check.h"
#include "AGX_LogCategory.h"
#include "AMOR/AGX_WireMergeSplitThresholds.h"
#include "Utilities/AGX_NotificationUtilities.h"
#include "Wire/AGX_WireComponent.h"

void FAGX_WireMergeSplitProperties::OnBeginPlay(UAGX_WireComponent& Owner)
{
	AGX_CHECK(Owner.HasNative());
	AGX_CHECK(!HasNative());

	// Only allocate native if either EnableMerge or EnableSplit is true.
	// Not having a native is a perfectly valid and regular thing for this class.
	if (bEnableMerge || bEnableSplit)
	{
		FAGX_NotificationUtilities::LogWarningIfAmorDisabled("Wire");
		CreateNative(Owner);
		CreateNativeThresholds(Owner.GetWorld());
		UpdateNativeProperties();
	}
}

#if WITH_EDITOR
void FAGX_WireMergeSplitProperties::OnPostEditChangeProperty(UAGX_WireComponent& Owner)
{
	if (bEnableMerge || bEnableSplit)
	{
		FAGX_NotificationUtilities::LogWarningIfAmorDisabled("Wire");
		if (Owner.HasNative() && !HasNative())
		{
			// If we have not yet allocated a native, and we are in Play, and EnableMerge or
			// EnableSplit is true, then we should now allocate a Native.
			CreateNative(Owner);
			CreateNativeThresholds(Owner.GetWorld());
		}
	}

	if (HasNative())
	{
		UpdateNativeProperties();
	}
}
#endif

void FAGX_WireMergeSplitProperties::CreateNative(UAGX_WireComponent& Owner)
{
	AGX_CHECK(Owner.HasNative());
	AGX_CHECK(!HasNative());

	NativeBarrier.AllocateNative(*Owner.GetNative());
}

void FAGX_WireMergeSplitProperties::BindBarrierToOwner(FWireBarrier& NewOwner)
{
	if (!NewOwner.HasNative())
	{
		NativeBarrier.ReleaseNative();
		return;
	}

	NativeBarrier.BindToNewOwner(NewOwner);
}

void FAGX_WireMergeSplitProperties::UpdateNativeProperties()
{
	AGX_CHECK(HasNative());
	NativeBarrier.SetEnableMerge(bEnableMerge);
	NativeBarrier.SetEnableSplit(bEnableSplit);

	UpdateNativeThresholds();
}

void FAGX_WireMergeSplitProperties::CreateNativeThresholds(UWorld* PlayingWorld)
{
	if (Thresholds == nullptr)
	{
		return;
	}

	UAGX_WireMergeSplitThresholds* ThresholdsInstance =
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

void FAGX_WireMergeSplitProperties::UpdateNativeThresholds()
{
	AGX_CHECK(HasNative());
	if (Thresholds == nullptr)
	{
		NativeBarrier.SetWireMergeSplitThresholds(nullptr);
		return;
	}

	UAGX_WireMergeSplitThresholds* ThresholdsInstance = Thresholds->GetInstance();
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

	FWireMergeSplitThresholdsBarrier* Barrier = ThresholdsInstance->GetNative();
	AGX_CHECK(Barrier);
	NativeBarrier.SetWireMergeSplitThresholds(Barrier);
}

UAGX_MergeSplitThresholdsBase* FAGX_WireMergeSplitProperties::GetThresholds()
{
	return Thresholds;
}
