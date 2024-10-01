// Copyright 2024, Algoryx Simulation AB.

#include "Constraints/Controllers/AGX_FrictionController.h"

#include "Constraints/ControllerConstraintBarriers.h"

FAGX_ConstraintFrictionController::FAGX_ConstraintFrictionController(bool bRotational)
	: FAGX_ConstraintController(bRotational)
{
}

void FAGX_ConstraintFrictionController::InitializeBarrier(
	TUniquePtr<FFrictionControllerBarrier> Barrier)
{
	check(!HasNative());
	NativeBarrier = std::move(Barrier);
	check(HasNative());
}

namespace
{
	FFrictionControllerBarrier* GetFrictionBarrier(FAGX_ConstraintFrictionController& Controller)
	{
		// See comment in GetElectricMotorController.
		return static_cast<FFrictionControllerBarrier*>(Controller.GetNative());
	}

	const FFrictionControllerBarrier* GetFrictionBarrier(
		const FAGX_ConstraintFrictionController& Controller)
	{
		// See comment in GetElectricMotorController.
		return static_cast<const FFrictionControllerBarrier*>(Controller.GetNative());
	}
}

void FAGX_ConstraintFrictionController::SetFrictionCoefficient(double InFrictionCoefficient)
{
	if (HasNative())
	{
		GetFrictionBarrier(*this)->SetFrictionCoefficient(InFrictionCoefficient);
	}
	FrictionCoefficient = InFrictionCoefficient;
}

double FAGX_ConstraintFrictionController::GetFrictionCoefficient() const
{
	if (HasNative())
	{
		return GetFrictionBarrier(*this)->GetFrictionCoefficient();
	}
	else
	{
		return FrictionCoefficient;
	}
}

void FAGX_ConstraintFrictionController::SetEnableNonLinearDirectSolveUpdate(
	bool bInEnableNonLinearDirectSolveUpdate)
{
	if (HasNative())
	{
		GetFrictionBarrier(*this)->SetEnableNonLinearDirectSolveUpdate(
			bInEnableNonLinearDirectSolveUpdate);
	}
	bEnableNonLinearDirectSolveUpdate = bInEnableNonLinearDirectSolveUpdate;
}

bool FAGX_ConstraintFrictionController::GetEnableNonLinearDirectSolveUpdate() const
{
	if (HasNative())
	{
		return GetFrictionBarrier(*this)->GetEnableNonLinearDirectSolveUpdate();
	}
	else
	{
		return bEnableNonLinearDirectSolveUpdate;
	}
}

void FAGX_ConstraintFrictionController::UpdateNativePropertiesImpl()
{
	FFrictionControllerBarrier* Barrier = GetFrictionBarrier(*this);
	check(Barrier);
	Barrier->SetFrictionCoefficient(FrictionCoefficient);
	Barrier->SetEnableNonLinearDirectSolveUpdate(bEnableNonLinearDirectSolveUpdate);
}

void FAGX_ConstraintFrictionController::CopyFrom(
	const FFrictionControllerBarrier& Source, TArray<FAGX_ConstraintFrictionController*>& Instances,
	bool ForceOverwriteInstances)
{
	TArray<FAGX_ConstraintController*> BaseInstances(Instances);
	Super::CopyFrom(Source, BaseInstances, ForceOverwriteInstances);

	for (auto Instance : Instances)
	{
		if (Instance == nullptr)
			continue;

		if (ForceOverwriteInstances || Instance->FrictionCoefficient == FrictionCoefficient)
			Instance->FrictionCoefficient = Source.GetFrictionCoefficient();

		if (ForceOverwriteInstances ||
			Instance->bEnableNonLinearDirectSolveUpdate == bEnableNonLinearDirectSolveUpdate)
			Instance->bEnableNonLinearDirectSolveUpdate =
				Source.GetEnableNonLinearDirectSolveUpdate();
	}

	FrictionCoefficient = Source.GetFrictionCoefficient();
	bEnableNonLinearDirectSolveUpdate = Source.GetEnableNonLinearDirectSolveUpdate();
}
