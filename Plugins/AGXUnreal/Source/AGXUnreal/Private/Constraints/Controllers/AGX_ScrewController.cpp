// Copyright 2024, Algoryx Simulation AB.

#include "Constraints/Controllers/AGX_ScrewController.h"

#include "Constraints/AGX_ConstraintConstants.h"
#include "Constraints/ControllerConstraintBarriers.h"

FAGX_ConstraintScrewController::FAGX_ConstraintScrewController(bool bRotational)
	: FAGX_ConstraintController(bRotational)
{
}

namespace
{
	FScrewControllerBarrier* GetScrewBarrier(FAGX_ConstraintScrewController& Controller)
	{
		// See comment in GetElectricMotorBarrier in AGX_GetElectricMotorController.cpp
		return static_cast<FScrewControllerBarrier*>(Controller.GetNative());
	}

	const FScrewControllerBarrier* GetScrewBarrier(const FAGX_ConstraintScrewController& Controller)
	{
		// See comment in GetElectricMotorBarrier in AGX_GetElectricMotorController.cpp
		return static_cast<const FScrewControllerBarrier*>(Controller.GetNative());
	}
}

void FAGX_ConstraintScrewController::SetLead(double InLead)
{
	Lead = InLead;
	if (HasNative())
	{
		GetScrewBarrier(*this)->SetLead(Lead);
	}
}

double FAGX_ConstraintScrewController::GetLead() const
{
	if (HasNative())
	{
		return GetScrewBarrier(*this)->GetLead();
	}
	else
	{
		return Lead;
	}
}

void FAGX_ConstraintScrewController::InitializeBarrier(TUniquePtr<FScrewControllerBarrier> Barrier)
{
	check(!HasNative());
	NativeBarrier = std::move(Barrier);
	check(HasNative());
}

void FAGX_ConstraintScrewController::UpdateNativePropertiesImpl()
{
	FScrewControllerBarrier* Barrier = GetScrewBarrier(*this);
	check(Barrier);
	Barrier->SetLead(Lead);
}

void FAGX_ConstraintScrewController::CopyFrom(
	const FScrewControllerBarrier& Source, TArray<FAGX_ConstraintScrewController*>& Instances,
	bool ForceOverwriteInstances)
{
	TArray<FAGX_ConstraintController*> BaseInstances(Instances);
	Super::CopyFrom(Source, BaseInstances, ForceOverwriteInstances);

	for (auto Instance : Instances)
	{
		if (Instance == nullptr)
			continue;

		if (ForceOverwriteInstances || Instance->Lead == Lead)
		{
			Instance->Lead = Source.GetLead();
		}
	}

	Lead = Source.GetLead();
}
