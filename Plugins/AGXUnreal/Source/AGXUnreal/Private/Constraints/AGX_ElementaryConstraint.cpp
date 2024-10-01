// Copyright 2024, Algoryx Simulation AB.

#include "Constraints/AGX_ElementaryConstraint.h"

// AGX Dynamics for Unreal includes.
#include "Utilities/AGX_ObjectUtilities.h"

FAGX_ElementaryConstraint::FAGX_ElementaryConstraint()
{
}

FAGX_ElementaryConstraint::~FAGX_ElementaryConstraint()
{
}

void FAGX_ElementaryConstraint::SetEnable(bool bInEnable)
{
	if (HasNative())
	{
		Barrier.SetEnabled(bInEnable);
	}
	bEnable = bInEnable;
}

bool FAGX_ElementaryConstraint::GetEnable() const
{
	if (HasNative())
	{
		return Barrier.GetEnabled();
	}
	return bEnable;
}

void FAGX_ElementaryConstraint::SetCompliance(double InCompliance)
{
	if (HasNative())
	{
		Barrier.SetCompliance(InCompliance);
	}
	Compliance = InCompliance;
}

double FAGX_ElementaryConstraint::GetCompliance() const
{
	if (HasNative())
	{
		return Barrier.GetCompliance();
	}
	return Compliance;
}

void FAGX_ElementaryConstraint::SetElasticity(double InElasticity)
{
	if (HasNative())
	{
		// Elasticity is a dependent property on compliance.
		Barrier.SetElasticity(InElasticity);
		Compliance = Barrier.GetCompliance();
	}
	else
	{
		// No Native so can't let AGX Dynamics do the job for us. This is a re-implementation of
		// what AGX Dynamics would have done if we had called it. I don't like this code duplication
		// much. What if the implementation in AGX Dynamics is ever changed? How to we assure this
		// code is changed accordingly?
		if (InElasticity > DBL_EPSILON)
		{
			Compliance = 1.0 / InElasticity;
		}
	}
}

double FAGX_ElementaryConstraint::GetElasticity() const
{
	if (HasNative())
	{
		return Barrier.GetElasticity();
	}

	// No Native so can't let AGX Dynamics do the job for us. This is a re-implementation of
	// what AGX Dynamics would have done if we had called it. I don't like this code duplication
	// much. What if the implementation in AGX Dynamics is ever changed? How to we assure this
	// code is changed accordingly?
	if (Compliance > DBL_EPSILON)
	{
		return 1.0 / Compliance;
	}
	else
	{
		return std::numeric_limits<double>::infinity();
	}
}

void FAGX_ElementaryConstraint::SetSpookDamping(double InSpookDamping)
{
	if (HasNative())
	{
		Barrier.SetSpookDamping(InSpookDamping);
	}
	SpookDamping = InSpookDamping;
}

double FAGX_ElementaryConstraint::GetSpookDamping() const
{
	if (HasNative())
	{
		return Barrier.GetSpookDamping();
	}
	return SpookDamping;
}

void FAGX_ElementaryConstraint::SetForceRange(const FDoubleInterval& InForceRange)
{
	if (HasNative())
	{
		Barrier.SetForceRange(InForceRange);
	}
	ForceRange = InForceRange;
}

void FAGX_ElementaryConstraint::SetForceRange(const FAGX_RealInterval& InForceRange)
{
	SetForceRange(InForceRange.ToDouble());
}

void FAGX_ElementaryConstraint::SetForceRange(double InForceRangeMin, double InForceRangeMax)
{
	SetForceRange(FDoubleInterval {InForceRangeMin, InForceRangeMax});
}

void FAGX_ElementaryConstraint::SetForceRangeMin(double InMinForce)
{
	if (HasNative())
	{
		Barrier.SetForceRangeMin(InMinForce);
	}
	ForceRange.Min = InMinForce;
}

void FAGX_ElementaryConstraint::SetForceRangeMax(double InMaxForce)
{
	if (HasNative())
	{
		Barrier.SetForceRangeMax(InMaxForce);
	}
	ForceRange.Max = InMaxForce;
}

FDoubleInterval FAGX_ElementaryConstraint::GetForceRange() const
{
	if (HasNative())
	{
		return Barrier.GetForceRange();
	}
	return ForceRange;
}

double FAGX_ElementaryConstraint::GetForceRangeMin() const
{
	if (HasNative())
	{
		return Barrier.GetForceRangeMin();
	}
	return ForceRange.Min;
}

double FAGX_ElementaryConstraint::GetForceRangeMax() const
{
	if (HasNative())
	{
		return Barrier.GetForceRangeMax();
	}
	return ForceRange.Max;
}

double FAGX_ElementaryConstraint::GetForce()
{
	if (HasNative())
	{
		return Barrier.GetForce();
	}
	return 0.0;
}

bool FAGX_ElementaryConstraint::IsActive() const
{
	if (HasNative())
	{
		return Barrier.IsActive();
	}
	return false;
}

FString FAGX_ElementaryConstraint::GetName() const
{
	if (HasNative())
	{
		return Barrier.GetName();
	}
	return FName(NAME_None).ToString();
}

bool FAGX_ElementaryConstraint::HasNative() const
{
	return Barrier.HasNative();
}

FElementaryConstraintBarrier* FAGX_ElementaryConstraint::GetNative()
{
	if (!HasNative())
	{
		return nullptr;
	}
	return &Barrier;
}

const FElementaryConstraintBarrier* FAGX_ElementaryConstraint::GetNative() const
{
	if (!HasNative())
	{
		return nullptr;
	}
	return &Barrier;
}

void FAGX_ElementaryConstraint::InitializeBarrier(const FElementaryConstraintBarrier& InBarrier)
{
	check(!HasNative());
	Barrier = InBarrier;
	check(HasNative());
}

void FAGX_ElementaryConstraint::UpdateNativeProperties()
{
	check(HasNative());
	Barrier.SetEnabled(bEnable);
	Barrier.SetCompliance(Compliance);
	Barrier.SetSpookDamping(SpookDamping);
	Barrier.SetForceRange(ForceRange);
}

void FAGX_ElementaryConstraint::CopyFrom(
	const FElementaryConstraintBarrier& Source,
	TArray<FAGX_ElementaryConstraint*>& ArchetypeInstances, bool bForceOverwriteInstances)
{
	const bool bEnableBarrier = Source.GetEnabled();
	const FAGX_Real ComplianceBarrier = Source.GetCompliance();
	const FAGX_Real SpookDampingBarrier = Source.GetSpookDamping();
	const FAGX_RealInterval ForceRangeBarrier = Source.GetForceRange();

	for (auto Instance : ArchetypeInstances)
	{
		if (Instance == nullptr)
		{
			continue;
		}

		if (bForceOverwriteInstances)
		{
			Instance->bEnable = bEnableBarrier;
			Instance->Compliance = ComplianceBarrier;
			Instance->SpookDamping = SpookDampingBarrier;
			Instance->ForceRange = ForceRangeBarrier;
		}
		else
		{
			FAGX_ObjectUtilities::SetIfEqual(Instance->bEnable, bEnable, bEnableBarrier);
			FAGX_ObjectUtilities::SetIfEqual(Instance->Compliance, Compliance, ComplianceBarrier);
			FAGX_ObjectUtilities::SetIfEqual(
				Instance->SpookDamping, SpookDamping, SpookDampingBarrier);
			FAGX_ObjectUtilities::SetIfEqual(Instance->ForceRange, ForceRange, ForceRangeBarrier);
		}
	}

	bEnable = bEnableBarrier;
	Compliance = ComplianceBarrier;
	SpookDamping = SpookDampingBarrier;
	ForceRange = ForceRangeBarrier;
}
