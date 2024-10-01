// Copyright 2024, Algoryx Simulation AB.

#pragma once

#include "Constraints/Constraint1DOFBarrier.h"

class FRigidBodyBarrier;

class AGXUNREALBARRIER_API FDistanceJointBarrier : public FConstraint1DOFBarrier
{
public:
	FDistanceJointBarrier();
	FDistanceJointBarrier(FDistanceJointBarrier&& Other) = default;
	FDistanceJointBarrier(std::unique_ptr<FConstraintRef> Native);
	virtual ~FDistanceJointBarrier();

private:
	virtual void AllocateNativeImpl(
		const FRigidBodyBarrier& Rb1, const FVector& FramePosition1, const FQuat& FrameRotation1,
		const FRigidBodyBarrier* Rb2, const FVector& FramePosition2,
		const FQuat& FrameRotation2) override;

private:
	FDistanceJointBarrier(const FDistanceJointBarrier&) = delete;
	void operator=(const FDistanceJointBarrier&) = delete;
};
