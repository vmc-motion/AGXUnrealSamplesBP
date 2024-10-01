// Copyright 2024, Algoryx Simulation AB.

#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

/**
 * Detail Customization of AGX_Constraint, which does the following:
 *
 * - Orders the Categories by importance.
 *
 */
class AGXUNREALEDITOR_API FAGX_ConstraintCustomization : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};
