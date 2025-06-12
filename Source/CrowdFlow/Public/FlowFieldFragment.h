// FlowFieldRefFragment.h
#pragma once

#include "CoreMinimal.h"
#include "FlowFieldVoxelBuilder.h"
#include "FlowFieldFragment.generated.h"

USTRUCT()
struct CROWDFLOW_API FFlowFieldFragment : public FMassFragment
{
    GENERATED_BODY()

    AFlowFieldVoxelBuilder* FlowField = nullptr;

    UPROPERTY(EditAnywhere,Category="Force")
	float ForceIntensity = 300.0f;

    UPROPERTY(EditAnywhere, Category = "Force")
	float ForceWeight = 0.1;

};
