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

	dtPolyRef PolyRef=0;

    UPROPERTY(EditAnywhere,Category="PolyFlow")
	FVector ProjectExtent = FVector(50.0f, 50.0f, 50.0f);

    UPROPERTY(EditAnywhere, Category = "PolyFlow")
    int32 MaxSearchNeibourNum = 5;

    UPROPERTY(EditAnywhere, Category = "Movement")
	float ForceWeight = 0.1f;

    UPROPERTY(EditAnywhere, Category = "Movement")
	float MaxSpeed = 150.0f;

    UPROPERTY(EditAnywhere, Category = "Movement")
    float Mass = 1.0f;

	UPROPERTY(VisibleAnywhere, Category = "Component")
	UFlowFieldNeiboursSubsystem* NeiboursSubsystem = nullptr;

	dtPolyRef CurrentPolyRef = 0;

};
