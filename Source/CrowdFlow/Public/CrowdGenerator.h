// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassEntitySpawnDataGeneratorBase.h"
#include "FlowFieldVoxelBuilder.h"
#include "CrowdGenerator.generated.h"

/**
 * 
 */

UCLASS()
class CROWDFLOW_API UCrowdGenerator : public UMassEntitySpawnDataGeneratorBase
{
    GENERATED_BODY()

public:
    // ÔÚ±à¼­Æ÷ÖÐÑ¡Ôñ FlowFieldBuilder
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
    AFlowFieldVoxelBuilder* FlowFieldBuilder;



    virtual void Generate(UObject& QueryOwner, TConstArrayView<FMassSpawnedEntityType> EntityTypes, int32 Count, FFinishedGeneratingSpawnDataSignature& FinishedGeneratingSpawnPointsDelegate) const override;
};
