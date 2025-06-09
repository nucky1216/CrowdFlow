// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassEntitySpawnDataGeneratorBase.h"
#include "FlowFieldVoxelBuilder.h"
#include "CrowdGenerator.generated.h"

/**
 * 
 */
class AFlowFieldVoxelBuilder;
UCLASS()
class CROWDFLOW_API UCrowdGenerator : public UMassEntitySpawnDataGeneratorBase
{
    GENERATED_BODY()

public:
    // 在编辑器中选择 FlowFieldBuilder
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
    AFlowFieldVoxelBuilder* FlowFieldBuilder;

    // 生成点类型
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Crowd")
    int32 PolyIndex = 0;


    virtual void Generate(UObject& QueryOwner, TConstArrayView<FMassSpawnedEntityType> EntityTypes, int32 Count, FFinishedGeneratingSpawnDataSignature& FinishedGeneratingSpawnPointsDelegate) const override;
};
