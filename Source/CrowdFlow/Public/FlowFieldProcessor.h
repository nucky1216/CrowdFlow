// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassProcessor.h"
#include "FlowFieldVoxelBuilder.h"
#include "FlowFieldProcessor.generated.h"

/**
 * 
 */
UCLASS()
class CROWDFLOW_API UFlowFieldProcessor : public UMassProcessor
{
	GENERATED_BODY()

public:
    UFlowFieldProcessor();

    UPROPERTY(EditAnywhere, Category = "Flow Field")
    float ForceStrength = 500.f;


protected:
    virtual void ConfigureQueries() override;
    virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;

    


private:
    FMassEntityQuery EntityQuery;
};
