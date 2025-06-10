// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassObserverProcessor.h"
#include "MassProcessor.h"
#include "FlowFieldAssignProcessor.generated.h"

/**
 * 
 */
UCLASS()
class CROWDFLOW_API UFlowFieldAssignProcessor : public UMassObserverProcessor
{
	GENERATED_BODY()

public:
    UFlowFieldAssignProcessor();

protected:
    virtual void ConfigureQueries() override;
    virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;

    FMassEntityQuery EntityQuery;
	
};
