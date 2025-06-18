// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassProcessor.h"
#include "FillNeiboursProcessor.generated.h"

/**
 * 
 */
UCLASS()
class CROWDFLOW_API UFillNeiboursProcessor : public UMassProcessor
{
	GENERATED_BODY()

public:
	UFillNeiboursProcessor();

protected:
	virtual void ConfigureQueries() override;
	virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;
	
	FMassEntityQuery EntityQuery;
};
