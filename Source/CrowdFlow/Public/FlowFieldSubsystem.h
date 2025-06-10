// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "FlowFieldVoxelBuilder.h"
#include "FlowFieldSubsystem.generated.h"

/**
 * 
 */
UCLASS()
class CROWDFLOW_API UFlowFieldSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	AFlowFieldVoxelBuilder* FlowFieldInstance = nullptr;

	void SetFlowField(AFlowFieldVoxelBuilder* InField) { FlowFieldInstance = InField; }
	AFlowFieldVoxelBuilder* GetFlowField() const { return FlowFieldInstance; }
	
};
