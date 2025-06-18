

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "DeltaMove.generated.h"

// This class does not need to be modified.
UINTERFACE(MinimalAPI)
class UDeltaMove : public UInterface
{
	GENERATED_BODY()
};

/**
 * 
 */
class CROWDFLOW_API IDeltaMove
{
	GENERATED_BODY()

	// Add interface functions to this class. This is the class that will be inherited to implement this interface.
public:
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "FlowField")
	FVector ForceToMove(float DeltaTime,FVector Velocity);

	
	 virtual FVector ForceToMove_Implementation(float DeltaTime,FVector Velocity);
	
};
