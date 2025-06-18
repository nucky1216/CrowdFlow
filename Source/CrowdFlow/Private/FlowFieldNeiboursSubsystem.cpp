// Fill out your copyright notice in the Description page of Project Settings.


#include "FlowFieldNeiboursSubsystem.h"

void UFlowFieldNeiboursSubsystem::RegisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle Entity)
{
	if (PolyRef == 0 || Entity.IsValid() == false)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef or Entity in RegisterPolyEntity"));
		return;
	}
	// Add the entity to the map with the poly reference as the key
	TArray<FMassEntityHandle>* Neobours = PolyNeibours.Find(PolyRef);
	if (Neobours == nullptr)
	{
		// If the poly reference does not exist, create a new entry
		TArray<FMassEntityHandle> NewNeibours;
		NewNeibours.Add(Entity);
		PolyNeibours.Add(PolyRef, NewNeibours);
	}
	else
	{
		// If it exists, add the entity to the existing array
		if (!Neobours->Contains(Entity))
		{
			Neobours->Add(Entity);
		}
	}
}

void UFlowFieldNeiboursSubsystem::UnregisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle Entity)
{
	if (PolyRef == 0 || Entity.IsValid() == false)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef or Entity in UnregisterPolyEntity"));
		return;
	}
	
	TArray<FMassEntityHandle>* Neobours = PolyNeibours.Find(PolyRef);
	if (Neobours != nullptr)
	{
		// Remove the entity from the array if it exists
		if (Neobours->Contains(Entity))
		{
			Neobours->Remove(Entity);
			if (Neobours->Num() == 0)
			{
				// If no entities left, remove the poly reference from the map
				PolyNeibours.Remove(PolyRef);
			}
		}
		}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("PolyRef %d not found in UnregisterPolyEntity"), PolyRef);	
	}
}

const TArray<FMassEntityHandle>* UFlowFieldNeiboursSubsystem::GetPolyEntities(dtPolyRef PolyRef) const
{
	if(PolyRef == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef in GetPolyEntities"));
		return nullptr;
	}
	const TArray<FMassEntityHandle>* Neibours = PolyNeibours.Find(PolyRef);
	if (Neibours != nullptr)
	{
		return Neibours;
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("PolyRef %d not found in GetPolyEntities"), PolyRef);
		return nullptr;
	}
	
}
