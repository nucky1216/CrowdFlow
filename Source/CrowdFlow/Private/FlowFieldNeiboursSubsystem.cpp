// Fill out your copyright notice in the Description page of Project Settings.


#include "FlowFieldNeiboursSubsystem.h"
#include "FlowFieldVoxelBuilder.h"
#include "EngineUtils.h"


void UFlowFieldNeiboursSubsystem::DebugMap()
{
	UE_LOG(LogTemp, Log, TEXT("Debugging PolyNeibours Map with Num:%d"), PolyNeibours.Num());
	for (auto Pair : PolyNeibours)
	{
		UE_LOG(LogTemp, Log, TEXT("PolyRef %llu has %d entities"), Pair.Key, Pair.Value.Num());
		for (const auto& Entity : Pair.Value)
		{
			UE_LOG(LogTemp, Log, TEXT("---Entity ID: %llu"), Entity.AsNumber());
		}
	}
}

void UFlowFieldNeiboursSubsystem::InitializeManual()
{
	// Initialize the PolyNeibours map
	PolyNeibours.Empty();
	UE_LOG(LogTemp, Log, TEXT("FlowFieldNeiboursSubsystem Initialized with empty PolyNeibours map."));
	DebugMap(); // Optional: Call DebugMap to log the initial state

	for (TActorIterator<AFlowFieldVoxelBuilder> It(GetWorld()); It; ++It)
	{
		FlowFieldBuilder = *It;
		UE_LOG(LogTemp, Log, TEXT("Get the FlowField with name:%s"), *FlowFieldBuilder->GetName());
		break;
	}
	if (!FlowFieldBuilder)
	{
		UE_LOG(LogTemp, Log, TEXT("Failed to Find FlowFieldVoxelBuilder in the world"));
	}
}

void UFlowFieldNeiboursSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	InitializeManual();
}


void UFlowFieldNeiboursSubsystem::RegisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle Entity)
{
	if (PolyRef == 0 )//|| Entity.IsValid() == false)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef or Entity in RegisterPolyEntity"));
		return;
	}
	// Add the entity to the map with the poly reference as the key
	TArray<FMassEntityHandle>* Neibours = PolyNeibours.Find(PolyRef);
	if (Neibours == nullptr)
	{
		// If the poly reference does not exist, create a new entry
		TArray<FMassEntityHandle> NewNeibours;
		NewNeibours.Add(Entity);
		PolyNeibours.Add(PolyRef, NewNeibours);
		//UE_LOG(LogTemp, Log, TEXT("Registered new PolyRef %u with Entity %u"), PolyRef, Entity.AsNumber());
	}
	else
	{
		// If it exists, add the entity to the existing array
		if (!Neibours->Contains(Entity))
		{
			Neibours->Add(Entity);
			//UE_LOG(LogTemp, Log, TEXT("Registered new PolyRef %u with Entity %u"), PolyRef, Entity.AsNumber());
		}
	}
}

void UFlowFieldNeiboursSubsystem::UnregisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle Entity)
{
	if (PolyRef == 0)// || Entity.IsValid() == false)
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
			//UE_LOG(LogTemp, Log, TEXT("Unregistered PolyRef %u with Entity %u"), PolyRef, Entity.AsNumber());
		}
		}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("PolyRef %d not found in UnregisterPolyEntity"), PolyRef);	
	}
}

void UFlowFieldNeiboursSubsystem::UpdatePolyEntity(dtPolyRef NewPolyRef, dtPolyRef OldPolyRef, FMassEntityHandle Entity)
{
	if ((NewPolyRef == 0 && OldPolyRef == 0))
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef or Entity in UpdatePolyEntity"));
		return;
	}
	
	// Unregister the entity from the old poly reference
	UnregisterPolyEntity(OldPolyRef, Entity);
	
	// Register the entity to the new poly reference
	RegisterPolyEntity(NewPolyRef, Entity);
}

TArray<FMassEntityHandle> UFlowFieldNeiboursSubsystem::GetPolyEntities(dtPolyRef PolyRef,int32 MaxNum) const
{
	TArray<FMassEntityHandle> OutNeibours;

	if(PolyRef == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef in GetPolyEntities"));
		return OutNeibours;
	}
	const TArray<FMassEntityHandle>* Neibours = PolyNeibours.Find(PolyRef);
	if (Neibours != nullptr)
	{
		
		OutNeibours.Append(Neibours->GetData(),FMath::Min(Neibours->Num(),MaxNum));

		if(OutNeibours.Num()<MaxNum)
		{
			FNavPolyFlow* CurPoly=FlowFieldBuilder->FlowFieldByPoly.Find(PolyRef);
			if (!CurPoly)
			{
				UE_LOG(LogTemp, Warning, TEXT("PolyRef %llu not found in FlowFieldByPoly"), PolyRef);
				return OutNeibours;
			}
			if (CurPoly->Neibours.Num() == 0)
			{
				UE_LOG(LogTemp, Log, TEXT("The Poly:%llu has No NeibourPoly.Ended with NeibourEntity Num:%d/%d"), PolyRef, OutNeibours.Num(), MaxNum);
			}

			TArray<bool> EmptyFlag;
			EmptyFlag.Init(false, CurPoly->Neibours.Num());
			for (int32 i=0,j=0;;)
			{
				if (OutNeibours.Num() >= MaxNum || !EmptyFlag.Contains(false))
				{
					break;
				}

				if (!EmptyFlag[i])
				{
					dtPolyRef NeibourPoly = CurPoly->Neibours[i];
					const TArray<FMassEntityHandle>* EntitiesOfNeibourPoly = PolyNeibours.Find(NeibourPoly);

					if (!EntitiesOfNeibourPoly)
					{
						UE_LOG(LogTemp, Warning, TEXT("NeibourPoly %llu not found in PolyNeibours"), NeibourPoly);
						EmptyFlag[i] = true;

					}
					else 
					{
						if (OutNeibours.Num() < MaxNum && j < EntitiesOfNeibourPoly->Num())
						{
							OutNeibours.Add((*EntitiesOfNeibourPoly)[j]);
						}
						if (j >= EntitiesOfNeibourPoly->Num())
						{
							EmptyFlag[i] = true;
						}
					}
				}

				if (i + 1 >= CurPoly->Neibours.Num())
				{
					j++;
				}
				i = (i + 1) % CurPoly->Neibours.Num();
			}
		}
		return OutNeibours;
	}
	else 
	{
		UE_LOG(LogTemp, Warning, TEXT("PolyRef %llu not found in GetPolyEntities"), PolyRef);
		return OutNeibours;
	}
	
}


