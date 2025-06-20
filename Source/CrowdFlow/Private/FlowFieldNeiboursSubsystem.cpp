// Fill out your copyright notice in the Description page of Project Settings.


#include "FlowFieldNeiboursSubsystem.h"
#include "FlowFieldVoxelBuilder.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "Engine/Level.h"
#include "Engine/LevelStreaming.h"
#include "Engine/WorldComposition.h"
#include "Engine/Engine.h"


void UFlowFieldNeiboursSubsystem::DebugMap()
{
	UE_LOG(LogTemp, Log, TEXT("Debugging PolyNeibours Map with Num:%d"), PolyNeibours.Num());
	for (auto Pair : PolyNeibours)
	{
		UE_LOG(LogTemp, Log, TEXT("PolyRef %llu has %d entities"), Pair.Key, Pair.Value.Num());
		for (const auto& Entity : Pair.Value)
		{
			AEntityActor* EntityActor = EntityToActor.FindRef(Entity);
			if (EntityActor!=nullptr)
			{
				UE_LOG(LogTemp, Log, TEXT("------Entity ID: %llu, EntityActor: %s"), Entity.AsNumber(), *EntityActor->GetName());
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("------Entity ID: %llu not found in EntityToActor map."), Entity.AsNumber());
			}
}
	}
}

void UFlowFieldNeiboursSubsystem::InitializeManual(AFlowFieldVoxelBuilder* FlowField)
{
	// Initialize the PolyNeibours map
	PolyNeibours.Empty();
	EntityToActor.Empty();

	UE_LOG(LogTemp, Log, TEXT("FlowFieldNeiboursSubsystem Initialized with empty PolyNeibours map."));
	DebugMap(); // Optional: Call DebugMap to log the initial state

	FlowFieldBuilder = FlowField;

	if (!FlowFieldBuilder)
	{
		UE_LOG(LogTemp, Log, TEXT("Failed to Find FlowFieldVoxelBuilder in the world"));
	}
}

void UFlowFieldNeiboursSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	UE_LOG(LogTemp, Log, TEXT("FlowFieldNeiboursSubsystem Initialized."));
	//FCoreUObjectDelegates::PostLoadMapWithWorld.AddUObject(this, &UFlowFieldNeiboursSubsystem::OnLevelAddedToWorld);
}

void UFlowFieldNeiboursSubsystem::Tick(float DeltaTime)
{
	
	// Iterate through all EntityActors in the world and update their poly references
	for (auto& EntityPair:EntityToActor)
	{
		AEntityActor* EntityActor = EntityPair.Value;
		if (EntityActor)
		{
			RegistryMassEntity(EntityActor);
		}
	}
	OnEntityRegistered.Broadcast(DeltaTime);
	
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

	if(!FlowFieldBuilder)
		return OutNeibours;

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
				//UE_LOG(LogTemp, Warning, TEXT("PolyRef %llu not found in FlowFieldByPoly"), PolyRef);
				return OutNeibours;
			}
			if (CurPoly->VertNeibours.Num() == 0)
			{
				UE_LOG(LogTemp, Log, TEXT("The Poly:%llu has No NeibourPoly.Ended with NeibourEntity Num:%d/%d"), PolyRef, OutNeibours.Num(), MaxNum);
			}

			TArray<bool> EmptyFlag;
			EmptyFlag.Init(false, CurPoly->VertNeibours.Num());
			for (int32 i=0,j=0;;)
			{
				if (OutNeibours.Num() >= MaxNum || !EmptyFlag.Contains(false))
				{
					break;
				}

				if (!EmptyFlag[i])
				{
					dtPolyRef NeibourPoly = CurPoly->VertNeibours[i];
					const TArray<FMassEntityHandle>* EntitiesOfNeibourPoly = PolyNeibours.Find(NeibourPoly);

					if (!EntitiesOfNeibourPoly)
					{
						//UE_LOG(LogTemp, Warning, TEXT("NeibourPoly %llu not found in PolyNeibours"), NeibourPoly);
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

				if (i + 1 >= CurPoly->VertNeibours.Num())
				{
					j++;
				}
				i = (i + 1) % CurPoly->VertNeibours.Num();
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

void UFlowFieldNeiboursSubsystem::RegistryMassEntity(AEntityActor* Entity)
{

	FMassEntityHandle EntityHandle = Entity->EntityHandle;

	if (!EntityToActor.Find(EntityHandle))
	{
		EntityToActor.Add(EntityHandle, Entity);
	}
	
	if(!FlowFieldBuilder)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlowFieldBuilder is not valid or EntityHandle is invalid."));
		return;
	}
	FVector EntityLocation = Entity->GetActorLocation();

	dtPolyRef NewEntityPolyRef = FlowFieldBuilder->GetPolyRef(Entity->GetActorLocation(), FVector(FlowFieldBuilder->VoxelSize / 2.0f));
	if (NewEntityPolyRef == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to get PolyRef for entity at location: %s"), *EntityLocation.ToString());
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("Entity %s:%d registered with NewPolyRef: %llu, OldPolyRef:%llu"),
		*Entity->GetName(), Entity->EntityID, NewEntityPolyRef, Entity->CurrentPolyRef);
	if (Entity->CurrentPolyRef != NewEntityPolyRef)
	{

		UpdatePolyEntity(NewEntityPolyRef, Entity->CurrentPolyRef, EntityHandle);

		Entity->SetPolyRef(NewEntityPolyRef);

	}


}


