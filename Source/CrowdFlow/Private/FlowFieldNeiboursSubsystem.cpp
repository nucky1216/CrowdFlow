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
	UE_LOG(LogTemp, Log, TEXT("EntityToActor Map with Num:%d"), EntityToActor.Num());

	for (auto Pair : PolyNeibours)
	{
		UE_LOG(LogTemp, Log, TEXT("PolyRef %llu has %d entities"), Pair.Key, Pair.Value.Num());
		int32 i = 0;
		for (const auto& Entity: Pair.Value)
		{
			//AEntityActor* EntityActor = EntityToActor.FindRef(Entity);
				UE_LOG(LogTemp, Log, TEXT("------EntityHandle:%llu: EntityLoc: %s"), i, Entity.Key,* Entity.Value.ToString());
				i++;
		}
	}
}

void UFlowFieldNeiboursSubsystem::InitializeManual(AFlowFieldVoxelBuilder* FlowField)
{
	// Initialize the PolyNeibours map
	PolyNeibours.Empty();
	EntityToActor.Empty();

	UE_LOG(LogTemp, Log, TEXT("FlowFieldNeiboursSubsystem Initialized with empty PolyNeibours map."));
	

	FlowFieldBuilder = FlowField;

	if (!FlowFieldBuilder)
	{
		UE_LOG(LogTemp, Log, TEXT("Failed to Find FlowFieldVoxelBuilder in the world"));
	}
	for(TActorIterator<AEntityActor>It(GetWorld()); It; ++It)
	{
		AEntityActor* EntityActor = *It;
		if (IsValid(EntityActor))
		{
			EntityActor->ConstructHandle();
			EntityActor->RegistryToSubsystem();
			UE_LOG(LogTemp, Log, TEXT("Found EntityActor: %s, with EntityID: %lld, Handle:%llu"), 
				*EntityActor->GetName(), EntityActor->EntityID,EntityActor->EntityHandle.AsNumber());
			EntityToActor.Add(EntityActor->EntityHandle, EntityActor);
		}
	}

	DebugMap(); 
}

//void UFlowFieldNeiboursSubsystem::Initialize(FSubsystemCollectionBase& Collection)
//{
//	Super::Initialize(Collection);
//	UE_LOG(LogTemp, Log, TEXT("FlowFieldNeiboursSubsystem Initialized."));
//	//FCoreUObjectDelegates::PostLoadMapWithWorld.AddUObject(this, &UFlowFieldNeiboursSubsystem::OnLevelAddedToWorld);
//}

//void UFlowFieldNeiboursSubsystem::Tick(float DeltaTime)
//{
//	UE_LOG(LogTemp, Log, TEXT("FlowFieldNeiboursSubsystem Tick called with EntityToActorNum: %d"), EntityToActor.Num());
//	// Iterate through all EntityActors in the world and update their poly references
//	if(EntityToActor.Num() == 0)
//	{
//		// If there are no EntityActors, we can skip the tick
//		UE_LOG(LogTemp, Log, TEXT("No EntityActors to tick. Skipping tick."));
//		return;
//	}
//	if (!FlowFieldBuilder)
//	{
//		UE_LOG(LogTemp, Warning, TEXT("FlowFieldBuilder is not valid in Tick."));
//		return;
//	}
//	for (auto& EntityPair:EntityToActor)
//	{
//		AEntityActor* EntityActor = EntityPair.Value;
//		if (IsValid(EntityActor))
//		{
//			//UE_LOG(LogTemp, Log, TEXT("Ticking EntityActor: %s with PolyRef: %llu"), *EntityActor->GetName(), EntityActor->CurrentPolyRef);
//			RegistryMassEntity(EntityActor);
//		}
//	}
//	OnEntityRegistered.Broadcast(DeltaTime);
//	
//}


void UFlowFieldNeiboursSubsystem::RegisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle EntityHandle, FVector EntityLoc)
{
	if (PolyRef == 0 )//|| Entity.IsValid() == false)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef or Entity in RegisterPolyEntity"));
		return;
	}
	// Add the entity to the map with the poly reference as the key
	TMap<FMassEntityHandle,FVector>* Neibours = PolyNeibours.Find(PolyRef);
	if (Neibours == nullptr)
	{
		// If the poly reference does not exist, create a new entry
		TMap<FMassEntityHandle, FVector> NewNeibours;
		NewNeibours.Add(EntityHandle,EntityLoc);
		PolyNeibours.Add(PolyRef, NewNeibours);
		//UE_LOG(LogTemp, Log, TEXT("Registered new PolyRef %u with Entity %u"), PolyRef, Entity.AsNumber());
	}
	else
	{
		// If it exists, add the entity to the existing array
		
		Neibours->Add(EntityHandle,EntityLoc);
		//UE_LOG(LogTemp, Log, TEXT("Registered new PolyRef %u with Entity %u"), PolyRef, Entity.AsNumber());
		
	}
}

void UFlowFieldNeiboursSubsystem::UnregisterPolyEntity(dtPolyRef PolyRef, FMassEntityHandle EntityHandle)
{
	if (PolyRef == 0)// || Entity.IsValid() == false)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef or Entity in UnregisterPolyEntity"));
		return;
	}
	
	TMap<FMassEntityHandle,FVector>* Neibours = PolyNeibours.Find(PolyRef);
	if (Neibours != nullptr)
	{
		// Remove the entity from the array if it exists
		if (Neibours->Find(EntityHandle))
		{
			Neibours->Remove(EntityHandle);
			if (Neibours->Num() == 0)
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

void UFlowFieldNeiboursSubsystem::UpdatePolyEntity(dtPolyRef NewPolyRef, dtPolyRef OldPolyRef, FMassEntityHandle EntityHandle,FVector EntityLoc)
{
	if ((NewPolyRef == 0 && OldPolyRef == 0))
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef or Entity in UpdatePolyEntity"));
		return;
	}
	
	// Unregister the entity from the old poly reference
	//UnregisterPolyEntity(OldPolyRef, EntityLoc);
	
	// Register the entity to the new poly reference
	RegisterPolyEntity(NewPolyRef, EntityHandle,EntityLoc);
}

TMap<FMassEntityHandle,FVector> UFlowFieldNeiboursSubsystem::GetPolyEntities(dtPolyRef PolyRef,int32 MaxNum) const
{
	TMap<FMassEntityHandle, FVector> OutNeibours;

	if(!FlowFieldBuilder)
		return OutNeibours;

	if(PolyRef == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Invalid PolyRef in GetPolyEntities"));
		return OutNeibours;
	}
	const TMap<FMassEntityHandle, FVector>* Neibours = PolyNeibours.Find(PolyRef);
	if (Neibours != nullptr)
	{
		int32 i = 0;
		for( auto Pair: *Neibours)
		{
			if(i >= MaxNum)
			{
				break;
			}
			OutNeibours.Add(Pair.Key, Pair.Value);
			i++;
		}

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
				return OutNeibours;
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
					const TMap<FMassEntityHandle,FVector>* EntitiesOfNeibourPoly = PolyNeibours.Find(NeibourPoly);

					if (!EntitiesOfNeibourPoly)
					{
						//UE_LOG(LogTemp, Warning, TEXT("NeibourPoly %llu not found in PolyNeibours"), NeibourPoly);
						EmptyFlag[i] = true;
					}
					else 
					{
						TArray<FMassEntityHandle> KeyArray;
						EntitiesOfNeibourPoly->GenerateKeyArray(KeyArray);
						if (OutNeibours.Num() < MaxNum && j < EntitiesOfNeibourPoly->Num())
						{
							OutNeibours.Add(KeyArray[j], EntitiesOfNeibourPoly->FindRef(KeyArray[j]));
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
	if(!IsValid(Entity))
	{
		UE_LOG(LogTemp, Warning, TEXT("Entity is invalid in RegistryMassEntity"));
		return;
	}
	FMassEntityHandle EntityHandle = Entity->EntityHandle;

	if (!EntityToActor.Find(EntityHandle))
	{
		EntityToActor.Add(EntityHandle, Entity);
	}
	
	if(!FlowFieldBuilder)
	{
		UE_LOG(LogTemp, Warning, TEXT("FlowFieldBuilder is not valid"));
		return;
	}
	FVector EntityLocation = Entity->GetActorLocation();

	dtPolyRef NewEntityPolyRef = FlowFieldBuilder->GetPolyRef(EntityLocation, FVector(FlowFieldBuilder->VoxelSize / 2.0f));
	if (NewEntityPolyRef == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to get PolyRef for entity:%llu at location: %s"),Entity->EntityHandle.AsNumber(), *EntityLocation.ToString());
		return;
	}

	//UE_LOG(LogTemp, Log, TEXT("Entity:%lld registered with NewPolyRef: %llu, OldPolyRef:%llu at location:%s"),
	//						Entity->EntityID, NewEntityPolyRef, Entity->CurrentPolyRef, *EntityLocation.ToString());
	if (Entity->CurrentPolyRef != NewEntityPolyRef)
	{

		UpdatePolyEntity(NewEntityPolyRef, Entity->CurrentPolyRef, Entity->EntityHandle,EntityLocation);

		Entity->SetPolyRef(NewEntityPolyRef);

	}

}

void UFlowFieldNeiboursSubsystem::InitialForMass()
{

	PolyNeibours.Empty();
	EntityToActor.Empty();
	for(TActorIterator<AFlowFieldVoxelBuilder>It(GetWorld()); It; ++It)
	{
		AFlowFieldVoxelBuilder* FlowField = *It;
		if (IsValid(FlowField))
		{
			FlowFieldBuilder = FlowField;
			break;
		}
	}
	if(!IsValid(FlowFieldBuilder))
	{
		UE_LOG(LogTemp, Warning, TEXT("FlowFieldVoxelBuilder not found in the world."));
		return;
	}
	UE_LOG(LogTemp, Log, TEXT("FlowFieldVoxelBuilder found: %s"), *FlowFieldBuilder->GetName());


}




