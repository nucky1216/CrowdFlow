// Fill out your copyright notice in the Description page of Project Settings.


#include "FlowFieldVoxelBuilder.h"
#include "NavigationSystem.h"
#include "NavigationPath.h"
#include "NavMesh/RecastNavMesh.h"
#include "NavMesh/NavMeshRenderingComponent.h"
#include "DrawDebugHelpers.h"

#include "NavMesh/RecastHelpers.h"

#include "Detour/DetourNavMesh.h"


// Sets default values
AFlowFieldVoxelBuilder::AFlowFieldVoxelBuilder()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
    ConstructNeibourOffsets();

}

// Called when the game starts or when spawned
void AFlowFieldVoxelBuilder::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AFlowFieldVoxelBuilder::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}
void AFlowFieldVoxelBuilder::OnConstruction(const FTransform& Transform)
{
    ConstructNeibourOffsets();
    GenerateFlowField();
   // DebugDrawFlowField();
}

FIntVector AFlowFieldVoxelBuilder::GetGridIndex(const FVector& Location) const
{
    FVector Local = Location - GetActorLocation();
    return FIntVector(
        FMath::RoundToInt(Local.X / VoxelSize),
        FMath::RoundToInt(Local.Y / VoxelSize),
        FMath::RoundToInt(Local.Z / VoxelSize));
}

FVector AFlowFieldVoxelBuilder::SampleDirction(const FVector& Location) 
{
	FIntVector GridIndex = GetGridIndex(Location);
	FVector Direction = FVector::ZeroVector;
	float TotalWeight = 0.0f;

    for(auto&  offset:IndexOffset)
    {

        FFlowVoxel* Voxel = FlowField.Find(GridIndex + offset);

        if (Voxel && Voxel->bIsValid)
        {
			float distance = FVector::Dist(Voxel->WorldPosition, Location);
            if (distance == 0)
                continue;
			float Weight = 1.0f / distance; 
            Direction= Weight*Voxel->FlowDirection;
			TotalWeight += Weight;

        }
	}
    if (TotalWeight > 0.0f)
    {
		Direction /= TotalWeight; 
        Direction=Direction.GetSafeNormal();// Normalize the direction
    }
    return Direction;
}

void AFlowFieldVoxelBuilder::ConstructNeibourOffsets()
{
	IndexOffset.Empty();
    // 6 directions: up, down, left, right, forward, backward
    IndexOffset.Add(FIntVector(1, 0, 0));  // Right
    IndexOffset.Add(FIntVector(-1, 0, 0)); // Left
    IndexOffset.Add(FIntVector(0, 1, 0));  // Forward
    IndexOffset.Add(FIntVector(0, -1, 0)); // Backward
    IndexOffset.Add(FIntVector(0, 0, 1));  // Up
    IndexOffset.Add(FIntVector(0, 0, -1)); // Down

    switch (NeibourType)
    {

        case EFlowFieldNeibourType::EdgeNeibour:
            // 6 face neighbors + 26 vertex neighbors
            IndexOffset.Add(FIntVector(1, 1, 0));  
            IndexOffset.Add(FIntVector(-1, -1, 0));
            IndexOffset.Add(FIntVector(1, -1, 0));
            IndexOffset.Add(FIntVector(-1, 1, 0));

            IndexOffset.Add(FIntVector(1, 0, 1));
		    IndexOffset.Add(FIntVector(-1, 0, -1));
            IndexOffset.Add(FIntVector(1, 0, -1));
            IndexOffset.Add(FIntVector(-1, 0, 1));

            IndexOffset.Add(FIntVector(0, 1, 1));
            IndexOffset.Add(FIntVector(0, -1, -1));
            IndexOffset.Add(FIntVector(0, 1, -1));
            IndexOffset.Add(FIntVector(0, -1, 1));

            break;
        case EFlowFieldNeibourType::VertNeibour:
            // 26 directions: all neighbors in a cube
            IndexOffset.Empty();
            for (int32 x = -1; x <= 1; ++x)
                for (int32 y = -1; y <= 1; ++y)
                    for (int32 z = -1; z <= 1; ++z)
                    {
                        if (x == 0 && y == 0 && z == 0)
                            continue; // skip self
                        IndexOffset.Add(FIntVector(x, y, z));
                    }
            break;

        case EFlowFieldNeibourType::FaceNeibour:
        default:
            break;
    }


}

void AFlowFieldVoxelBuilder::GenerateFlowField()
{
    FlowField.Empty();

    UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(GetWorld());
    if (!NavSys) return;

    FVector Origin = GetActorLocation();
    int32 XCount = FMath::CeilToInt(VolumeSize.X / VoxelSize);
    int32 YCount = FMath::CeilToInt(VolumeSize.Y / VoxelSize);
    int32 ZCount = FMath::CeilToInt(VolumeSize.Z / VoxelSize);

    for (int32 X = -XCount / 2; X <= XCount / 2; ++X)
    {
        for (int32 Y = -YCount / 2; Y <= YCount / 2; ++Y)
        {
            for (int32 Z = -ZCount / 2; Z <= ZCount / 2; ++Z)
            {
                FVector SamplePos = Origin + FVector(X, Y, Z) * VoxelSize;
                FNavLocation ProjectedLocation;
                
                bool bNav = NavSys->ProjectPointToNavigation(SamplePos, ProjectedLocation,FVector(VoxelSize/2.0));

                if (bNav)
                {
                    FFlowVoxel Voxel;
                    Voxel.WorldPosition = SamplePos;
                    Voxel.bIsValid = bNav;
                    Voxel.Heat = 0;
                    FlowField.Add(FIntVector(X, Y, Z), Voxel);
                }

            }
        }
    }

    //Caculate the Heat
    FIntVector StartIdx= GetGridIndex(TargetLocation);

	TQueue<FIntVector> Queue;
	Queue.Enqueue(StartIdx);
    while (!Queue.IsEmpty())
    {
        FIntVector Current;
        Queue.Dequeue(Current);
		FFlowVoxel* CurrentVoxel = FlowField.Find(Current);
        int32 CurrentHeat = 0;
        if(CurrentVoxel)
        {
            CurrentHeat = CurrentVoxel->Heat;
            for (auto& Offset : IndexOffset)
            {
                FFlowVoxel* Neibour = FlowField.Find(Current + Offset);
                //UE_LOG(LogTemp, Log, TEXT("Current: %s Neibour: %s"), *Current.ToString(), *(Current + Offset).ToString());
                if (Neibour && Neibour->Heat == 0)
                {
                    Neibour->Heat = 1 + CurrentHeat;
                    Queue.Enqueue(Current + Offset);
                }
            }
        }
    }

	//Caculate Flow Direction
    for(auto& Elem : FlowField)
    {
        FFlowVoxel& Voxel = Elem.Value;
		
        if (Voxel.bIsValid)
        {
            FVector BestDirection = FVector::ZeroVector;
            int32 BestHeat = Voxel.Heat;
            for (auto Offset : IndexOffset)
            {
                FFlowVoxel* Neibour = FlowField.Find(Elem.Key + Offset);
                if (Neibour && Neibour->bIsValid && Neibour->Heat < BestHeat)
                {
                    BestDirection = FVector(Offset.X, Offset.Y, Offset.Z);
                    BestHeat = Neibour->Heat;
                }
            }
            if (!BestDirection.IsZero())
            {
                Voxel.FlowDirection = BestDirection.GetSafeNormal();
            }
        }
	}

}

void AFlowFieldVoxelBuilder::DebugDrawFlowField()
{
    int32 count = 0;
    TArray<FIntVector> KeyValue;
    FlowField.GenerateKeyArray(KeyValue);
    int32 num = KeyValue.Num();
    for (int32 i=DebugStart;i<num&&i<DebugEnd&&i>=0;i++)
    {
        count++;
        if (count > MaxDrawCount)
        {
            UE_LOG(LogTemp, Log, TEXT("Too Much Draw.Total Count:%d MaxCountSet:%d"),FlowField.Num(),MaxDrawCount);
            break;
        }
        const FFlowVoxel* Voxel = FlowField.Find(KeyValue[i]);
        if (Voxel)
        {
            DrawDebugDirectionalArrow(
                GetWorld(),
                Voxel->WorldPosition+ FVector(0,0,VoxelSize / 2.0),
                Voxel->WorldPosition + Voxel->FlowDirection * VoxelSize /2.0+ FVector(0,0,VoxelSize / 2.0),
                30.f, FColor::Green, false, DebugDrawTime, 0, 1.5f);

            DrawDebugBox(GetWorld(), Voxel->WorldPosition, FVector(VoxelSize/2.0), FColor::Cyan, false, DebugDrawTime, 0, 2.0f);
            //DrawDebugString(GetWorld(), Voxel->WorldPosition,FString::FromInt(Voxel->Heat), nullptr, FColor::Red, DebugDrawTime, false, 10);
            
        }
        else
        {
            DrawDebugPoint(GetWorld(), Voxel->WorldPosition, 5.0f, FColor::Red, false, DebugDrawTime);
        }
    }
}

