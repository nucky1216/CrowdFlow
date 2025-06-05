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
    GenerateFlowField();
    DebugDrawFlowField();
}

FIntVector AFlowFieldVoxelBuilder::GetGridIndex(const FVector& Location) const
{
    FVector Local = Location - GetActorLocation();
    return FIntVector(
        FMath::RoundToInt(Local.X / VoxelSize),
        FMath::RoundToInt(Local.Y / VoxelSize),
        FMath::RoundToInt(Local.Z / VoxelSize));
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

                FFlowVoxel Voxel;
                Voxel.WorldPosition = SamplePos;
                Voxel.bIsValid = false;

                if (bNav)
                {
                    UNavigationPath* Path = NavSys->FindPathToLocationSynchronously(
                        GetWorld(), ProjectedLocation.Location, TargetLocation);

                    if (Path && Path->PathPoints.Num() >= 2)
                    {
                        Voxel.FlowDirection = (Path->PathPoints[1] - ProjectedLocation.Location).GetSafeNormal();
                        Voxel.bIsValid = true;
                    }
                }

                FlowField.Add(FIntVector(X, Y, Z), Voxel);
            }
        }
    }
}

void AFlowFieldVoxelBuilder::DebugDrawFlowField()
{
    for (const auto& Elem : FlowField)
    {
        const FFlowVoxel& Voxel = Elem.Value;
        if (Voxel.bIsValid)
        {
            DrawDebugDirectionalArrow(
                GetWorld(),
                Voxel.WorldPosition,
                Voxel.WorldPosition + Voxel.FlowDirection * 50.f,
                30.f, FColor::Green, false, 5.0f, 0, 1.5f);
        }
        else
        {
            DrawDebugPoint(GetWorld(), Voxel.WorldPosition, 5.0f, FColor::Red, false, 5.0f);
        }
    }
}

