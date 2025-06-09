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
    GenerateFlowFieldPoly();
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

void AFlowFieldVoxelBuilder::GenerateFlowFieldPoly()
{
    FlowFieldByPoly.Empty();

    UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(GetWorld());
    if (!NavSys) return;

    const ARecastNavMesh* RecastNavMesh = Cast<ARecastNavMesh>(NavSys->GetMainNavData());
    if (!RecastNavMesh) return;

    const dtNavMesh* DetourMesh = RecastNavMesh->GetRecastMesh();
    if (!DetourMesh) return;

    for (int32 i = 0; i < DetourMesh->getMaxTiles(); ++i)
    {
        const dtMeshTile* Tile = DetourMesh->getTile(i);
        if (!Tile || !Tile->header) continue;

        for (int32 j = 0; j < Tile->header->polyCount; ++j)
        {
            const dtPoly* Poly = &Tile->polys[j];
            if (Poly->getType() != DT_POLYTYPE_GROUND) continue;

            dtPolyRef PolyRef = DetourMesh->encodePolyId(Tile->salt, i,j);

            // 计算多边形中心
            FVector Center = FVector::ZeroVector;
            int32 VertCount = Poly->vertCount;
			TArray<FVector> Vertices;
            for (int32 k = 0; k < VertCount; ++k)
            {
                const dtReal* v = &Tile->verts[Poly->verts[k] * 3];
                Center += FVector(-1.0*v[0], -1.0 * v[2], v[1]);
                if(Vertices.Num() <=3)
                {
                    Vertices.Add(FVector(-1.0 * v[0], -1.0 * v[2], v[1]));
				}

                DrawDebugPoint(GetWorld(), FVector(-1.0 * v[0],-1.0*v[2], v[1]), 5.0f, FColor::Blue, false, 5.0f);
            }
            Center /= float(VertCount);

            //GetPolyNormal
			FVector PolyNormal = FVector::CrossProduct(Vertices[1]-Vertices[0],Vertices[2]-Vertices[1]).GetSafeNormal();

            FNavLocation NavLoc;
            if (!NavSys->ProjectPointToNavigation(Center, NavLoc)) continue;

            UNavigationPath* Path = NavSys->FindPathToLocationSynchronously(GetWorld(), Center, TargetLocation);

            FNavPolyFlow Flow;
            Flow.Center = Center;
            Flow.bIsValid = false;

            if (Path && Path->PathPoints.Num() >= 2)
            {
                Flow.FlowDirection = FVector::VectorPlaneProject((Path->PathPoints[1] - Center),PolyNormal).GetSafeNormal();
                Flow.bIsValid = true;
            }

            FlowFieldByPoly.Add(PolyRef, Flow);
        }
    }
}

void AFlowFieldVoxelBuilder::DebugDrawFlowFieldPoly()
{
    for (const auto& Elem : FlowFieldByPoly)
    {
        const FNavPolyFlow& Flow = Elem.Value;
        if (Flow.bIsValid)
        {
            DrawDebugDirectionalArrow(
                GetWorld(),
                Flow.Center,
                Flow.Center + Flow.FlowDirection * 100.f,
                30.f, FColor::Cyan, false, 5.0f, 0, 1.5f);
            DrawDebugPoint(GetWorld(), Flow.Center, 5.0f, FColor::Green, false, 5.0f);
        }
        else
        {
            DrawDebugPoint(GetWorld(), Flow.Center, 5.0f, FColor::Red, false, 5.0f);
        }
    }
}

FVector AFlowFieldVoxelBuilder::GetFlowByPoly(const FVector& Location, FVector ProjectExtent ) const
{
    UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(GetWorld());
    if (!NavSys)
    {
		UE_LOG(LogTemp, Warning, TEXT("Navigation System not found!"));
    }
    const ARecastNavMesh* RecastNavMesh = Cast<ARecastNavMesh>(NavSys->GetMainNavData());
    if (!RecastNavMesh) return FVector::ZeroVector;

    const dtNavMesh* DetourMesh = RecastNavMesh->GetRecastMesh();
    if (!DetourMesh) return FVector::ZeroVector;

	FNavLocation NavLocation;
	
    if (!NavSys->ProjectPointToNavigation(Location, NavLocation, ProjectExtent))
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to project point to navigation! Location: %s with Extent:%s"), *Location.ToString(), *ProjectExtent.ToString());
		return FVector::ZeroVector;
    }

    dtPolyRef PolyRef = NavLocation.NodeRef;
	const FNavPolyFlow* CurPolyFlow = FlowFieldByPoly.Find(PolyRef);
    if (!CurPolyFlow)
    {
        UE_LOG(LogTemp, Warning, TEXT("PolyRef not found in FlowFieldByPoly! PolyRef: %u"), PolyRef);
		return FVector::ZeroVector;
    }

	TArray<uint64> NeibourPolyRefs;
    const dtMeshTile* Tile = nullptr;
    const dtPoly* Poly = nullptr;

    if (DetourMesh->getTileAndPolyByRef(PolyRef, &Tile, &Poly) == DT_SUCCESS && Poly && Tile)
    {
        for (unsigned int i = 0; i < Poly->vertCount; ++i)
        {
            dtPolyRef NeighborRef = 0;
            if (Poly->neis[i] & DT_EXT_LINK)
            {
                // 外部链接（跨Tile），需要遍历Tile的links
                unsigned int offMeshBase = static_cast<unsigned int>(Tile->header->offMeshBase);
                unsigned int offMeshConCount = static_cast<unsigned int>(Tile->header->offMeshConCount);
                for (unsigned int j = Tile->header->offMeshBase; j < (offMeshBase + offMeshConCount); ++j)
                {
                    const dtLink& link = Tile->links[j];
                    if (link.edge == i)
                    {
                        NeighborRef = link.ref;
                        break;
                    }
                }
            }
            else if (Poly->neis[i])
            {
                // 内部链接
                unsigned int neiIndex = Poly->neis[i] - 1;
                NeighborRef = DetourMesh->encodePolyId(Tile->salt, DetourMesh->getTileIndex(Tile), neiIndex);
            }

            if (NeighborRef != 0)
            {
                // 这里就是邻接多边形的 PolyRef
                // 你可以收集或处理它
				NeibourPolyRefs.Add(NeighborRef);
            }
        }
    }

    // 计算邻接多边形的平均流向
	FVector FlowDirection = FVector(0,0,0);
	float TotalWeight = 0.0f;
    NeibourPolyRefs.Add(PolyRef);

    for(auto & NeibourPolyRef : NeibourPolyRefs)
    {
        const FNavPolyFlow* NeibourFlow = FlowFieldByPoly.Find(NeibourPolyRef);
        if (NeibourFlow && NeibourFlow->bIsValid)
        {
			float weight = 1/FVector::Dist(Location, NeibourFlow->Center);
            FlowDirection += weight*NeibourFlow->FlowDirection;
			TotalWeight += weight;
        }
	}

    return FlowDirection/TotalWeight;
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

