// Fill out your copyright notice in the Description page of Project Settings.


#include "FlowFieldVoxelBuilder.h"
#include "NavigationSystem.h"
#include "NavigationPath.h"
#include "NavMesh/RecastNavMesh.h"
#include "NavMesh/NavMeshRenderingComponent.h"
#include "DrawDebugHelpers.h"
#include "FlowFieldSubsystem.h"
#include "NavMesh/RecastHelpers.h"
#include "HAL/IConsoleManager.h"
#include "EntityActor.h"
#include "FlowFieldNeiboursSubsystem.h"
#include <Detour\DetourNavMeshQuery.h>


#define DEBUG_DRAW 1

// Sets default values
AFlowFieldVoxelBuilder::AFlowFieldVoxelBuilder()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;
    ConstructNeibourOffsets();
    //GenerateFlowFieldPoly();

}

// Called when the game starts or when spawned
void AFlowFieldVoxelBuilder::BeginPlay()
{
	Super::BeginPlay();
    RegistryNeibourSubsystem();
    GenerateFlowFieldPoly();
}

// Called every frame
void AFlowFieldVoxelBuilder::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}
bool SegmentCircleIntersection2D(FVector Start, FVector End, FVector Center3D, float Radius)
{
    TArray<FVector2D> Intersections;
    FVector2D P1 = FVector2D(Start.X, Start.Y);
    FVector2D P2 = FVector2D(End.X, End.Y);
    FVector2D Center = FVector2D(Center3D.X, Center3D.Y);

    FVector2D d = P2 - P1;
    FVector2D f = P1 - Center;

    float a = FVector2D::DotProduct(d, d);
    float b = 2 * FVector2D::DotProduct(f, d);
    float c = FVector2D::DotProduct(f, f) - Radius * Radius;

    float discriminant = b * b - 4 * a * c;
    if (discriminant >= 0)
    {
        discriminant = FMath::Sqrt(discriminant);

        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        if (t1 >= 0 && t1 <= 1)
            Intersections.Add(P1 + t1 * d);
        if (t2 >= 0 && t2 <= 1 && !FMath::IsNearlyEqual(t1, t2))
            Intersections.Add(P1 + t2 * d);
    }
    if (Intersections.Num() > 0)
    {
        return true;
    }
    return false;
}
bool IsSegmentInOrIntersectCircle2D(const FVector& Start, const FVector& End, const FVector& Center3D, float Radius)
{
    FVector2D P1(Start.X, Start.Y);
    FVector2D P2(End.X, End.Y);
    FVector2D Center(Center3D.X, Center3D.Y);

    // 判断两端点是否都在圆内
    float r2 = Radius * Radius;
    bool bStartIn = (P1 - Center).SizeSquared() <= r2;
    bool bEndIn = (P2 - Center).SizeSquared() <= r2;
    if (bStartIn && bEndIn)
        return true; // 完全在圆内

    // 判断是否有交点
    if (SegmentCircleIntersection2D(Start, End, Center3D, Radius))
        return true; // 有交点

    return false; // 完全在圆外
}
float CalcPolygonArea3D(const TArray<FVector>& Vertices)
{
    int32 N = Vertices.Num();
    if (N < 3) return 0.f;

    FVector Sum = FVector::ZeroVector;
    for (int32 i = 0; i < N; ++i)
    {
        const FVector& Current = Vertices[i];
        const FVector& Next = Vertices[(i + 1) % N];
        Sum += FVector::CrossProduct(Current, Next);
    }
    // 法向量方向为多边形法线，面积为模长的一半
    return 0.5f * Sum.Size();
}
void AFlowFieldVoxelBuilder::OnConstruction(const FTransform& Transform)
{
    //GenerateFlowFieldPoly();
    ConstructNeibourOffsets();
    
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
    VertToPolyMap.Empty();


    UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(GetWorld());
    if (!NavSys) return;

    const ARecastNavMesh* RecastNavMesh = Cast<ARecastNavMesh>(NavSys->GetMainNavData());
    if (!RecastNavMesh) return;

    const dtNavMesh* DetourMesh = RecastNavMesh->GetRecastMesh();
    if (!DetourMesh) return;

    //遍历所有的多边形，构建多边形流场
    for (int32 i = 0; i < DetourMesh->getMaxTiles(); ++i)
    {
        const dtMeshTile* Tile = DetourMesh->getTile(i);
        if (!Tile || !Tile->header) continue;

        for (int32 j = 0; j < Tile->header->polyCount; ++j)
        {
            const dtPoly* Poly = &Tile->polys[j];
            if (Poly->getType() != DT_POLYTYPE_GROUND) continue;

            int32 TileIndexInPool = DetourMesh->getTileIndex(Tile);
            dtPolyRef PolyRef = DetourMesh->encodePolyId(Tile->salt, TileIndexInPool, j);
            if (PolyRef == 0)
            {
                UE_LOG(LogTemp, Warning, TEXT("PolyRef is zero for tile %d, poly %d"), i, j);
                continue;
            }

            FNavPolyFlow Flow;
            Flow.bIsValid = false;
            Flow.StrPolyRef = FString::Printf(TEXT("%llu"), PolyRef);

            // 获取多边形顶点，计算多边形中心
            Flow.Center = FVector::ZeroVector;
            int32 VertCount = Poly->vertCount;
            for (int32 k = 0; k < VertCount; ++k)
            {
                const dtReal* v = &Tile->verts[Poly->verts[k] * 3];
                FVector PolyVert = FVector(-1.0 * v[0], -1.0 * v[2], v[1]); // 注意Recast的Y和Z轴是反的
                Flow.Center += PolyVert;
                Flow.PolyVerts.Add(PolyVert);

                VertToPolyMap.Add(PolyVert, PolyRef);

                if ( Poly->neis[k] == 0 ) //Tile内没有邻接多边形
                {
                    Flow.BoundaryEdge.Add(k);
                }

                DrawDebugPoint(GetWorld(), PolyVert, 5.0f, FColor::Blue, false, DebugDrawTime);
            }
            Flow.Center /= float(VertCount);

            //获取多边形边的邻接多边形
            for (unsigned int edgeIndex = 0; edgeIndex < Poly->vertCount; ++edgeIndex)
            {
                if (Poly->neis[edgeIndex] & DT_EXT_LINK)
                {
                    bool hasNeibour = false;
                    // 外部链接（跨Tile），需要遍历Tile的links
                    for (unsigned int linkIndex = Poly->firstLink; linkIndex != DT_NULL_LINK;)
                    {
                        const dtLink& link = Tile->links[linkIndex];
                        if (link.edge == edgeIndex && link.ref != 0)
                        {
                            //UE_LOG(LogTemp, Log, TEXT("Found external neighbor poly: %llu for poly: %llu"), link.ref, PolyRef);
                            Flow.EdgeNeibours.Add(link.ref);
							hasNeibour = true;
                        }
                       
                        linkIndex = link.next;
                    }
                    if (!hasNeibour)
                    {
                        Flow.BoundaryEdge.Add(edgeIndex);
                    }
                }
                else if (Poly->neis[edgeIndex])
                {
                    // 内部链接
                    unsigned int neiIndex = Poly->neis[edgeIndex] - 1;
                    dtPolyRef NeighborRef = DetourMesh->encodePolyId(Tile->salt, DetourMesh->getTileIndex(Tile), neiIndex);
                    if (NeighborRef != 0)
                    {
                        Flow.EdgeNeibours.Add(NeighborRef);
                    }
                }
            }

            //计算边界法线向量
            int32 BoundaryCount = Flow.BoundaryEdge.Num();
            for (int32 k = 0; k < BoundaryCount; k++)
            {
                int32 EdgeIndex = Flow.BoundaryEdge[k];
                FVector Edge0 = Flow.PolyVerts[(EdgeIndex + 1) % VertCount] - Flow.PolyVerts[EdgeIndex];
                FVector Edge1 = Flow.PolyVerts[(EdgeIndex + 2) % VertCount] - Flow.PolyVerts[(EdgeIndex + 1) % VertCount];

                FVector PolyNormal = FVector::CrossProduct(Edge0, Edge1).GetSafeNormal();
                FVector InnerNormal = FVector::CrossProduct(PolyNormal, Edge0).GetSafeNormal();

                Flow.BoundaryEdgeNormal.Add(InnerNormal.GetSafeNormal());
            }

            FNavLocation NavLoc;
            if (!NavSys->ProjectPointToNavigation(Flow.Center, NavLoc))
            {
                UE_LOG(LogTemp, Warning, TEXT("Failed to project poly center to navigation! Center: %s"), *Flow.Center.ToString());
                continue;
            }
            UNavigationPath* Path = NavSys->FindPathToLocationSynchronously(GetWorld(), Flow.Center, TargetLocation);
            if (Path->IsPartial())
            {
                continue; //如果路径没有到达目标点，则不添加当前多边形
            }

            //获取多边形中心点流向
            if (Path && Path->PathPoints.Num() >= 2)
            {
                FVector Edge0 = Flow.PolyVerts[1] - Flow.PolyVerts[0];
                FVector Edge1 = Flow.PolyVerts[2] - Flow.PolyVerts[1];

                Flow.PolyNormal = FVector::CrossProduct(Edge0, Edge1).GetSafeNormal();

                Flow.FlowDirection = FVector::VectorPlaneProject((Path->PathPoints[1] - Flow.Center), Flow.PolyNormal).GetSafeNormal();

                Flow.PolyNormal = -1 * Flow.PolyNormal.GetSafeNormal();

                Flow.bIsValid = true;

				Flow.Area = CalcPolygonArea3D(Flow.PolyVerts) / 10000.f; //cm^2 --> m^2

            }

            FlowFieldByPoly.Add(PolyRef, Flow);
        }
    }

    //构建多边形顶点邻接多边形
    for (auto& Elem : FlowFieldByPoly)
    {
        dtPolyRef PolyRef = Elem.Key;
        FNavPolyFlow& Flow = Elem.Value;
        if (Flow.bIsValid)
        {
            for (const FVector& PolyVert : Flow.PolyVerts)
            {
                TArray<dtPolyRef> VertNeibourPolys;

                VertToPolyMap.MultiFind(PolyVert, VertNeibourPolys);
                for (dtPolyRef NeiRef : VertNeibourPolys)
                {
                    if (NeiRef != 0 && NeiRef != PolyRef)
                    {
                        Flow.VertNeibours.AddUnique(NeiRef);
                    }
                }
            }
        }
    }


    //调整拐角处流向
    for (auto& Elem : FlowFieldByPoly)
    {
        if (Elem.Value.BoundaryEdge.Num() == 0)
            continue;

        float MaxAngle = 60.f; // 最大限制角度
        FVector FinalFlow = FVector::ZeroVector;

        for (int32 i = 0; i < Elem.Value.BoundaryEdge.Num(); i++)
        {
            int32 EdgeIndex = Elem.Value.BoundaryEdge[i];
            FVector Start = Elem.Value.PolyVerts[EdgeIndex];
            FVector End = Elem.Value.PolyVerts[(EdgeIndex + 1) % Elem.Value.PolyVerts.Num()];
            FVector EdgeNormal = Elem.Value.BoundaryEdgeNormal[i];
            FVector EdgeTangent = (Start - End).GetSafeNormal();

            //如果边界法线与流向夹角大于90度，则调整流向
            float Angle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(Elem.Value.FlowDirection, EdgeNormal.GetSafeNormal())));
            if (Angle > MaxAngle)
            {
                MaxAngle = Angle;
                if (EdgeTangent.Dot(Elem.Value.FlowDirection) > 0)
                    FinalFlow = EdgeTangent;
                else
                    FinalFlow = -EdgeTangent;
            }
        }
        if (!FinalFlow.IsZero())
        {
            Elem.Value.FlowDirection = FinalFlow;
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
            //画出流向
            DrawDebugDirectionalArrow(
                GetWorld(),
                Flow.Center,
                Flow.Center + Flow.FlowDirection * 100.f,
                30.f, FColor::Cyan, false, 5.0f, 0, 1.5f);
            DrawDebugPoint(GetWorld(), Flow.Center, 5.0f, FColor::Green, false, 5.0f);

            //画出面的法线
            DrawDebugDirectionalArrow(
                GetWorld(),
                Flow.Center,
                Flow.Center + Flow.PolyNormal * 100.f,
                30.f, FColor::Cyan, false, 5.0f, 0, 1.5f);
            DrawDebugPoint(GetWorld(), Flow.Center, 5.0f, FColor::White, false, 5.0f);

            //画出多边形边界边
            int32 BoundaryCount = Flow.BoundaryEdge.Num();
            for (int32 k = 0; k < BoundaryCount; ++k)
            {
                int32 EdgeIndex = Flow.BoundaryEdge[k];
                FVector Start = Flow.PolyVerts[EdgeIndex];
                FVector End = Flow.PolyVerts[(EdgeIndex + 1) % Flow.PolyVerts.Num()];
				//boundary边界线
                DrawDebugLine(GetWorld(), Start, End, FColor::Yellow, false, DebugDrawTime, 0, 10.0f);

				//画出边界法线
                FVector EdgeNormal = Flow.BoundaryEdgeNormal[k];
                FVector MidPoint = (Start + End) / 2.0f;
				DrawDebugDirectionalArrow(GetWorld(), MidPoint, MidPoint + EdgeNormal * 100.0f, 30.0f, FColor::Red, false, DebugDrawTime, 0, 5.0f);
            }

        }
        else
        {
            DrawDebugPoint(GetWorld(), Flow.Center, 5.0f, FColor::Red, false, 5.0f);
        }
    }
}

void AFlowFieldVoxelBuilder::DebugDrawSinglePoly(FString PolyRef)
{
	dtPolyRef PolyRefValue = FCString::Atoi64(*PolyRef);
	UE_LOG(LogTemp, Log, TEXT("DebugDrawSinglePoly PolyRef: %llu"), PolyRefValue);

	FNavPolyFlow* PolyFlow = FlowFieldByPoly.Find(PolyRefValue);
    if (!PolyFlow)
    {
        UE_LOG(LogTemp, Warning, TEXT("PolyRef not found in FlowFieldByPoly! PolyRef: %llu"), PolyRefValue);
		return;
    }
	//draw the polygon center
	DrawDebugPoint(GetWorld(), PolyFlow->Center, 10.0f, FColor::Green, false, DebugDrawTime);

	//Draw VertNeibours
	UE_LOG(LogTemp, Log, TEXT("PolyFlow VertNeibours Count: %d"), PolyFlow->VertNeibours.Num());
    for(const auto& Nei:PolyFlow->VertNeibours)
    {
        FNavPolyFlow* NeibourFlow = FlowFieldByPoly.Find(Nei);
        if (NeibourFlow && NeibourFlow->bIsValid)
        {
            DrawDebugLine(GetWorld(), PolyFlow->Center, NeibourFlow->Center, FColor::Blue, false, DebugDrawTime, 0, 2.0f);
            DrawDebugPoint(GetWorld(), NeibourFlow->Center, 5.0f, FColor::Blue, false, DebugDrawTime);
        }
	}

	//Draw EdgeNeibours
	//UE_LOG(LogTemp, Log, TEXT("PolyFlow EdgeNeibours Count: %d"), PolyFlow->EdgeNeibours.Num());
 //   for(const auto& Nei : PolyFlow->EdgeNeibours)
 //   {
 //       FNavPolyFlow* NeibourFlow = FlowFieldByPoly.Find(Nei);
 //       if (NeibourFlow && NeibourFlow->bIsValid)
 //       {
 //           DrawDebugLine(GetWorld(), PolyFlow->Center, NeibourFlow->Center, FColor::Red, false, DebugDrawTime, 0, 2.0f);
 //           DrawDebugPoint(GetWorld(), NeibourFlow->Center, 5.0f, FColor::Red, false, DebugDrawTime);
 //       }
	//}
}

FVector AFlowFieldVoxelBuilder::GetFlowByPoly(const FVector& Location, FVector CurVelocity, 
    FVector& RepelForce, FVector& GuidanceForce, FVector& PlaneForce, FVector ProjectExtent) 
{
	bool DebugDraw = false;
    GuidanceForce = FVector::ZeroVector;
    RepelForce = FVector::ZeroVector;
    PlaneForce = FVector::ZeroVector;
        
    dtPolyRef PolyRef = GetPolyRef(Location, ProjectExtent);

    const FNavPolyFlow* CurPolyFlow = FlowFieldByPoly.Find(PolyRef);

    if (!CurPolyFlow)
    {
        UE_LOG(LogTemp, Warning, TEXT("PolyRef not found in FlowFieldByPoly! PolyRef: %llu"), PolyRef);
        return FVector::ZeroVector;
    }
    //位于中心的点或没有邻居多边形时，直接返回当前多边形的流向
    if (CurPolyFlow->VertNeibours.Num() == 0 || FVector::Distance(CurPolyFlow->Center, Location) <= 0.001)
    {
        UE_LOG(LogTemp, Warning, TEXT("Poly:%llu Has No Neibours or Locate at Center"), PolyRef);
        return CurPolyFlow->FlowDirection;
    }

    FVector FlowDirection = CurPolyFlow->FlowDirection * DesiredForceStrength;
    // 计算邻接多边形的平均流向
    //FVector GuidanceForce = FVector::ZeroVector;
    GuidanceForce = FVector::ZeroVector;
    if (0)
    {
        float TotalWeight = 0;

        for (auto& NeibourPolyRef : CurPolyFlow->VertNeibours)
        {
            const FNavPolyFlow* NeibourFlow = FlowFieldByPoly.Find(NeibourPolyRef);
            if (NeibourFlow && NeibourFlow->bIsValid)
            {
                float weight = FMath::Clamp(1 / FVector::Dist(Location, NeibourFlow->Center), 0.0001, 1);
                GuidanceForce += weight * NeibourFlow->FlowDirection;
                TotalWeight += weight;

            }
        }
        GuidanceForce = (GuidanceForce / TotalWeight) * GuidanceForceStrength;
    }

    //边界检测
    //FVector TargetPos= Location + (FlowDirection + GuidanceForce).GetSafeNormal() * AgentRadius;

    int32 BoundaryCount = CurPolyFlow->BoundaryEdge.Num();
    if (BoundaryCount != 0)
    {
        //UE_LOG(LogTemp, Log, TEXT("Add RepelStrength...has %d Boundary"),BoundaryCount);

        float DistToBoundary = 1.0;
        RepelForce = FVector::ZeroVector;
        for (int32 k = 0; k < BoundaryCount; k++)
        {
            int32 EdgeIndex = CurPolyFlow->BoundaryEdge[k];
            FVector Start = CurPolyFlow->PolyVerts[EdgeIndex];
            FVector End = CurPolyFlow->PolyVerts[(EdgeIndex + 1) % CurPolyFlow->PolyVerts.Num()];

			
            FVector ClosestPointAtEdge = FMath::ClosestPointOnSegment(Location, Start, End);
         
			FVector EdgeToEntity = (Location-ClosestPointAtEdge);

            FVector BoundaryRepel = A_Wall * FMath::Exp((AgentRadius - EdgeToEntity.Size()) / B_Wall) * EdgeToEntity.GetSafeNormal();;

            RepelForce += FVector::VectorPlaneProject(BoundaryRepel, CurPolyFlow->PolyNormal);
        }

        RepelForce = RepelForce.GetClampedToMaxSize(Max_Wall);// .GetClampedToSize(0, 10 * RepelForceStrength);
        
        
     }


    //垂直修正力
	float SignDistToPoly = FVector::PointPlaneDist(Location, CurPolyFlow->Center, CurPolyFlow->PolyNormal);
    if (FMath::Abs(SignDistToPoly)>SurfaceHeightTolerance)
    {
        float PlaneForceZ = -SignDistToPoly * SurfaceSpringK;// +CurVelocity.Z * SurfaceSpringDampingC; // 距离越远，修正力越大
        PlaneForce = PlaneForceZ * CurPolyFlow->PolyNormal; // 施加在多边形法线方向上的力
        //PlaneForce = PlaneForce.GetClampedToMaxSize(SurfaceMaxForce); // 限制最大修正力
		//UE_LOG(LogTemp, Log, TEXT("DistToPoly: %f, SignDistToPlane: %f,CurVelocity.Z:%f, PlaneForceZ: %f, PlaneForce:%s"), 
        //    SignDistToPoly, SignDistToPlane, CurVelocity.Z, PlaneForceZ, *PlaneForce.ToString());
    }



    FlowDirection = (FlowDirection + GuidanceForce + RepelForce + PlaneForce);

    if (DebugDraw)
    {
        float lenth = 2.0;
	
		//Draw GuidanceForce
        DrawDebugDirectionalArrow(GetWorld(), Location, Location + GuidanceForce * lenth,
            100.0, FColor::Purple, false, GetWorld()->GetDeltaSeconds() + 0.001, 0, 10);

		//Draw RepelForce
        DrawDebugDirectionalArrow(GetWorld(), Location, Location + RepelForce * lenth,
			100.0, FColor::Red, false, GetWorld()->GetDeltaSeconds() + 0.001, 0, 10);

		//Draw PlaneForce
        DrawDebugDirectionalArrow(GetWorld(), Location, Location + PlaneForce * lenth,
            100.0, FColor::White, false, GetWorld()->GetDeltaSeconds() + 0.001, 0, 10);

		//Draw Final Force
        DrawDebugDirectionalArrow(GetWorld(), Location, Location + FlowDirection * lenth,
			100.0, FColor::Green, false, GetWorld()->GetDeltaSeconds() + 0.001, 0, 10);
    }

    return FlowDirection;
}

void AFlowFieldVoxelBuilder::GetLowDensityForce(dtPolyRef PolyRef,FVector EntityLoc, FVector EntityVel, FVector& LDForce)
{
    bool DebugDraw = true;
	LDForce = FVector::ZeroVector;

    if (!FlowFieldSubsystem)
    {
        UE_LOG(LogTemp, Warning, TEXT("FlowFieldSubsystem is null."));
        return;
    }
	FNavPolyFlow* CurPoly=FlowFieldByPoly.Find(PolyRef);
    if(!CurPoly)
    {
        UE_LOG(LogTemp, Warning, TEXT("PolyRef not found or invalid! PolyRef: %llu"), PolyRef);
        return;
	}
    float CurDenstiy = FlowFieldSubsystem->PolyNeibours.FindRef(PolyRef).Num() / CurPoly->Area;

    float MinDestity = CurDenstiy;
	dtPolyRef MinDensityPolyRef = PolyRef;
	FVector CurFlowDirection = CurPoly->FlowDirection;
    FVector CurCenter = CurPoly->Center;

    for(auto NeibourPolyRef:CurPoly->VertNeibours)
    {
		FNavPolyFlow* NeibourPoly = FlowFieldByPoly.Find(NeibourPolyRef);

		FVector NeibourFlow = NeibourPoly->FlowDirection;
		float AngluarWeight = FMath::Acos((NeibourPoly->Center- CurCenter).GetSafeNormal().Dot(CurFlowDirection.GetSafeNormal()));

		//FVector NeibourCenter = NeibourPoly->Center;
  //      if (CurFlowDirection.Dot(NeibourCenter - EntityLoc) <= 0)
  //          continue;
		float NeibourDensity = (DensityAngularScale*AngluarWeight+FlowFieldSubsystem->PolyNeibours.FindRef(NeibourPolyRef).Num() )/ NeibourPoly->Area;

        //float NeibourDensity = FlowFieldSubsystem->PolyNeibours.FindRef(NeibourPolyRef).Num() / NeibourPoly->Area;

        if(NeibourDensity < MinDestity)
        {
            MinDestity = NeibourDensity;
            MinDensityPolyRef = NeibourPolyRef;
        }
    }
    if(MinDestity < CurDenstiy)
    {
        FVector Center = GetFlowCenter(PolyRef);
        FVector NeibourCenter = GetFlowCenter(MinDensityPolyRef);
		LDForce = (NeibourCenter- EntityLoc).GetSafeNormal() * LowDensityForceStrength * (CurDenstiy - MinDestity);

        if (DebugDraw)
        {
            DrawDebugDirectionalArrow(GetWorld(), EntityLoc, EntityLoc + LDForce, 85,FColor::Orange, false, GetWorld()->GetDeltaSeconds() + 0.001, 0, 8);
        }
        
	}
        
}

float AFlowFieldVoxelBuilder::GetPolyDensity(dtPolyRef PolyRef)
{

    if (!FlowFieldSubsystem)
    {
        UE_LOG(LogTemp, Warning, TEXT("FlowFieldSubsystem is null."));
        return 1.0;
    }
    FNavPolyFlow* CurPoly = FlowFieldByPoly.Find(PolyRef);
    if (!CurPoly)
    {
        UE_LOG(LogTemp, Warning, TEXT("PolyRef not found or invalid! PolyRef: %llu"), PolyRef);
        return 1.0;
	}
    int32 Num = FlowFieldSubsystem->PolyNeibours.FindRef(PolyRef).Num();
    
	float Density = (Num * FMath::Pow(AgentRadius, 2) * PI) / CurPoly->Area; // 计算密度，单位为每平方米的实体数

	return DensityRatio*FMath::Exp(-Density / DensityScale);
}

void AFlowFieldVoxelBuilder::GetForceFromNeibours(dtPolyRef CurPolyRef, FMassEntityHandle EntityHandle, FVector EntityVel, FVector& NeiRepel, int32 MaxNum)
{
    const bool bDebugDraw = false;
    NeiRepel = FVector::ZeroVector;

    if (!FlowFieldSubsystem)
    {
        UE_LOG(LogTemp, Warning, TEXT("FlowFieldSubsystem is null."));
        return;
    }

    TMap<FMassEntityHandle, FVector> EntityLocations = FlowFieldSubsystem->GetPolyEntities(CurPolyRef, MaxNum);
    if (EntityLocations.Num() <= 1) return;

    const FVector EntityLoc = EntityLocations.FindRef(EntityHandle);
    EntityLocations.Remove(EntityHandle);

    const FNavPolyFlow* CurPoly = FlowFieldByPoly.Find(CurPolyRef);
    if (!CurPoly) return;

    for (const auto& Pair : EntityLocations)
    {
        const FVector OtherLoc = Pair.Value;
        FVector Offset = OtherLoc - EntityLoc;
        Offset = FVector::VectorPlaneProject(Offset, CurPoly->PolyNormal);

        float Distance = Offset.Size();
        if (Distance <= KINDA_SMALL_NUMBER) continue;

        if (Offset.Dot(EntityVel) <= 0)
            continue;

        // 用于方向相关权重（模拟注意力在正前方）
        //float CosAngle = Offset.GetSafeNormal().Dot(EntityVel.GetSafeNormal());
        //float AngularWeight = FMath::Clamp((CosAngle - 0.2f) / 0.8f, 0.f, 1.f); // 只对前方 0~70° 有效

        // 社会力模型中的指数式排斥力
        float ForceMag = A_Neibour * FMath::Exp(-(Distance-AgentRadius*2.2) / B_Neibour);
        FVector Force = FVector::VectorPlaneProject( - Offset.GetSafeNormal(),CurPoly->PolyNormal) * ForceMag;// *AngularWeight;

        NeiRepel += Force;

        if (bDebugDraw)
        {
            //DrawDebugLine(GetWorld(), EntityLoc, OtherLoc, FColor::Green, false, GetWorld()->GetDeltaSeconds() + 0.01, 0, 1.5f);
            //DrawDebugDirectionalArrow(GetWorld(), EntityLoc, EntityLoc + Force, 50.f, FColor::Red, false, GetWorld()->GetDeltaSeconds() + 0.01, 0, 5.f);
        }
    }

    // 限制总合力大小
    NeiRepel = NeiRepel.GetClampedToMaxSize(Max_Neibour);

    if (bDebugDraw)
    {
        DrawDebugDirectionalArrow(GetWorld(), EntityLoc, EntityLoc + NeiRepel , 100.f, FColor::Black, false, GetWorld()->GetDeltaSeconds() + 0.01, 0, 10.f);
    }

 
}

void AFlowFieldVoxelBuilder::RegistryNeibourSubsystem()
{
	FlowFieldSubsystem = GetWorld()->GetSubsystem<UFlowFieldNeiboursSubsystem>();
    if(!FlowFieldSubsystem)
    {
        UE_LOG(LogTemp, Warning, TEXT("FlowFieldNeiboursSubsystem not found! Cannot register subsystem."));
        return;
    }
    FlowFieldSubsystem->InitializeManual(this);
}

FVector AFlowFieldVoxelBuilder::DeltaMove(AEntityActor* Agent, UPARAM(ref)FVector& Velocity, float MaxSpeed, float Mass)
{

    if (!Agent) return FVector::ZeroVector;


	FVector NeiRepel = FVector::ZeroVector;
	GetForceFromNeibours(Agent->CurrentPolyRef, Agent->EntityHandle,FVector(1,0,0),NeiRepel,15);

    FVector RepelForce = FVector::ZeroVector;
    FVector GuidanceForce = FVector::ZeroVector;
    FVector PlaneForce = FVector::ZeroVector;

    FVector AgentLocation = Agent->GetActorLocation();
    float DeltaTime = GetWorld()->GetDeltaSeconds();

    FVector FinalForce = GetFlowByPoly(AgentLocation, Velocity, RepelForce, GuidanceForce, PlaneForce, FVector(VoxelSize / 2.0f));
	FinalForce += NeiRepel; // 添加邻居的排斥力

    FVector Acceleration = FinalForce / Mass; // 使用质量来计算加速度
    Velocity += Acceleration * DeltaTime; // 更新速度
    Velocity = Velocity.GetClampedToSize(0, MaxSpeed); // 限制速度

    //UE_LOG(LogTemp,Log,TEXT("Entity:%d has Velocity:%s, Acceleration:%s, FinalForce:%s, RepelForce:%s, NeiRepel:%s, PlaneForce:%s"),
    //   Agent->EntityID, *Velocity.ToString(), *(Acceleration * DeltaTime).ToString(), *FinalForce.ToString(), *RepelForce.ToString(), *NeiRepel.ToString(), *PlaneForce.ToString());

    FVector DeltaLocation = Velocity * DeltaTime; // 计算位移

    return AgentLocation + DeltaLocation;
}

FVector AFlowFieldVoxelBuilder::GetFlowCenter(dtPolyRef PolyRef) const
{
	const FNavPolyFlow* Flow = FlowFieldByPoly.Find(PolyRef);
    if (Flow && Flow->bIsValid)
    {
		return Flow->Center;
    }
	UE_LOG(LogTemp, Warning, TEXT("PolyRef not found or invalid! PolyRef: %llu"), PolyRef);
    return FVector::ZeroVector;
}

FVector AFlowFieldVoxelBuilder::GetRandomPointInPoly(dtPolyRef PolyRef) const
{
	FVector RandomPoint = FVector::ZeroVector;

    const ANavigationData* NavData = UNavigationSystemV1::GetCurrent(GetWorld())->
        GetDefaultNavDataInstance(FNavigationSystem::ECreateIfEmpty::Create);

	if (!NavData)
    {
		UE_LOG(LogTemp, Warning, TEXT("Navigation Data not found! Cannot get random point in poly."));
        return FVector::ZeroVector;
    }

    const ARecastNavMesh* RecastNavMesh = Cast<ARecastNavMesh>(NavData);
    if(!RecastNavMesh)
    {
        UE_LOG(LogTemp, Warning, TEXT("RecastNavMesh not found! Cannot get random point in poly."));
        return FVector::ZeroVector;
	}

    if(RecastNavMesh->GetRandomPointInPoly(PolyRef, RandomPoint))
    {
		return RandomPoint;
    }
	return FVector::ZeroVector; // 如果没有找到随机点，则返回零向量
}

dtPolyRef AFlowFieldVoxelBuilder::GetPolyRef(const FVector& Location, FVector ProjectExtent) const
{
    if (Location.ContainsNaN())
    {
        UE_LOG(LogTemp, Warning, TEXT("Location contains NaN! Location: %s"), *Location.ToString());
        return 0;
    }
    UNavigationSystemV1* NavSys = UNavigationSystemV1::GetCurrent(GetWorld());
    if (!NavSys)
    {
        UE_LOG(LogTemp, Warning, TEXT("Navigation System not found!"));
        return 0;
    }
    //DrawDebugPoint(GetWorld(), Location, 15, FColor::Black, false, 5.0, 0);

    FNavLocation NavLocation;

    if (!NavSys->ProjectPointToNavigation(Location, NavLocation, ProjectExtent))
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to project point to navigation! Location: %s with Extent:%s"), *Location.ToString(), *ProjectExtent.ToString());
        //PlaneForce = FVector(0, 0, -CurVelocity.Z * SurfaceSpringK*10.f);
        return 0;
    }

    dtPolyRef PolyRef = NavLocation.NodeRef;

    return PolyRef;
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
        //if (count > MaxDrawCount)
        //{
        //    UE_LOG(LogTemp, Log, TEXT("Too Much Draw.Total Count:%d MaxCountSet:%d"),FlowField.Num(),MaxDrawCount);
        //    break;
        //}
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

