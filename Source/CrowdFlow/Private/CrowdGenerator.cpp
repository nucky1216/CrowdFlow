// Fill out your copyright notice in the Description page of Project Settings.


#include "CrowdGenerator.h"
#include "Detour/DetourNavMesh.h"
#include "MassSpawnLocationProcessor.h"

void UCrowdGenerator::Generate(UObject& QueryOwner, 
    TConstArrayView<FMassSpawnedEntityType> EntityTypes,
    int32 Count,
    FFinishedGeneratingSpawnDataSignature& FinishedGeneratingSpawnPointsDelegate) const
{
    if (!FlowFieldBuilder)
    {
        UE_LOG(LogTemp, Warning, TEXT("FlowFieldBuilder 未设置"));
        FinishedGeneratingSpawnPointsDelegate.ExecuteIfBound({});
        return;
    }
    if (Count <= 0)
    {
        FinishedGeneratingSpawnPointsDelegate.Execute(TArray<FMassEntitySpawnDataGeneratorResult>());
        return;
    }

    // 构建生成结果
    TArray<FMassEntitySpawnDataGeneratorResult> Results;
    BuildResultsFromEntityTypes(Count, EntityTypes, Results);

    // 获取生成点
    TArray<dtPolyRef> SpawnedKey;
    FlowFieldBuilder->FlowFieldByPoly.GenerateKeyArray(SpawnedKey);

    for (FMassEntitySpawnDataGeneratorResult& Result : Results)
    {
        Result.SpawnDataProcessor = UMassSpawnLocationProcessor::StaticClass();
        Result.SpawnData.InitializeAs<FMassTransformsSpawnData>();
        FMassTransformsSpawnData& Transforms = Result.SpawnData.GetMutable<FMassTransformsSpawnData>();

        Transforms.Transforms.Reserve(Result.NumEntities);
        int32 SpawnedCount = 0;
        for (int32 i = 0; SpawnedCount < Result.NumEntities; i++)
        {
            FTransform& Transform = Transforms.Transforms.AddDefaulted_GetRef();
            FVector RandomPoint = FlowFieldBuilder->GetRandomPointInPoly(SpawnedKey[i% SpawnedKey.Num()]);

            if (RandomPoint.IsZero())
                continue;

			UE_LOG(LogTemp, Log, TEXT("PolyCenter: %s"), *RandomPoint.ToString());
            Transform.SetLocation(RandomPoint);
			SpawnedCount++;
            
        }
    }

    FinishedGeneratingSpawnPointsDelegate.Execute(Results);
}