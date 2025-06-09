// Fill out your copyright notice in the Description page of Project Settings.


#include "CrowdGenerator.h"
#include "MassSpawnLocationProcessor.h"

void UCrowdGenerator::Generate(UObject& QueryOwner, 
    TConstArrayView<FMassSpawnedEntityType> EntityTypes,
    int32 Count,
    FFinishedGeneratingSpawnDataSignature& FinishedGeneratingSpawnPointsDelegate) const
{
    if (!FlowFieldBuilder)
    {
        UE_LOG(LogTemp, Warning, TEXT("FlowFieldBuilder δ����"));
        FinishedGeneratingSpawnPointsDelegate.ExecuteIfBound({});
        return;
    }
    if (Count <= 0)
    {
        FinishedGeneratingSpawnPointsDelegate.Execute(TArray<FMassEntitySpawnDataGeneratorResult>());
        return;
    }

    // �������ɽ��
    TArray<FMassEntitySpawnDataGeneratorResult> Results;
    BuildResultsFromEntityTypes(Count, EntityTypes, Results);

    // ��ȡ���ɵ�
    TArray<uint64> SpawnedKey;
    FlowFieldBuilder->FlowFieldByPoly.GenerateKeyArray(SpawnedKey);

    for (FMassEntitySpawnDataGeneratorResult& Result : Results)
    {
        Result.SpawnDataProcessor = UMassSpawnLocationProcessor::StaticClass();
        Result.SpawnData.InitializeAs<FMassTransformsSpawnData>();
        FMassTransformsSpawnData& Transforms = Result.SpawnData.GetMutable<FMassTransformsSpawnData>();

        Transforms.Transforms.Reserve(Result.NumEntities);
        for (int i = 0; i < Result.NumEntities; i++)
        {
            FTransform& Transform = Transforms.Transforms.AddDefaulted_GetRef();
            FVector PolyCenter = FlowFieldBuilder->FlowFieldByPoly.Find(SpawnedKey[i])->Center;
            Transform.SetLocation(PolyCenter);
        }
    }

    FinishedGeneratingSpawnPointsDelegate.Execute(Results);
}