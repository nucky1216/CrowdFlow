// Fill out your copyright notice in the Description page of Project Settings.

#include "FlowFieldAssignProcessor.h"
#include "MassCommonFragments.h"
#include "FlowFieldFragment.h"
#include "EngineUtils.h"
#include "FlowFieldSubsystem.h"
#include "FlowFieldVoxelBuilder.h"

UFlowFieldAssignProcessor::UFlowFieldAssignProcessor():EntityQuery(*this)
{
    ObservedType = FFlowFieldFragment::StaticStruct(); // 仅当该 Fragment 存在于 Archetype 中才会触发
    Operation = EMassObservedOperation::Add;

}

void UFlowFieldAssignProcessor::ConfigureQueries()
{
    EntityQuery.AddRequirement<FFlowFieldFragment>(EMassFragmentAccess::ReadWrite);
    EntityQuery.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadOnly);
}

void UFlowFieldAssignProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
    UE_LOG(LogTemp,Log,TEXT("Assign Processor Running....."));

    
    UFlowFieldNeiboursSubsystem* Subsystem = Context.GetWorld()->GetSubsystem<UFlowFieldNeiboursSubsystem>();
    Subsystem->InitialForMass();


    AFlowFieldVoxelBuilder* FlowField = Subsystem->FlowFieldBuilder;
    if (!IsValid(FlowField))
    {
		UE_LOG(LogTemp, Warning, TEXT("FlowFieldVoxelBuilder not found in the world! Please ensure it is placed in the level."));
        return;
    }

    EntityQuery.ForEachEntityChunk(EntityManager, Context, [&](FMassExecutionContext& Context)
        {
            const TArrayView<FFlowFieldFragment> FlowFieldRefs = Context.GetMutableFragmentView<FFlowFieldFragment>();
			const TArrayView<FTransformFragment> TransformRefs = Context.GetMutableFragmentView<FTransformFragment>();
            for (int i =0;i<Context.GetNumEntities();i++)
            {
                FlowFieldRefs[i].FlowField = FlowField;
				//FlowFieldRefs[i].NeiboursSubsystem = Subsystem;

                FVector Location = TransformRefs[i].GetTransform().GetLocation();

                //dtPolyRef newPolyRef=FlowField->GetPolyRef(Location, FVector(FlowField->VoxelSize/2));
                //Subsystem->UpdatePolyEntity(newPolyRef,FlowFieldRefs[i].CurrentPolyRef,Context.GetEntity(i));

				UE_LOG(LogTemp, Log, TEXT("Assigning FlowField to entity at location: %d->%s"),i, *Location.ToString());
            }
        });
}
