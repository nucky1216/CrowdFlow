// Fill out your copyright notice in the Description page of Project Settings.


#include "FillNeiboursProcessor.h"
#include "MassCommonFragments.h"
#include "FlowFieldFragment.h"
#include "FlowFieldNeiboursSubsystem.h"

UFillNeiboursProcessor::UFillNeiboursProcessor():EntityQuery(*this)
{
	ExecutionOrder.ExecuteInGroup = TEXT("Movement");
	ExecutionOrder.ExecuteBefore.Add(TEXT("FlowFieldProcessor"));
	bAutoRegisterWithProcessingPhases = true;
}

void UFillNeiboursProcessor::ConfigureQueries()
{
	EntityQuery.AddRequirement<FFlowFieldFragment>(EMassFragmentAccess::ReadWrite);
	EntityQuery.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadOnly);
}

void UFillNeiboursProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
	UE_LOG(LogTemp, Log, TEXT("FillNeiboursLocs..."));

	UFlowFieldNeiboursSubsystem* FlowFieldNeiboursSubsystem = UWorld::GetSubsystem<UFlowFieldNeiboursSubsystem>(Context.GetWorld());

	if (!FlowFieldNeiboursSubsystem)
	{
		UE_LOG(LogTemp, Warning, TEXT("UFlowFieldNeiboursSubsystem not found!"));
		return;
	}
	FlowFieldNeiboursSubsystem->PolyNeibours.Empty();

	EntityQuery.ForEachEntityChunk(EntityManager, Context, [&](FMassExecutionContext& Context)
	{
			TArrayView<FFlowFieldFragment> FlowFieldFragments = Context.GetMutableFragmentView<FFlowFieldFragment>();
			const TArrayView<FTransformFragment> TransformFragments = Context.GetMutableFragmentView<FTransformFragment>();

			int32 NumEntities = Context.GetNumEntities();
			for(int32 i = 0; i < NumEntities; ++i)
			{
				FMassEntityHandle CurEntity = Context.GetEntity(i);

				FVector Location  = TransformFragments[i].GetTransform().GetLocation();;
				dtPolyRef CurPolyRef= FlowFieldFragments[i].PolyRef;
				FVector ProjectorExtent = FlowFieldFragments[i].ProjectExtent;

				AFlowFieldVoxelBuilder* FlowField = FlowFieldFragments[i].FlowField;
				
				dtPolyRef SamplerRef = FlowField->GetPolyRef(Location, ProjectorExtent);
				
				FlowFieldNeiboursSubsystem->RegisterPolyEntity(SamplerRef, CurEntity, Location);

				FlowFieldFragments[i].CurrentPolyRef = SamplerRef;
				
			}
	});
}
