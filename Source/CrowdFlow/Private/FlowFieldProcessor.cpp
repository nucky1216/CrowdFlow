#include "FlowFieldProcessor.h"
#include "MassCommonFragments.h"
#include "MassMovementFragments.h"
#include "EngineUtils.h"
#include "FlowFieldFragment.h"
#include "Steering\MassSteeringFragments.h"
#include "FlowFieldVoxelBuilder.h"
#include "MassNavigationFragments.h"
#include "FlowFieldNeiboursSubsystem.h"


UFlowFieldProcessor::UFlowFieldProcessor():EntityQuery(*this)
{
    ExecutionOrder.ExecuteInGroup = TEXT("Movement");
    bAutoRegisterWithProcessingPhases = true;
   
    //ExecutionFlags = (int32)EProcessorExecutionFlags::All;

}

void UFlowFieldProcessor::ConfigureQueries()
{
    // 配置查询以获取需要处理的实体
    EntityQuery.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadOnly);
    EntityQuery.AddRequirement<FMassVelocityFragment>(EMassFragmentAccess::ReadOnly);
    //EntityQuery.AddRequirement<FMassForceFragment>(EMassFragmentAccess::ReadWrite);
    //EntityQuery.AddRequirement<FMassMoveTargetFragment>(EMassFragmentAccess::ReadWrite);
	EntityQuery.AddRequirement<FFlowFieldFragment>(EMassFragmentAccess::ReadOnly);
    //EntityQuery.AddRequirement<FMassSteeringFragment>(EMassFragmentAccess::ReadWrite);
    

}


void UFlowFieldProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
    bool DebugDraw = false;

    UE_LOG(LogTemp, Log, TEXT("Process Running"));
    UFlowFieldNeiboursSubsystem* FlowFieldNeiboursSubsystem = UWorld::GetSubsystem<UFlowFieldNeiboursSubsystem>(Context.GetWorld());
    if(!FlowFieldNeiboursSubsystem)
    {
       UE_LOG(LogTemp, Warning, TEXT("FlowFieldNeiboursSubsystem is null"));
       return;
	}
	TArray<FMassEntityHandle> EntitiesToRemove;

    EntityQuery.ForEachEntityChunk(EntityManager, Context, [&](FMassExecutionContext& Context)
        {
            const int32 NumEntities = Context.GetNumEntities();
			float DeltaTime = Context.GetDeltaTimeSeconds();
            

            const TArrayView<FTransformFragment> Transforms = Context.GetMutableFragmentView<FTransformFragment>();
            //const TArrayView<FMassSteeringFragment> Steerings = Context.GetMutableFragmentView<FMassSteeringFragment>();
			const TArrayView<FFlowFieldFragment> FlowFields = Context.GetMutableFragmentView<FFlowFieldFragment>();
            //const TArrayView<FMassMoveTargetFragment> MoveTargets = Context.GetMutableFragmentView<FMassMoveTargetFragment>();
            const TArrayView<FMassVelocityFragment> Vels = Context.GetMutableFragmentView<FMassVelocityFragment>();
       

            for (int32 i = 0; i < NumEntities; ++i)
            {
				FMassEntityHandle CurEntityHandle=Context.GetEntity(i);  // 获取实体句柄
                FVector Position = Transforms[i].GetTransform().GetLocation();

             

               // FMassMoveTargetFragment MoveTarget=MoveTargets[i];
                AFlowFieldVoxelBuilder* FlowField = FlowFields[i].FlowField;
                float Mass = FlowFields[i].Mass; // 假设质量为1.0，实际应用中可以从实体中获取质量信息
                float MaxSpeed = FlowFields[i].MaxSpeed;  // 获取最大速度
                int32 MaxNeibourNum = FlowFields[i].MaxSearchNeibourNum;
                dtPolyRef CurPolyRef = FlowFields[i].CurrentPolyRef;

                if(!FlowField)
                {
                    UE_LOG(LogTemp, Warning, TEXT("FlowField is null for entity %d"), i);
                    continue;  // 如果流场为空，跳过此实体
				}

                if ((FlowField->TargetLocation-Position).Length()<100.f)
                {
                    UE_LOG(LogTemp, Warning, TEXT("Reach Target"));
					EntitiesToRemove.Add(CurEntityHandle);  // 如果到达目标位置，添加到删除列表
                    continue;
                }

                FVector DebugGuidanceForce, DebugPlaneForce, DebugRepelForce;

                FVector FlowForce =FlowField->GetFlowByPoly(Position, Vels[i].Value,
                    DebugRepelForce,DebugGuidanceForce,DebugPlaneForce);  // 获取流场力

				FVector NeiRepelForce = FVector::ZeroVector;
                FlowField->GetForceFromNeibours(CurPolyRef, CurEntityHandle, Vels[i].Value,NeiRepelForce, MaxNeibourNum);

				FlowForce += NeiRepelForce;  // 添加邻居的排斥力

                if (Vels[i].Value.Size() < MaxSpeed/2.0)
                {
                    FVector LDForce;
                    FlowField->GetLowDensityForce(CurPolyRef, LDForce);

					FlowForce += LDForce;  // 添加低密度力
                }

                if (DebugDraw)
                {
					DrawDebugDirectionalArrow(Context.GetWorld(), Position, Position+2.0*FlowForce,
                        120,FColor::Blue, false, Context.GetWorld()->GetDeltaSeconds()+0.001, 0, 5.0f);
                }

				FVector Velocity=Vels[i].Value + FlowForce/Mass * DeltaTime;  // 施加流场方向的力，数值可调
                Vels[i].Value= Velocity.GetClampedToSize(0, MaxSpeed);              
            }
        });

    if(EntitiesToRemove.Num() > 0)
    {
		EntityManager.Defer().DestroyEntities(EntitiesToRemove);  // 异步批量删除到达目标的实体
	}
}