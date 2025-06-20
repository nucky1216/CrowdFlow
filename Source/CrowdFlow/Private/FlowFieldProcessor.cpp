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
   
    ExecutionFlags = (int32)EProcessorExecutionFlags::All;

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
    UE_LOG(LogTemp, Log, TEXT("Process Running"));
    UFlowFieldNeiboursSubsystem* FlowFieldNeiboursSubsystem = UWorld::GetSubsystem<UFlowFieldNeiboursSubsystem>(Context.GetWorld());
    if(!FlowFieldNeiboursSubsystem)
    {
       UE_LOG(LogTemp, Warning, TEXT("FlowFieldNeiboursSubsystem is null"));
       return;
	}

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

                if(Position.ContainsNaN())
                {
                    UE_LOG(LogTemp, Warning, TEXT("Entity contain NaN position"), i);
                    continue;  
				}

               // FMassMoveTargetFragment MoveTarget=MoveTargets[i];
                AFlowFieldVoxelBuilder* FlowField = FlowFields[i].FlowField;
                float Mass = FlowFields[i].Mass; // 假设质量为1.0，实际应用中可以从实体中获取质量信息
                float MaxSpeed = FlowFields[i].MaxSpeed;  // 获取最大速度
                int32 MaxNeibourNum = FlowFields[i].MaxSearchNeibourNum;


                if(!FlowField)
                {
                    UE_LOG(LogTemp, Warning, TEXT("FlowField is null for entity %d"), i);
                    continue;  // 如果流场为空，跳过此实体
				}

                FVector DebugGuidanceForce, DebugPlaneForce, DebugRepelForce;
                dtPolyRef PolyRef=0;
                FVector FlowForce =FlowField->GetFlowByPoly(Position, Vels[i].Value,
                    DebugRepelForce,DebugGuidanceForce,DebugPlaneForce);  // 获取流场力




                const float Radius = 500.f;

				//找到实体周围的邻居
                for(int32 j=0;j<MaxNeibourNum;j++)
                {
					TArray<FMassEntityHandle> Neibours = FlowFieldNeiboursSubsystem->GetPolyEntities(PolyRef,10);
                    if(Neibours.Num() == 0)
                    {
                        UE_LOG(LogTemp, Warning, TEXT("No neighbours found for entity %d with Handle:%u"), i, CurEntityHandle.AsNumber());
                        break;  // 如果没有邻居，跳过此实体
					}
				}
                

				FVector Velocity=Vels[i].Value + FlowForce/Mass * DeltaTime;  // 施加流场方向的力，数值可调
                Vels[i].Value= Velocity.GetClampedToSize(0, MaxSpeed);
            }
        });
}