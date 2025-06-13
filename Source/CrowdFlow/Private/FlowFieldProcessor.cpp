#include "FlowFieldProcessor.h"
#include "MassCommonFragments.h"
#include "MassMovementFragments.h"
#include "EngineUtils.h"
#include "FlowFieldFragment.h"
#include "Steering\MassSteeringFragments.h"
#include "FlowFieldVoxelBuilder.h"
#include "MassNavigationFragments.h"



UFlowFieldProcessor::UFlowFieldProcessor():EntityQuery(*this)
{
    ExecutionOrder.ExecuteInGroup = TEXT("Movement");
    bAutoRegisterWithProcessingPhases = true;
   
    ExecutionFlags = (int32)EProcessorExecutionFlags::All;

}

void UFlowFieldProcessor::ConfigureQueries()
{
    // ���ò�ѯ�Ի�ȡ��Ҫ�����ʵ��
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
   

    EntityQuery.ForEachEntityChunk(EntityManager, Context, [&](FMassExecutionContext& Context)
        {
            const int32 NumEntities = Context.GetNumEntities();

            const TArrayView<FTransformFragment> Transforms = Context.GetMutableFragmentView<FTransformFragment>();
            //const TArrayView<FMassSteeringFragment> Steerings = Context.GetMutableFragmentView<FMassSteeringFragment>();
			const TArrayView<FFlowFieldFragment> FlowFields = Context.GetMutableFragmentView<FFlowFieldFragment>();
            //const TArrayView<FMassMoveTargetFragment> MoveTargets = Context.GetMutableFragmentView<FMassMoveTargetFragment>();
            const TArrayView<FMassVelocityFragment> Vels = Context.GetMutableFragmentView<FMassVelocityFragment>();
       

            for (int32 i = 0; i < NumEntities; ++i)
            {
                FVector Position = Transforms[i].GetTransform().GetLocation();
                if(Position.ContainsNaN())
                {
                    UE_LOG(LogTemp, Warning, TEXT("Entity contain NaN position"), i);
                    continue;  
				}

               // FMassMoveTargetFragment MoveTarget=MoveTargets[i];
                AFlowFieldVoxelBuilder* FlowField = FlowFields[i].FlowField;
				float ForceIntensity = FlowFields[i].ForceIntensity;  // ��ȡ��ǿ��
				float ForceWeight = FlowFields[i].ForceWeight;  // ��ȡ��Ȩ��

                if(!FlowField)
                {
                    UE_LOG(LogTemp, Warning, TEXT("FlowField is null for entity %d"), i);
                    continue;  // �������Ϊ�գ�������ʵ��
				}
                FVector Dir = FlowField->GetFlowByPoly(Position);  // ��ȡ��������
                UE_LOG(LogTemp,Log,TEXT("Index:%d Dir:%s SamplePos:%s,KeyNum:%d,ForceIntensity:%f"),
                    i,*Dir.ToString(),*Position.ToString(),FlowField->FlowFieldByPoly.Num(), ForceIntensity);
                //MoveTarget.Center = Position + Dir * ForceIntensity;  // �����ƶ�Ŀ��λ��
				//MoveTarget.Forward = Dir.GetSafeNormal();  // �����ƶ�Ŀ�귽��

				Vels[i].Value = Dir * ForceIntensity;  // ʩ�����������������ֵ�ɵ�
            }
        });
}