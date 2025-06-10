#include "FlowFieldProcessor.h"
#include "MassCommonFragments.h"
#include "MassMovementFragments.h"
#include "EngineUtils.h"
#include "FlowFieldFragment.h"
#include "..\..\..\..\..\..\Unreal Egine5.1\UE_5.3\Engine\Plugins\AI\MassAI\Source\MassNavigation\Public\Steering\MassSteeringFragments.h"


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
    //EntityQuery.AddRequirement<FMassForceFragment>(EMassFragmentAccess::ReadWrite);
    EntityQuery.AddRequirement<FMassVelocityFragment>(EMassFragmentAccess::ReadWrite);
	EntityQuery.AddRequirement<FFlowFieldFragment>(EMassFragmentAccess::ReadOnly);
    EntityQuery.AddRequirement<FMassSteeringFragment>(EMassFragmentPresence::All);
    

}


void UFlowFieldProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
    UE_LOG(LogTemp, Log, TEXT("Process Running"));
   

    EntityQuery.ForEachEntityChunk(EntityManager, Context, [&](FMassExecutionContext& Context)
        {
            const int32 NumEntities = Context.GetNumEntities();

            const TArrayView<FTransformFragment> Transforms = Context.GetMutableFragmentView<FTransformFragment>();
            //const TArrayView<FMassForceFragment> Forces = Context.GetMutableFragmentView<FMassForceFragment>();
			const TArrayView<FFlowFieldFragment> FlowFields = Context.GetMutableFragmentView<FFlowFieldFragment>();
            const TArrayView<FMassVelocityFragment> Vels = Context.GetMutableFragmentView<FMassVelocityFragment>();
       

            for (int32 i = 0; i < NumEntities; ++i)
            {
                FVector Position = Transforms[i].GetTransform().GetLocation();
                AFlowFieldVoxelBuilder* FlowField = FlowFields[i].FlowField;
                if(!FlowField)
                {
                    UE_LOG(LogTemp, Warning, TEXT("FlowField is null for entity %d"), i);
                    continue;  // �������Ϊ�գ�������ʵ��
				}
                FVector Dir = FlowField->GetFlowByPoly(Position);  // ��ȡ��������
                //UE_LOG(LogTemp,Log,TEXT("Index:%d Dir:%s KeyNum:%d"),i,*Dir.ToString(),FlowField->FlowFieldByPoly.Num());
                Vels[i].Value = Dir * 300.f;  // ʩ��������ֵ�ɵ�
            }
        });
}