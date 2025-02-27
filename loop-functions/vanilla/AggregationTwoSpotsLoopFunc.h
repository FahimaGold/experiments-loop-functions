/**
  * @file <loop-functions/example/AggregationLoopFunc.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef AGGREGATION_TWO_SPOTS_LOOP_FUNC
#define AGGREGATION_TWO_SPOTS_LOOP_FUNC

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/entity.h>
#include "../../src/CoreLoopFunctions.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
#include <argos3/core/simulator/physics_engine/physics_model.h>



using namespace argos;

class AggregationTwoSpotsLoopFunction: public CoreLoopFunctions {
  public:
    AggregationTwoSpotsLoopFunction();
    AggregationTwoSpotsLoopFunction(const AggregationTwoSpotsLoopFunction& orig);
    virtual ~AggregationTwoSpotsLoopFunction();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void Reset();

    Real GetObjectiveFunction();

    CVector3 GetRandomPosition();
    CVector3 GetLeftPosition();
    virtual void PreStep();
   
  private:
    Real m_fRadius;
    CVector2 m_cCoordSpot1;
    CVector2 m_cCoordSpot2;

    UInt32 m_unScoreSpot1;
    UInt32 m_unScoreSpot2;
    Real m_fObjectiveFunction;
     /*
    These variable allow to track the number of timesteps to be 100 and then change the light intensity in the environment
    
    */
   //  CLightEntity* m_pcLight;;
    UInt32 m_counter;
    UInt32 m_numSteps;



};

#endif
