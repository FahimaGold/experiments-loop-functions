/**
  * @file <loop-functions/example/AggregationLoopFunc.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef AGGREGATION_SINGLE_SPOT_LOOP_FUNC
#define AGGREGATION_SINGLE_SPOT_LOOP_FUNC

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include "../../src/CoreLoopFunctions.h"

using namespace argos;

class AggregationSingleSpotLoopFunction: public CoreLoopFunctions {
  public:
    AggregationSingleSpotLoopFunction();
    AggregationSingleSpotLoopFunction(const AggregationSingleSpotLoopFunction& orig);

    virtual ~AggregationSingleSpotLoopFunction();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void Reset();

    Real GetObjectiveFunction();

    bool isInitiallyOutsideBlackSpot(CVector2 cEpuckPosition);
    void MoveRobots();

    CVector3 GetRandomPosition();

  private:
    Real m_fRadius;
    CVector2 m_cCoordSpot;

    UInt32 m_unScoreSpot;
    Real m_fObjectiveFunction;
};

#endif
