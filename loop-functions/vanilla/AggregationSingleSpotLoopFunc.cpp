/**
  * @file <loop-functions/AggregationSingleSpotLoopFunc.cpp>
  *
  * @author Fahima Mokhtari - <f.mokhtari@innopolis.university>
  *
  * @license MIT License
  */

#include "AggregationSingleSpotLoopFunc.h"

/****************************************/
/****************************************/

AggregationSingleSpotLoopFunction::AggregationSingleSpotLoopFunction() {
  m_fRadius = 0.3;
  m_cCoordSpot = CVector2(0,0);
 
  m_unScoreSpot = 0;
 
  m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

// Validation function to make sure that the robots initially are outside the black spots.

bool AggregationSingleSpotLoopFunction::isInitiallyOutsideBlackSpot(CVector2 cEpuckPosition){
   Real fDistanceSpot = (m_cCoordSpot - cEpuckPosition).Length();
    ;
     if (fDistanceSpot <= m_fRadius) 
     return false;
  return true;
}

/****************************************/
/****************************************/

void AggregationSingleSpotLoopFunction::MoveRobots() {
  CEPuckEntity* pcEpuck;
  bool bPlaced = false;
  UInt32 unTrials;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    pcEpuck = any_cast<CEPuckEntity*>(it->second);
    // Choose position at random
    unTrials = 0;
    do {
       ++unTrials;
       CVector3 cEpuckPosition = GetRandomPosition();
       bPlaced = MoveEntity(pcEpuck->GetEmbodiedEntity(),
                            cEpuckPosition,
                            CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO,CRadians::TWO_PI)),
                            CRadians::ZERO,CRadians::ZERO),false);
      bPlaced = isInitiallyOutsideBlackSpot(CVector2(cEpuckPosition.GetX(), cEpuckPosition.GetY()));

    } while(!bPlaced && unTrials < 1000);
    if(!bPlaced) {
       THROW_ARGOSEXCEPTION("Can't place robot");
    }
  }
}

/****************************************/
/****************************************/



AggregationSingleSpotLoopFunction::AggregationSingleSpotLoopFunction(const AggregationSingleSpotLoopFunction& orig) {}

/****************************************/
/****************************************/

void AggregationSingleSpotLoopFunction::Init(TConfigurationNode& t_tree) {
    // CoreLoopFunctions::Init(t_tree);
    m_pcRng = CRandom::CreateRNG("argos");
    TConfigurationNode cParametersNode;
  try {
    cParametersNode = GetNode(t_tree, "params");
    GetNodeAttributeOrDefault(cParametersNode, "number_robots", m_unNumberRobots, (UInt32) 1);
    GetNodeAttributeOrDefault(cParametersNode, "dist_radius", m_fDistributionRadius, (Real) 0);
  } catch(std::exception e) {
    LOGERR << "Problem with Attributes in node params" << std::endl;
  }
    MoveRobots();
}

/****************************************/
/****************************************/


AggregationSingleSpotLoopFunction::~AggregationSingleSpotLoopFunction() {}

/****************************************/
/****************************************/

void AggregationSingleSpotLoopFunction::Destroy() {}

/****************************************/
/****************************************/

argos::CColor AggregationSingleSpotLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  return CColor::GRAY50;
}


/****************************************/
/****************************************/

void AggregationSingleSpotLoopFunction::Reset() {
  m_fObjectiveFunction = 0;
  m_unScoreSpot = 0;

  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void AggregationSingleSpotLoopFunction::PostExperiment() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot = (m_cCoordSpot - cEpuckPosition).Length();
    
    if (fDistanceSpot <= m_fRadius) {
      m_unScoreSpot += 1;
    } 
  }

  m_fObjectiveFunction = m_unScoreSpot/(Real) m_unNumberRobots;
  LOG << "Score = " << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real AggregationSingleSpotLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AggregationSingleSpotLoopFunction::GetRandomPosition() {
  Real temp;
  Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real  b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  // If b < a, swap them
  if (b < a) {
    temp = a;
    a = b;
    b = temp;
  }
  Real fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b));
  Real fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a/b));

  return CVector3(fPosX, fPosY, 0);
}



REGISTER_LOOP_FUNCTIONS(AggregationSingleSpotLoopFunction, "aggregation_single_spot_loop_functions");
