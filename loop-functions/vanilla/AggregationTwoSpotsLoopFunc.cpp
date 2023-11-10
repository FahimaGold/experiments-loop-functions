/**
  * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "AggregationTwoSpotsLoopFunc.h"

/****************************************/
/****************************************/

AggregationTwoSpotsLoopFunction::AggregationTwoSpotsLoopFunction() {
  m_fRadius = 0.3;
  m_cCoordSpot1 = CVector2(0.55,0);
  m_cCoordSpot2 = CVector2(-0.55,0);
  m_unScoreSpot1 = 0;
  m_unScoreSpot2 = 0;
  m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

AggregationTwoSpotsLoopFunction::AggregationTwoSpotsLoopFunction(const AggregationTwoSpotsLoopFunction& orig) {}

/****************************************/
/****************************************/

void AggregationTwoSpotsLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    // Getting the light entity
    CSpace::TMapPerType& mapEntities = GetSpace().GetEntitiesByType("light");
     /* if (!mapEntities.empty()) {
         CSpace::TMapPerType::iterator it = mapEntities.begin();
         m_pcLight = any_cast<CLightEntity*>(it->second);
      } else {
         THROW_ARGOSEXCEPTION("No light entity found in the space!");
      }*/

    m_counter = 0;
   // we want to change the light intensity after 100 timesteps
   m_numSteps = 100;
   
    PreStep();
}

/****************************************/
/****************************************/


AggregationTwoSpotsLoopFunction::~AggregationTwoSpotsLoopFunction() {}

/****************************************/
/****************************************/

void AggregationTwoSpotsLoopFunction::Destroy() {}

/****************************************/
/****************************************/

argos::CColor AggregationTwoSpotsLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  d = (m_cCoordSpot2 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  return CColor::GRAY50;
}


/****************************************/
/****************************************/

void AggregationTwoSpotsLoopFunction::Reset() {
  m_fObjectiveFunction = 0;
  m_unScoreSpot1 = 0;
  m_unScoreSpot2 = 0;
  CoreLoopFunctions::Reset();
 
}

/****************************************/
/****************************************/

void AggregationTwoSpotsLoopFunction::PostExperiment() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    Real fDistanceSpot2 = (m_cCoordSpot2 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius) {
      m_unScoreSpot1 += 1;
    } else if (fDistanceSpot2 <= m_fRadius){
      m_unScoreSpot2 += 1;
    }
  }

  m_fObjectiveFunction = Max(m_unScoreSpot1, m_unScoreSpot2)/(Real) m_unNumberRobots;
  LOG << "Score = " << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real AggregationTwoSpotsLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AggregationTwoSpotsLoopFunction::GetRandomPosition() {
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

// Placing robots on the opposite side of the light source
CVector3 AggregationTwoSpotsLoopFunction::GetLeftPosition() {
  
    Real temp;
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 0.8f));
    Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 0.8f));
    // If b < a, swap them
    if (b < a) {
        temp = a;
        a = b;
        b = temp;
    }
    Real fPosY =  b;  // set the x-coordinate to the left boundary of the arena
    Real fPosX =  a * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b));
    //Real fPosZ = m_pcRng->Uniform(CRange<Real>(-0.5f, 0.5f));  // generate a random z-coordinate

    // Calculate the opposite position with respect to the light source
    
    CVector3 oppositePosition(fPosX, fPosY, 0);  // opposite position
    std::cout  <<"PosX: "<< fPosX << " PosY: "<< fPosY <<std::endl;
   

    return oppositePosition;
}

 void AggregationTwoSpotsLoopFunction:: PreStep() {
      // Increment the counter
      m_counter++;
      
      // Check if the desired number of time steps is reached
      if(m_counter == 100){
       CBoxEntity* pcObstacle1 = new CBoxEntity("obstacle1",
        argos::CVector3(0.0, 0.0, 0.1),     // Position (x, z)
        argos::CQuaternion(),                // Orientation (quaternion)
        true,                                // Movable
        argos::CVector3(0.1, 0.1, 0.1),     // Size (x, y, z)
        1.0                                  // Mass
            );
       
        CBoxEntity* pcObstacle2 = new CBoxEntity("obstacle2",
        argos::CVector3(-0.7, 0.1, 0.5),     // Position (x, z)
        argos::CQuaternion(),                // Orientation (quaternion)
        true,                                // Movable
        argos::CVector3(0.1, 0.1, 0.1),     // Size (x, y, z)
        1.0                                  // Mass
            );

         CBoxEntity* pcObstacle3 = new CBoxEntity("obstacle3",
        argos::CVector3(-0.5, 0.4, 0.5),     // Position (x, z)
        argos::CQuaternion(),                // Orientation (quaternion)
        true,                                // Movable
        argos::CVector3(0.1, 0.1, 0.1),     // Size (x, y, z)
        1.0                                  // Mass
            );
            CBoxEntity* pcObstacle4 = new CBoxEntity("obstacle4",
        argos::CVector3(-0.3, 0.25, 0.7),     // Position (x, z)
        argos::CQuaternion(),                // Orientation (quaternion)
        true,                                // Movable
        argos::CVector3(0.1, 0.1, 0.1),     // Size (x, y, z)
        1.0                                  // Mass
            );
      pcObstacle1->Enable();
      pcObstacle2->Enable();
      pcObstacle3->Enable();
      pcObstacle4->Enable();
      GetSpace().AddEntity(*pcObstacle1);
      GetSpace().AddEntity(*pcObstacle2);
      GetSpace().AddEntity(*pcObstacle3);
      GetSpace().AddEntity(*pcObstacle4);

      // Removing obstacles by the end of the experiment

      // Check the end of the experiment
   if (GetSpace().GetSimulationClock() == 200) {
      // Remove all obstacles
     
         GetSpace().RemoveEntity(*pcObstacle1);
         GetSpace().RemoveEntity(*pcObstacle2);
         GetSpace().RemoveEntity(*pcObstacle3);
         GetSpace().RemoveEntity(*pcObstacle4);
         delete pcObstacle1;
         delete pcObstacle2;
      
      }
    
      
      }
      
 }

    
    



REGISTER_LOOP_FUNCTIONS(AggregationTwoSpotsLoopFunction, "aggregation_loop_functions");
