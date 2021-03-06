#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include <argos2/simulator/dynamic_linking/loop_functions.h>

#include <argos2/simulator/space/entities/floor_entity.h>
#include <argos2/common/utility/math/range.h>
#include <argos2/common/utility/argos_random.h>

using namespace argos;

class CForagingLoopFunctions : public CLoopFunctions {

public:

   CForagingLoopFunctions();
   virtual ~CForagingLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PrePhysicsEngineStep();

private:

   Real m_fFoodSquareRadius;
   Real num_forager;
   Real foragers;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY, m_cRandomClock;
   std::vector<CVector2> m_cFoodPos;
   std::vector<CVector2> unFoodItems;
   CFloorEntity* m_pcFloor;
   CARGoSRandom::CRNG* m_pcRNG;
   std::string m_strOutput;
   std::ofstream m_cOutput;
   UInt32 unFoodItems4;
   UInt32 m_unTimeOfNewFoods;
   UInt32 m_unCollectedFood;
   SInt64 m_nEnergy;
   SInt64 m_nEnergy2;
   UInt32 m_unEnergyPerFoodItem;
   UInt32 m_unEnergy2PerFoodItem;
   UInt32 m_unEnergyPerWalkingRobot;
};

#endif
