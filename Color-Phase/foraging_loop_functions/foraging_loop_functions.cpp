#include "foraging_loop_functions.h"
#include <argos2/simulator/simulator.h>
#include <argos2/common/utility/configuration/argos_configuration.h>
#include <argos2/common/utility/datatypes/any.h>
#include <argos2/simulator/space/entities/footbot_entity.h>
#include <controllers/footbot_foraging/footbot_foraging.h>

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() :
   m_cForagingArenaSideX(-3.9f, 3.9f),
   m_cForagingArenaSideY(-3.9f, 3.9f),
   m_cRandomClock(1000,10000),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unTimeOfNewFoods(0),
   m_unCollectedFood(0),
   m_nEnergy(0),
   m_nEnergy2(0),
   m_unEnergyPerFoodItem(1),
   m_unEnergyPerWalkingRobot(1) {
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tForaging = GetNode(t_node, "foraging");
      /* Get a pointer to the floor entity */
      m_pcFloor = &m_cSpace.GetFloorEntity();
      /* Get the number of food items we want to be scattered from XML */
      num_forager=10;
      UInt32 unFoodItems1=9,unFoodItems2=9,unFoodItems3=9;
      unFoodItems4=9;
      //GetNodeAttribute(tForaging, "items", unFoodItems1);
      /* Get the number of food items we want to be scattered from XML */
      GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius=0.3;
      m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      m_pcRNG = CARGoSRandom::CreateRNG("argos");
      /* Distribute uniformly the items in the environment */
      unFoodItems.push_back(CVector2(unFoodItems1,0));
      m_cForagingArenaSideX.SetMin(3.0f);
      m_cForagingArenaSideX.SetMax(3.7f);
      m_cForagingArenaSideY.SetMin(3.0f);
      m_cForagingArenaSideY.SetMax(3.7f);
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));
      unFoodItems.push_back(CVector2(unFoodItems2,0));
      m_cForagingArenaSideX.SetMin(3.0f);
      m_cForagingArenaSideX.SetMax(3.7f);
      m_cForagingArenaSideY.SetMin(-3.7f);
      m_cForagingArenaSideY.SetMax(-3.0f);
      unFoodItems.push_back(CVector2(unFoodItems3,0));
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));
      m_cForagingArenaSideX.SetMin(-3.7f);
      m_cForagingArenaSideX.SetMax(-3.0f);
      m_cForagingArenaSideY.SetMax(3.7f);
      m_cForagingArenaSideY.SetMin(3.0f);
      
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));
      
      m_cRandomClock.SetMin(1000);
      m_cRandomClock.SetMax(2000);
      m_unTimeOfNewFoods = m_pcRNG->Uniform(m_cRandomClock);

                                    
      /* Get the output file name from XML */
      GetNodeAttribute(tForaging, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy\tconsume_energy\tenergy_per_food" << std::endl;
      /* Get energy gain per item collected */
      GetNodeAttribute(tForaging, "energy_per_item", m_unEnergyPerFoodItem);
      /* Get energy loss per walking robot */
      GetNodeAttribute(tForaging, "energy_per_walking_robot", m_unEnergyPerWalkingRobot);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset() {
   /* Zero the counters */
   m_unCollectedFood = 0;
   m_nEnergy = 0;
   m_nEnergy2 =0;
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy\tconsume_energy\tenergy_per_food" << std::endl;
   /* Distribute uniformly the items in the environment */
         
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if (i==1)
      {	
      m_cForagingArenaSideX.SetMax(2.7f); 
      m_cForagingArenaSideX.SetMin(2.5f);
      m_cForagingArenaSideY.SetMax(2.7f);
      m_cForagingArenaSideY.SetMin(2.2f); 
      }
      else if(i==2)
      {
      m_cForagingArenaSideX.SetMax(3.7f);
      m_cForagingArenaSideX.SetMin(3.0f);
      m_cForagingArenaSideY.SetMin(-3.7f);
      m_cForagingArenaSideY.SetMax(-3.0f);
      }
      else if(i==3)
      {
      m_cForagingArenaSideX.SetMin(-3.7f);
      m_cForagingArenaSideX.SetMax(-3.0f);
      m_cForagingArenaSideY.SetMax(-3.0f);
      m_cForagingArenaSideY.SetMin(-3.7f);
      }
      m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));

   }

}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Destroy() {
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
      
   if(c_position_on_plane.GetX() < 1.0f && c_position_on_plane.GetX() > -1.0f && c_position_on_plane.GetY() < 1.0f && c_position_on_plane.GetY()> -1.0f){
      return CColor::GRAY50;
   }
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i) {
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
         
         if(unFoodItems[i].GetX()> num_forager){
            return CColor::BLACK;
         }else{if(unFoodItems[i].GetX()> (num_forager/2)){
               return CColor::BLUE;}
               else{if(unFoodItems[i].GetX()> (num_forager/3)){
                       return CColor::GREEN;}
                    else{return CColor::YELLOW;}
                    }
          }
      }
   }
   if(c_position_on_plane.GetX()> 4.0f||  c_position_on_plane.GetX()< -4.0f ||  c_position_on_plane.GetY()> 4.0f || c_position_on_plane.GetY()< -4.0f ){
      return CColor::WHITE;}
   if((c_position_on_plane.GetX() > 3.9f && c_position_on_plane.GetX()< 4.0f) || (c_position_on_plane.GetX() <-3.9f && c_position_on_plane.GetX()> -4.0f) ||( c_position_on_plane.GetY() > 3.9f && c_position_on_plane.GetY()< 4.0f )||( c_position_on_plane.GetY() <-3.9f && c_position_on_plane.GetY()> -4.0f) ){
      return CColor::RED;
   }
   return CColor::WHITE;
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::PrePhysicsEngineStep() {
   /* Logic to pick and drop food items */
   /*
    * If a robot is in the nest, drop the food item
    * If a robot is on a food item, pick it
    * Each robot can carry only one food item per time
    */
   UInt32 unWalkingFBs = 0;
   UInt32 unRestingFBs = 0;
   /* Check whether a robot is on a food item */
   CSpace::TAnyEntityMap& m_cFootbots = m_cSpace.GetEntitiesByType("footbot_entity");
   
   if(m_cSpace.GetSimulationClock() == m_unTimeOfNewFoods){
      unFoodItems.push_back(CVector2(unFoodItems4,0));
      m_cForagingArenaSideX.SetMax(-2.0f);
      m_cForagingArenaSideX.SetMin(-2.6f);
      m_cForagingArenaSideY.SetMin(-2.6f);
      m_cForagingArenaSideY.SetMax(-2.0f);
      
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));
     }

   for(CSpace::TAnyEntityMap::iterator it = m_cFootbots.begin();
       it != m_cFootbots.end();
       ++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotForaging& cController = dynamic_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
      //num_forager=cController.NumForagers();
      /* Count how many foot-bots are in which state */
      if(! cController.IsResting()) ++unWalkingFBs;
      else ++unRestingFBs;
      if((m_cSpace.GetSimulationClock()%10)==0){
      if( cController.IsLeavingHome() ||cController.IsHoming() || cController.IsAvoidancing()){
        m_nEnergy2 +=6; 
       }
      if( cController.IsRandomWalking() || cController.IsScanningArea() || cController.IsMovingToFood())
        m_nEnergy2 +=8; 
      if( cController.IsMovingHome() | cController.IsDepositingFood()|| cController.IsGrabingFood())
        m_nEnergy2 +=12; 
      if(! cController.IsRestingRobot())
        m_nEnergy2 +=1;
      }
      /* Get the position of the foot-bot on the ground as a CVector2 */
      CVector2 cPos;
      cPos.Set(cFootBot.GetEmbodiedEntity().GetPosition().GetX(),
               cFootBot.GetEmbodiedEntity().GetPosition().GetY());

      /* Get food data */
      CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
      CFootBotForaging::SStateData& sStateData = cController.GetStateData();
      //if(sStateData.RobotType == 3)
      //std::cout<<cFootBot.GetEmbodiedEntity().GetOrientation()<<std::endl;
      // std::cout<<"robot id"<<cFootBot.GetId()<<std::endl;
      if(m_cSpace.GetSimulationClock()==1){
         sStateData.robot_id=cFootBot.GetId();
         if(sStateData.RobotType == 3)
            foragers++;}
      if(m_cSpace.GetSimulationClock() ==2 ){
           sStateData.NumberOfForagers=foragers;}
           //num_forager=foragers;}
      if(sStateData.RobotType == 3)
         sStateData.direction = cFootBot.GetEmbodiedEntity().GetOrientation();
       /*if(sStateData.setorientation==1){
         std::cout<<"set orientation"<<std::endl;
         cFootBot.GetEmbodiedEntity().SetOrientation(sStateData.new_direction);
         cFootBot.GetEmbodiedEntity().UpdateBoundingBox();}*/
      /* The foot-bot has a food item */
	 sStateData.RoboPos = cPos;
      if(sFoodData.HasFoodItem) {
         /* Check whether the foot-bot is in the nest */
         if(cPos.GetX() > -1.0f && cPos.GetX() < 1.0f && cPos.GetY() < 1.0f && cPos.GetY() > -1.0f) {
            /* Place a new food item on the ground (active in case dynamic environment)*/

        //    m_cFoodPos[sFoodData.FoodItemIdx].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
        //                                          m_pcRNG->Uniform(m_cForagingArenaSideY));
	       //
            /* Drop the food item */
            sFoodData.HasFoodItem = false;
            sStateData.EnergyState = CFootBotForaging::SStateData::STATE_DEPOSIT_FOOD;
            sFoodData.FoodItemIdx = 0;
            ++sFoodData.TotalFoodItems;
            /* Increase the energy and food count */
            m_nEnergy += m_unEnergyPerFoodItem;
            ++m_unCollectedFood;
            /* The floor texture must be updated */
            m_pcFloor->SetChanged();
         }
      }
      else {
		
         /* The foot-bot has no food item */
         /* Check whether the foot-bot is out of the nest */
		//std::cout<<"Hello"<<std::endl;
         if(!(cPos.GetX() > -1.0f && cPos.GetX() < 1.0f && cPos.GetY() < 1.0f && cPos.GetY() > -1.0f)) {
            /* Check whether the foot-bot is on a food item */

            bool bDone = false;
            for(size_t i = 0; i < m_cFoodPos.size() && !bDone; ++i) {
               if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
                  /* If so, we move that item out of sight */
		  sStateData.Food = m_cFoodPos[i];
                  Real a = unFoodItems[i].GetX();
                  if(a> num_forager){
                     sStateData.food_color=1;}
                     //sStateData.employers= num_forager;}
                  else if(a > (num_forager/2)){
                          sStateData.food_color=2;}
                          //sStateData.employers= num_forager/2;}
                       else if(a > (num_forager/3)){
                               sStateData.food_color=3;}
                                //sStateData.employers= num_forager/3;}
                             else {sStateData.food_color=4;}
                                  // sStateData.employers= num_forager/4;}
                  unFoodItems[i].SetX(a-1);
                  if(unFoodItems[i].GetX()==0)	
                      m_cFoodPos[i].Set(100.0f, 100.f);
                  /* The foot-bot is now carrying an item */
                  sFoodData.HasFoodItem = true;
                  sFoodData.FoodItemIdx = i;
                  /* The floor texture must be updated */
                  m_pcFloor->SetChanged();
                  /* We are done */
                  bDone = true;
               }
            }
         }
      }
   }
   /* Update energy expediture due to walking robots */
   m_nEnergy -= unWalkingFBs * m_unEnergyPerWalkingRobot;
   if(m_unCollectedFood!=0)
     m_unEnergy2PerFoodItem = m_nEnergy2/m_unCollectedFood;
   else
     m_unEnergy2PerFoodItem=0;
   /* Output stuff to file */
   if((m_cSpace.GetSimulationClock()%10)==0 && m_cSpace.GetSimulationClock()!=0 ){
   m_cOutput << m_cSpace.GetSimulationClock()/10 << "\t"
             << unWalkingFBs << "\t"
             << unRestingFBs << "\t"
             << m_unCollectedFood << "\t"<<"\t"
             << m_nEnergy << "\t"
             << m_nEnergy2 << "\t"<<"\t"
             <<m_unEnergy2PerFoodItem<<std::endl;}
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions")
