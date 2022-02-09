/* Include the controller definition */
#include "footbot_foraging.h"
/* Function definitions for XML parsing */
#include <argos2/common/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos2/common/utility/math/vector2.h>
#include <argos2/common/utility/math/angles.h>
/* Logging */
#include <argos2/common/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CFootBotForaging::SFoodData::SFoodData() :
   HasFoodItem(false),
   FoodItemIdx(0),
   TotalFoodItems(0) {}

void CFootBotForaging::SFoodData::Reset() {
   HasFoodItem = false;
   FoodItemIdx = 0;
   TotalFoodItems = 0;
}

/****************************************/
/****************************************/

CFootBotForaging::SDiffusionParams::SDiffusionParams() :
   GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotForaging::SDiffusionParams::Init(TConfigurationNode& t_node) {
   try {
      CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
      GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
      GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                               ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
      GetNodeAttribute(t_node, "delta", Delta);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller diffusion parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

CFootBotForaging::SStateData::SStateData() :
   ProbRange(0.0f, 1.0f) {}

void CFootBotForaging::SStateData::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "initial_rest_to_explore_prob", InitialRestToExploreProb);
      GetNodeAttribute(t_node, "initial_explore_to_rest_prob", InitialExploreToRestProb);
      GetNodeAttribute(t_node, "food_rule_explore_to_rest_delta_prob", FoodRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "food_rule_rest_to_explore_delta_prob", FoodRuleRestToExploreDeltaProb);
      GetNodeAttribute(t_node, "collision_rule_explore_to_rest_delta_prob", CollisionRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "social_rule_rest_to_explore_delta_prob", SocialRuleRestToExploreDeltaProb);
      GetNodeAttribute(t_node, "social_rule_explore_to_rest_delta_prob", SocialRuleExploreToRestDeltaProb);
      GetNodeAttribute(t_node, "minimum_resting_time", MinimumRestingTime);
      GetNodeAttribute(t_node, "minimum_unsuccessful_explore_time", MinimumUnsuccessfulExploreTime);
      GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);
      GetNodeAttribute(t_node, "robot_typte", RobotType);
     	

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
}

void CFootBotForaging::SStateData::Reset() {
   State = STATE_RESTING;
   InNest = true;
   RestToExploreProb = InitialRestToExploreProb;
   ExploreToRestProb = InitialExploreToRestProb;
   TimeExploringUnsuccessfully = 0;
   /* Initially the robot is resting, and by setting RestingTime to MinimumRestingTime
      we force the robots to make a decision at the experiment start. If
      instead we set RestingTime to zero, we would have to wait till RestingTime reaches
      MinimumRestingTime before something happens, which is just a waste of time. */
   TimeRested = MinimumRestingTime;
   TimeSearchingForPlaceInNest = 0;
   food_color=0;
}

/****************************************/
/****************************************/

CFootBotForaging::CFootBotForaging() :
   m_pcWheels(NULL),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcProximity(NULL),
   m_pcLight(NULL),
   m_pcGround(NULL),
   m_pcRNG(NULL), 
   //delta_weights(NULL),
   m_pfInputs(NULL),
   m_pfHiddens(NULL),
   m_pfOutputs(NULL),
   m_pfWeightsIH(NULL), 
   m_pfWeightsHO(NULL),
   DeltaH(NULL),
   DeltaO(NULL),
   m_cRandomW(-16.0f,16.0f){}

/****************************************/
/****************************************/

void CFootBotForaging::Init(TConfigurationNode& t_node) {
   try {
      /*
       * Initialize sensors/actuators
       */
      m_pcWheels    = dynamic_cast<CCI_FootBotWheelsActuator* >  (GetRobot().GetActuator("footbot_wheels"      ));
      m_pcLEDs      = dynamic_cast<CCI_FootBotLedsActuator* >    (GetRobot().GetActuator("footbot_leds"        ));
      m_pcRABA      = dynamic_cast<CCI_RangeAndBearingActuator*> (GetRobot().GetActuator("range_and_bearing"   ));
      m_pcRABS      = dynamic_cast<CCI_RangeAndBearingSensor*>   (GetRobot().GetSensor  ("range_and_bearing"   ));
      m_pcProximity = dynamic_cast<CCI_FootBotProximitySensor*>  (GetRobot().GetSensor  ("footbot_proximity"   ));
      m_pcLight     = dynamic_cast<CCI_FootBotLightSensor*>      (GetRobot().GetSensor  ("footbot_light"       ));
      m_pcGround    = dynamic_cast<CCI_FootBotMotorGroundSensor*>(GetRobot().GetSensor  ("footbot_motor_ground"));
      /*
       * Parse XML parameters
       */
      /* Diffusion algorithm */
      m_sDiffusionParams.Init(GetNode(t_node, "diffusion"));
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Controller state */
      m_sStateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot foraging controller for robot \"" << GetRobot().GetRobotId() << "\"", ex);
   }

  
   /*
    * Initialize other stuff
    */
   /* Create a random number generator. We use the 'argos' category so that creation, reset, seeding and cleanup are managed by ARGoS. */
   m_pcRNG = CARGoSRandom::CreateRNG("argos");
   key=1;
   spiral=false;
   spiral_state=1;
   spiral_inc_step=0;
   spiral_key=0;
   //send_id.push_back(CVector2((UInt8)500,0));

   /////neural network///////
   if (m_sStateData.RobotType ==2)
    {
      m_unNumberOfInputs = 2;
      m_unNumberOfHiddens = 10;
      m_unNumberOfOutputs = 1;
      m_unNumberOfWeightsIH = (m_unNumberOfInputs + 1) * m_unNumberOfHiddens;
      m_unNumberOfWeightsHO = (m_unNumberOfHiddens + 1) * m_unNumberOfOutputs;
     

     m_pfInputs = new Real[m_unNumberOfInputs];
     m_pfHiddens = new Real[m_unNumberOfHiddens];
     m_pfOutputs = new Real[m_unNumberOfOutputs];
     m_pfWeightsIH = new Real[m_unNumberOfWeightsIH];
     m_pfWeightsHO = new Real[m_unNumberOfWeightsHO];
     //delta_weights = new Real[m_unNumberOfWeightsIH + m_unNumberOfWeightsHO];
     DeltaO = new Real[m_unNumberOfOutputs];
     DeltaH = new Real[m_unNumberOfHiddens];

     m_cRandomW.SetMin(-3.9f);
     m_cRandomW.SetMax(3.9f);
     Real rand_x=0.5;
     for(UInt32 i = 0; i < m_unNumberOfWeightsIH;i++) {
       m_pfWeightsIH[i]=rand_x;
          // x+=0.1; // m_pcRNG->Uniform(m_cRandomW);
        }
     for(UInt32 i = 0; i < m_unNumberOfWeightsHO;i++) {
         m_pfWeightsHO[i]=rand_x;
        // x+=0.1;// m_pcRNG->Uniform(m_cRandomW);
    }
      for(UInt32 i = 0; i < m_unNumberOfOutputs;i++){
         DeltaO[i]=0;
       }
     for(UInt32 i = 0; i < m_unNumberOfOutputs;i++){
         m_pfOutputs[i]=0;
       }
     for(UInt32 i = 0; i < m_unNumberOfHiddens;i++){
         DeltaH[i]=0;
       }
     m_pfInputs[0]=0;
     m_pfInputs[1]=0;
     //m_pfInputs[2]=5;
     eta = 0.9;
     //train faz
     Real x,y,v,target,output;
      CRadians rx;//ry
     for(UInt32 i = 0; i < 100;i++) {
        x=m_pcRNG->Uniform(m_cRandomW);
        y=m_pcRNG->Uniform(m_cRandomW);
        if((x<=2.0f &&x>=-2.0f) &&( y<=2.0f &&y>=-2.0f)){
         i--;}
        else{
          //rx.SetValue(x*y);
          //ry.SetValue(y);
          v=y/sqrt(x*x+y*y);
          target=Abs(v);
          EstimateQuantity();
          output = m_pfOutputs[0];
         // std::cout<<"target"<<target<<std::endl;
          BackPropagateError(output,target,0); 
          
        }//end of else
       }//end of for
         std::cout<<"end of initialization"<<std::endl;
   }//end of if
   Reset();
}

/****************************************/
/****************************************/

void CFootBotForaging::ControlStep() {
   switch(m_sStateData.State) {
      case SStateData::STATE_RESTING: {
         Rest();
         break;
      }
      case SStateData::STATE_EXPLORING: {
         Explore();
         break;
      }
      case SStateData::STATE_RETURN_TO_NEST: {
         ReturnToNest();
         break;
      }
      case SStateData::STATE_TURNING: {
         Turning();
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::Reset() {
   /* Reset robot state */
   m_sStateData.Reset();
   /* Reset food data */
   m_sFoodData.Reset();
   /* Set LED color */
   if(m_sStateData.RobotType==2)
      m_pcLEDs->SetAllColors(CColor::YELLOW);
   else
      m_pcLEDs->SetAllColors(CColor::RED);
   /* Clear up the last exploration result */
   m_eLastExplorationResult = LAST_EXPLORATION_NONE;
   TRangeAndBearingReceivedPacket::TRangeAndBearingData tData;
   tData[0] = LAST_EXPLORATION_NONE;
   m_pcRABA->SetData(tData);
}

/****************************************/
/****************************************/

void CFootBotForaging::Destroy(){
 if( m_pfInputs ) delete[] m_pfInputs;
   m_pfInputs = NULL;
   m_unNumberOfInputs = 0;

 if( m_pfHiddens ) delete[] m_pfHiddens;
   m_pfHiddens = NULL;
   m_unNumberOfHiddens = 0;

 if( m_pfWeightsIH ) delete[] m_pfWeightsIH;
   m_pfWeightsIH = NULL;
   m_unNumberOfWeightsIH = 0;


 if( m_pfWeightsHO ) delete[] m_pfWeightsHO;
   m_pfWeightsHO = NULL;
   m_unNumberOfWeightsHO = 0;

 //if(delta_weights ) delete[] delta_weights;
   //delta_weights = NULL;
 
 if(DeltaH ) delete[] DeltaH;
   DeltaH = NULL;
 if(DeltaO ) delete[] DeltaO;
   DeltaO = NULL;

}
/****************************************/
/****************************************/
void CFootBotForaging::UpdateState() {
   /* Reset state flags */
   m_sStateData.InNest = false;
   m_sStateData.OnFood = false;
   m_sStateData.OnGround = false;

   /* Read stuff from the ground sensor */
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
   /*
    * You can say whether you are in the nest by checking the ground sensor
    * placed close to the wheel motors. It returns a value between 0 and 1.
    * It is 1 when the robot is on a white area, it is 0 when the robot
    * is on a black area and it is around 0.5 when the robot is on a gray area. 
    * The foot-bot has 4 sensors like this, two in the front
    * (corresponding to readings 0 and 1) and two in the back (corresponding
    * to reading 2 and 3).  Here we want the back sensors (readings 2 and 3) to
    * tell us whether we are on gray: if so, the robot is completely in the nest,
    * otherwise it's outside.
    */
   if(tGroundReads[2].Value > 0.49f && 
      tGroundReads[2].Value < 0.5f &&
      tGroundReads[3].Value > 0.49f &&
      tGroundReads[3].Value < 0.5f) {
      m_sStateData.InNest = true;
	 m_sStateData.OnFood = false;
	 m_sStateData.OnGround = false;
   }
   else if(tGroundReads[2].Value >= 0.0f && 
      	   tGroundReads[2].Value < 0.2f &&
      	   tGroundReads[3].Value >= 0.0f &&
	   tGroundReads[3].Value < 0.2f){
      m_sStateData.InNest = false; 
	 m_sStateData.OnFood =true ;
	 m_sStateData.OnGround = false;
   }
   else if(tGroundReads[2].Value > 0.5f && 
      	   tGroundReads[2].Value < 0.9f &&
      	   tGroundReads[3].Value > 0.5f &&
	   tGroundReads[3].Value < 0.9f){
	 m_sStateData.InNest = false; 
	 m_sStateData.OnFood = true;
	 m_sStateData.OnGround = false;
   }else{ m_sStateData.InNest = false;
	 m_sStateData.OnFood = true;
	 m_sStateData.OnGround = false;
}

	
}

/****************************************/
/****************************************/

CVector2 CFootBotForaging::CalculateVectorToLight() {
   /* Get readings from light sensor */
   const CCI_FootBotLightSensor::TReadings& tLightReads = m_pcLight->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
    //std::cout<<"light angle"<<cAccumulator.Angle()<<std::endl;
   /* If the light was perceived, return the vector */
   if(cAccumulator.Length() > 0.0f) {
      return CVector2(1.0f, cAccumulator.Angle());
   }
   /* Otherwise, return zero */
   else {
      return CVector2();
   }
}

/****************************************/
/********************for scout robots********************/

CVector2 CFootBotForaging::DiffusionVector(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle is far enough,
      ignore the vector and go straight, otherwise return it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector.Angle()) &&
      cDiffusionVector.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector.Normalize();
      return -cDiffusionVector;
   }
}
/****************************************/
/*********************for forager robots********************/

CVector2 CFootBotForaging::DiffusionVector2(bool& b_collision) {
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cDiffusionVector2;
   for(size_t i = 0; i < 4; ++i) {
      cDiffusionVector2 += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   for(size_t i =23; i < 20; --i) {
      cDiffusionVector2 += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /* If the angle of the vector is small enough and the closest obstacle is far enough,
      ignore the vector and go straight, otherwise return it */
   if(m_sDiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cDiffusionVector2.Angle()) &&
      cDiffusionVector2.Length() < m_sDiffusionParams.Delta ) {
      b_collision = false;
      return CVector2::X;
   }
   else {
      b_collision = true;
      cDiffusionVector2.Normalize();
      return -cDiffusionVector2;
   }
}

/****************************************/
/****************************************/

void CFootBotForaging::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

   /* Turning state switching conditions */
   if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
      /* No Turn, heading angle very small */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
   }
   else if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
      /* Hard Turn, heading angle very large */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
   }
   else if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN &&
           Abs(cHeadingAngle) > m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
      /* Soft Turn, heading angle in between the two cases */
      m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
   }

   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }

      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }

      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }

   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CFootBotForaging::Rest() {

    /* This helps to implement different kind of controllers*/  
     std::string str= m_sStateData.robot_id.substr(m_sStateData.robot_id.find("_")+1,m_sStateData.robot_id.size()-m_sStateData.robot_id.find("_")); 
     const char * c = str.c_str();
     UInt8 id = atoi(c);
     
	switch (m_sStateData.RobotType){
     TRangeAndBearingReceivedPacket::TRangeAndBearingData tData;
     
    /* This is for scouts */
	   case 1: {
			  /* If we have stayed here enough, probabilistically switch to 'exploring' */
		   if(m_sStateData.TimeRested > m_sStateData.MinimumRestingTime &&
			 m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.RestToExploreProb) {
			 m_pcLEDs->SetAllColors(CColor::GREEN);
			 m_sStateData.State = SStateData::STATE_EXPLORING;
                         m_sStateData.EnergyState = SStateData::STATE_LEAVING_HOME;
			 m_sStateData.TimeRested = 0;
		   }
		   else {
			 ++m_sStateData.TimeRested;
			 /* Be sure not to send the last exploration result multiple times */
			 if(m_sStateData.TimeRested == 1) {
			    TRangeAndBearingReceivedPacket::TRangeAndBearingData tData;
			    tData[0] = LAST_EXPLORATION_NONE;
			    tData[1] =	0;
			    tData[2] =	0;
	   		    tData[9] = 1;
			    m_pcRABA->SetData(tData);
			 }
		   }	

	   	break;
	   }
    /* This is for hives */
     case 2: {
                Real v,max=0,imax=0;
                UpdateState();
                if(!m_sStateData.InNest){
                   m_pcWheels->SetLinearVelocity(2.0f, 2.0f);
                   m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;}	
	        const CCI_RangeAndBearingSensor::TLastReceivedPackets& tPackets = m_pcRABS->GetLastReceivedPackets();
		for(CCI_RangeAndBearingSensor::TLastReceivedPackets::const_iterator it = tPackets.begin();
		    it != tPackets.end();
		     ++it) {
                    if(hive_time){
                                   if( hive_stack.size()>0){      
                                          for(UInt32 i = 0; i < hive_stack.size();i++) {
                                               m_pfInputs[0] = hive_stack[i].GetX(); //x = hive_stack[i].GetX();
                                               m_pfInputs[1] = hive_stack[i].GetY();//y = hive_stack[i].GetY();
                                               EstimateQuantity();
                                               //output =
                                               v = Floor(10*m_pfOutputs[0]);//v = y/sqrt(x*x+y*y);
                                               if(v>max){
                                                   max = v;
                                                   imax = i;
                                               }
                                           }
                                            //m_pfInputs[0] = hive_stack[imax].GetX();     //m_sStateData.Food.GetX();
                                            //m_pfInputs[1] = hive_stack[imax].GetY();
                                            hive_time = false;
                                             m_sStateData.Food = CVector2(hive_stack[imax].GetX(),hive_stack[imax].GetY());
                                             UInt32 x_convert=(Abs(m_sStateData.Food.GetX())*100);
				             UInt32 y_convert=(Abs(m_sStateData.Food.GetY())*100);
				             UInt8 a ;
				             if(m_sStateData.Food.GetX()>=0&&m_sStateData.Food.GetY()>=0)
				                a=1;
				             else if(m_sStateData.Food.GetX()>=0&&m_sStateData.Food.GetY()<0)
				                    a=2;
				                  else if(m_sStateData.Food.GetX()<0&&m_sStateData.Food.GetY()>=0)
				                        a=3;
				                        else if(m_sStateData.Food.GetX()<0&&m_sStateData.Food.GetY()<0)
				                                a=4;
					    tData[0] = it->second.Data[0];
					    tData[1] = (UInt8)(x_convert/100);
					    tData[2] = (UInt8)(x_convert%100);
					    tData[3] = a;
					    tData[4] = (UInt8)(y_convert/100);
				            tData[5] = (UInt8)(y_convert%100);
		                            tData[8] = id;
                                            tData[9] = 2;
                                            hive_stack[imax].Set(hive_stack[hive_stack.size()-1].GetX()
                                                                 ,hive_stack[hive_stack.size()-1].GetY());
                                              hive_stack.pop_back();
                                            //hive_stack.clear(); 
		                             //m_pfInputs[2]=color;
		                             //index = Area(m_sStateData.Food.GetX(),m_sStateData.Food.GetY());
				             //EstimateQuantity();
				             //BackPropagateError(output,(output+0.5f),index);     
				             // }//end of if index
		                             m_sStateData.food_value = max; //Floor(10*m_pfOutputs[0]);
		                             std::cout<<"EstimateQuantity-food value "<<m_sStateData.food_value<<std::endl;
		                             m_sStateData.employers = 0;
		                             //if(m_sStateData.employers>(m_sStateData.NumberOfForagers*3/4)){
		                             if(m_sStateData.food_value >= (3/4 * m_sStateData.NumberOfForagers)){
		                                     //broadcast message
						      tData[7] = 1;
		                              }else{
							 tData[7] = 2;
							 m_sStateData.employers = m_sStateData.food_value;
		                                   }//end of else
		                             std::cout<<"message type "<<(Real)tData[7]<<"go employers"<<m_sStateData.employers<<std::endl;
					     m_pcRABA->SetData(tData);
					     m_pcLEDs->SetAllColors(CColor::MAGENTA);
					     m_sStateData.State = SStateData::STATE_EXPLORING;
		                             m_sStateData.EnergyState = SStateData::STATE_RESTING_ROBOT;//?
                                        }
                                     }//end of if(hive_time)
		    switch(it->second.Data[9]){	
		           case 1:{
		                  switch(it->second.Data[0]) {
		                        case LAST_EXPLORATION_SUCCESSFUL: {

/*data packet protocol:
tData[0]-> for scout robot means find food or not
tData[1]-tData[6]-> address of source food
tData[8]-> id of robot
tData[9]-> type of robot 1->scout 2->hive 3->forager
tData[7]-> type of msg.  
for robot hive :  tData[7]=1 means msg is broadcast and every one should be go to the this source food
                  tData[7]=2 means msg is multicast and every one that get this msg should be reply with another msg
                  tData[7]=4 means msg is unicast to robot with robot number tData[8] to go to the source food
for robot forager:tData[7]=3 means im ready for go to source food
                  tData[7]=5 means im going to source food
                  tData[7]=6 means i'm back to home from exploring
*/                                   //std::cout<<"food_color "<<it->second.Data[6]<<std::endl;
                                     std::cout<<"employer "<<m_sStateData.NumberOfForagers<<std::endl;
                                     //size_t color = it->second.Data[6]; 
                                     m_sStateData.employers=0;
                                     Real a,b,output;
                                    // UInt32 index=0;
		                     switch(it->second.Data[3]){
		                              case 1:{a=1;b=1;break;}
		                              case 2:{a=1;b=-1;break;}
		                              case 3:{a=-1;b=1;break;}
		                              case 4:{a=-1;b=-1;break;}
		                        } 
				     m_sStateData.Food = CVector2((((Real)it->second.Data[1])
		                                                     +((Real)it->second.Data[2])/100)*(a),
		                                                     (((Real)it->second.Data[4])
		                                                     +((Real)it->second.Data[5])/100)*(b));
                                     hive_stack.push_back(CVector2(m_sStateData.Food.GetX(),m_sStateData.Food.GetY()));
                                     
				     break;
				 }
				  case LAST_EXPLORATION_UNSUCCESSFUL: {
				     break;
				  }
			    }//end of switch 
			 break; }//end of case=1
                         case 2:{
                                    if(id!=it->second.Data[8]){
                                    tData[0] = it->second.Data[0];
			            tData[1] = it->second.Data[1];
				    tData[2] = it->second.Data[2];
				    tData[3] = it->second.Data[3];	   
			            tData[4] = it->second.Data[4];
	                            tData[5] = it->second.Data[5];
	                            tData[6] = it->second.Data[6];
	                            tData[7] = it->second.Data[7];
		                    tData[8] = it->second.Data[8];
		                    tData[9] = it->second.Data[9];}
                                         break;
                         }//end of case=2
                         case 3:{
                                   switch(it->second.Data[7]) {
                                           case 3:{
                                                   //if(robots_send==0){ 
                                                   
                                                    if(m_sStateData.employers>0){
                                                        send_id.push_back(CVector2(it->second.Data[8],0));
                                                        m_sStateData.employers--;
                                 //   std::cout<<"message type 4- robot "<<(Real)it->second.Data[8]<<"go employers"<<m_sStateData.employers<<std::endl;
                                                     }else{
                                                     if(send_id.size()>0){
                                                     tData[0] = it->second.Data[0];
					             tData[1] = it->second.Data[1];
						     tData[2] = it->second.Data[2];
						     tData[3] = it->second.Data[3];	   
						     tData[4] = it->second.Data[4];
				                     tData[5] = it->second.Data[5];
				                     tData[6] = it->second.Data[6];
				                     tData[7] = 4;
		                                     tData[8] = send_id[send_id.size()-1].GetX();
		                                     tData[9] = 2;
                                                     m_pcRABA->SetData(tData);
                                                     m_sStateData.State = SStateData::STATE_EXPLORING;
                                                     m_sStateData.EnergyState = SStateData::STATE_RESTING_ROBOT;//?
                                    // std::cout<<"ms 4- robot "<<(Real)tData[8]<<"size stack"<<(Real)send_id.size()<<std::endl;
                                     
                                                  } }break;}//end of case 3
                                           case 5:{
                                                 if(send_id.size()>0){
                                                   if(send_id[send_id.size()-1].GetX()==it->second.Data[8])
                                                       send_id.pop_back();
                                                 }else{
                                                  send_id.clear();
                                                  m_sStateData.employers=0;}
                                                    break;}//end of case 5
                                           case 6:{
					                Real a,b,output;
                                                        UInt32 index;
						        switch(it->second.Data[3]){
							      case 1:{a=1;b=1;break;}
							      case 2:{a=1;b=-1;break;}
							      case 3:{a=-1;b=1;break;}
							      case 4:{a=-1;b=-1;break;}
							} 
						        CVector2 old_food = CVector2((((Real)it->second.Data[1])
							                             +((Real)it->second.Data[2])/100)*(a),
							                             (((Real)it->second.Data[4])
							                             +((Real)it->second.Data[5])/100)*(b));
                                                        m_pfInputs[0] = old_food.GetX();
                                                        m_pfInputs[1] =  old_food.GetY();
                                                        //index = Area(old_food.GetX(),old_food.GetY());
                                                        index=0;
                                                        if(index!=8){  
                                                           switch( it->second.Data[0]){ 
                                                           	case LAST_EXPLORATION_SUCCESSFUL:{
                                                                       // m_pfInputs[2]= it->second.Data[6];
                                                                        EstimateQuantity();
                                                                        output = m_pfOutputs[index];
                                                           		BackPropagateError(output,(output-0.1f),index);
                                                                         break;
                                                                        }
                                                                case LAST_EXPLORATION_UNSUCCESSFUL:{
                                                                       //m_pfInputs[2]= 5;//not found color
                                                                        EstimateQuantity();
                                                                        output = m_pfOutputs[index];
                                                           		BackPropagateError(output,(output-0.2f),index);
                                                                          break;}
                                                            }//end of switch
                                                          std::cout<<"ms 6- robot "<<(Real) it->second.Data[8]<<send_id.size()<<std::endl;
                                                         }//end of if index
                                                         break;
                                                        }//end of case 6
                                   }//end of switch(it->second.Data[7])
                         break;
                         }//end of case=3
                       }//end of switch
		   }//end of for
          /* Clear the last received packets */
          //m_pcRABS->ClearRABReceivedPackets();
		break;
	   }//end of case robotype=2
    /* This is for foragers */
	   case 3: {
                 ++m_sStateData.TimeRested;
			 /* Be sure not to send the last exploration result multiple times */
			 if(m_sStateData.TimeRested == 1) {
			    TRangeAndBearingReceivedPacket::TRangeAndBearingData tData;
			    tData[0] = LAST_EXPLORATION_NONE;
			    tData[1] =	0;
			    tData[2] =	0;
	   		    tData[9] = 3;
			    m_pcRABA->SetData(tData);
                            //std::cout<<"okkkkkk"<<std::endl;
			 }
		 const CCI_RangeAndBearingSensor::TLastReceivedPackets& tPackets = m_pcRABS->GetLastReceivedPackets();
		 for(CCI_RangeAndBearingSensor::TLastReceivedPackets::const_iterator it = tPackets.begin();
		     it != tPackets.end();
		     ++it) {
		    if(it->second.Data[9]==2){	
					switch(it->second.Data[7]){
                                               case 1:{
                                                        Real a,b;
                                                        switch(it->second.Data[3]){
                                                              case 1:{a=1;b=1;break;}
                                                              case 2:{a=1;b=-1;break;}
                                                              case 3:{a=-1;b=1;break;}
                                                              case 4:{a=-1;b=-1;break;}
                                                        } 
					                m_sStateData.Food = CVector2((((Real)it->second.Data[1])
                                                                                     +((Real)it->second.Data[2])/100)*(a),
                                                                                     (((Real)it->second.Data[4])
                                                                                     +((Real)it->second.Data[5])/100)*(b));
					                m_pcLEDs->SetAllColors(CColor::MAGENTA);
                                                        m_sStateData.heading_angle=0;
                                                        FindFood();
					                m_sStateData.State = SStateData::STATE_EXPLORING;
                                                        m_sStateData.EnergyState = SStateData::STATE_LEAVING_HOME;
                                                        m_sStateData.TimeRested=0;
                                                        break;}//end of case 1
                                               case 2:{
                                                     tData[0] = it->second.Data[8];
					             tData[1] = it->second.Data[1];
						     tData[2] = it->second.Data[2];
						     tData[3] = it->second.Data[3];	   
						     tData[4] = it->second.Data[4];
				                     tData[5] = it->second.Data[5];
				                     tData[6] = it->second.Data[6];
				                     tData[7] = 3;
		                                     tData[8] = id;
		                                     tData[9] = 3;
                                                     std::cout<<"message type 3- robot "<<(Real)tData[8]<< std::endl;
                                                     m_pcRABA->SetData(tData);break;}//end of case 2
                                               case 4:{
                                                      
                                                        if(it->second.Data[8] == id ){
		                                             tData[0] = it->second.Data[0];
							     tData[1] = it->second.Data[1];
							     tData[2] = it->second.Data[2];
							     tData[3] = it->second.Data[3];	   
							     tData[4] = it->second.Data[4];
						             tData[5] = it->second.Data[5];
						             tData[6] = it->second.Data[6];
						             tData[7] = 5;
				                             tData[8] = id;
				                             tData[9] = 3;
		                                         std::cout<<"message type 4-get robot "<<(Real)id<< std::endl;
		                                              m_pcRABA->SetData(tData);
					                Real a,b;
                                                        switch(it->second.Data[3]){
                                                              case 1:{a=1;b=1;break;}
                                                              case 2:{a=1;b=-1;break;}
                                                              case 3:{a=-1;b=1;break;}
                                                              case 4:{a=-1;b=-1;break;}
                                                        } 
					                m_sStateData.Food = CVector2((((Real)it->second.Data[1])
                                                                                     +((Real)it->second.Data[2])/100)*(a),
                                                                                     (((Real)it->second.Data[4])
                                                                                     +((Real)it->second.Data[5])/100)*(b));
					                m_pcLEDs->SetAllColors(CColor::MAGENTA);
                                                        m_sStateData.heading_angle=0;
                                                        FindFood();
					                m_sStateData.State = SStateData::STATE_EXPLORING;
                                                        m_sStateData.EnergyState = SStateData::STATE_LEAVING_HOME;
                                                        m_sStateData.TimeRested=0;}//end of if
                                                       
                                                        break;}//end of case4
                                                
					}//end of switch(it->second.Data[7])
			      }//end of if
		   }//end of for
          /* Clear the last received packets */
          //m_pcRABS->ClearRABReceivedPackets();
		break;
	   }//end of case 3	
    }//end of switch

}

/****************************************/
/****************************************/

void CFootBotForaging::Explore() {
    
    const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcGround->GetReadings();
    /* This hepls to implement different kind of controllers*/  
	switch (m_sStateData.RobotType){
    /* This is for explorers */
	   case 1: {
                   /*//gray 0.498039 blue 0.114 yellow 0.886 green 0.587
                   if(!(tGroundReads[0].Value == 1 && tGroundReads[1].Value == 1 ) )
                      if(!(tGroundReads[0].Value > 0.49 && tGroundReads[0].Value < 0.5))
                         if(!( tGroundReads[1].Value > 0.49 && tGroundReads[1].Value < 0.5) ){
                         std::cout<<"color 0 ="<< tGroundReads[0].Value<<"color 1 ="<< tGroundReads[1].Value<<std::endl;*/
                         

		   /* We switch to 'return to nest' in two situations:
		    * 1. if we have a food item
		    * 2. if we have not found a food item for some time;
		    *    in this case, the switch is probabilistic
		    */
		   bool bReturnToNest(false);
		   /*
		    * Test the first condition: have we found a food item?
		    * NOTE: the food data is updated by the loop functions, so
		    * here we just need to read it
		    */
		   if(m_sFoodData.HasFoodItem) {
			 /* Apply the food rule, decreasing ExploreToRestProb and increasing RestToExploreProb */
			 m_sStateData.ExploreToRestProb -= m_sStateData.FoodRuleExploreToRestDeltaProb;
			 m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
			 m_sStateData.RestToExploreProb += m_sStateData.FoodRuleRestToExploreDeltaProb;
			 m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
			 /* Store the result of the expedition */
			 m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
			 /* Switch to 'return to nest' */
			 bReturnToNest = true;
                         m_sStateData.EnergyState = SStateData::STATE_MOVE_TO_HOME;
		   }
		   /* Test the second condition: we probabilistically switch to 'return to nest' if we
			 have been wandering for some time and found nothing */
		   else if(m_sStateData.TimeExploringUnsuccessfully > m_sStateData.MinimumUnsuccessfulExploreTime) {
			 if (m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.ExploreToRestProb) {
			    /* Store the result of the expedition */
			    m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
			    /* Switch to 'return to nest' */
			    bReturnToNest = true;
                            m_sStateData.EnergyState = SStateData::STATE_HOMING;
			 }
			 else {
			    /* Apply the food rule, increasing ExploreToRestProb and decreasing RestToExploreProb */
			    m_sStateData.ExploreToRestProb += m_sStateData.FoodRuleExploreToRestDeltaProb;
			    m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
			    m_sStateData.RestToExploreProb -= m_sStateData.FoodRuleRestToExploreDeltaProb;
			    m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
			 }
		   }
		   /* So, do we return to the nest now? */
		   if(bReturnToNest) {
			 /* Yes, we do! */
                         m_sStateData.EnergyState = SStateData::STATE_MOVE_TO_HOME;
			 m_sStateData.TimeExploringUnsuccessfully = 0;
			 m_sStateData.TimeSearchingForPlaceInNest = 0;
			 m_pcLEDs->SetAllColors(CColor::BLUE);
			 m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
		   }
		   else {
			 /* No, perform the actual exploration */
                         m_sStateData.EnergyState = SStateData::STATE_RANDOM_WALK;
			 ++m_sStateData.TimeExploringUnsuccessfully;
			 UpdateState();
			 /* Get the diffusion vector to perform obstacle avoidance */
			 bool bCollision;
			 CVector2 cDiffusion = DiffusionVector(bCollision);
			 /* Apply the collision rule, if a collision avoidance happened */
			 if(bCollision) {
                            m_sStateData.EnergyState = SStateData::STATE_AVOIDANCE;
			    /* Collision avoidance happened, increase ExploreToRestProb and decrease RestToExploreProb */
			    m_sStateData.ExploreToRestProb += m_sStateData.CollisionRuleExploreToRestDeltaProb;
			    m_sStateData.ProbRange.TruncValue(m_sStateData.ExploreToRestProb);
			    m_sStateData.RestToExploreProb -= m_sStateData.CollisionRuleExploreToRestDeltaProb;
			    m_sStateData.ProbRange.TruncValue(m_sStateData.RestToExploreProb);
			 }
			 /*
			  * If we are in the nest, we combine antiphototaxis with obstacle avoidance
			  * Outside the nest, we just use the diffusion vector
			  */
			 if(m_sStateData.InNest) {
                            m_sStateData.EnergyState = SStateData::STATE_LEAVING_HOME;
			    /*
				* The vector returned by CalculateVectorToLight() points to
				* the light. Thus, the minus sign is because we want to go away
				* from the light.
				*/
			    SetWheelSpeedsFromVector(
				  m_sWheelTurningParams.MaxSpeed * cDiffusion -
				  m_sWheelTurningParams.MaxSpeed * 0.25f * CalculateVectorToLight());
			 }
			 else {
			    /* Use the diffusion vector only */
                            m_sStateData.EnergyState = SStateData::STATE_RANDOM_WALK;
			    SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed * cDiffusion);
			 }
		   }

	   	break;
	   }

    /* This is for hive robot */
	   case 2: {
		//m_pcWheels->SetLinearVelocity(2.0f, -2.0f);    
		TRangeAndBearingReceivedPacket::TRangeAndBearingData tData;
		tData[0] = LAST_EXPLORATION_NONE;
		tData[1] =	0;
		tData[2] =	0;
                tData[3] =	0;
                tData[4] =	0;
                tData[5] =	0;
                tData[6] =	0;
	        tData[9] = 2;
		m_pcRABA->SetData(tData);
		m_pcLEDs->SetAllColors(CColor::ORANGE);
                UpdateState();
                if(!m_sStateData.InNest){
                   m_pcWheels->SetLinearVelocity(2.0f, 2.0f);
                   m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;}
                else	
		   m_sStateData.State = SStateData::STATE_RESTING;	
		break;
	   }


    /* This is for foragers */
	   case 3: {
		   if(!m_sStateData.InNest)
                      m_sStateData.EnergyState = SStateData::STATE_SCAN_AREA;
                   if(spiral==1){
                        m_sStateData.EnergyState = SStateData::STATE_SCAN_AREA;
                        SpiralSearch();	}
		   if (m_sStateData.angle != 0){
                       key=7;
                       m_sStateData.State = SStateData::STATE_TURNING;
                       Turning();
		   }
		   else if (m_sFoodData.HasFoodItem == false && m_sStateData.TimeExploringUnsuccessfully <= m_sStateData.MinimumUnsuccessfulExploreTime)
		   {
			 bool bCollision;
			 DiffusionVector2(bCollision);
 			if(!bCollision){
                               if( m_sStateData.distance>0){
                                     m_sStateData.distance--;
                                 //  std::cout<<"Distance="<< m_sStateData.distance<<std::endl;
				     m_pcWheels->SetLinearVelocity(10.0f, 10.0f);
				}else{
                                      m_sStateData.EnergyState = SStateData::STATE_SCAN_AREA;
                                      spiral =1;
                                    //  std::cout<<"spiral-distance"<<std::endl;
                                      SpiralSearch();
                                           }

			}
			else
			{       //if red tGroundReads[0].Value= 0.299
                                //std::cout<<"color 0 ="<< tGroundReads[0].Value<<"color 1 ="<< tGroundReads[1].Value<<std::endl;   
                                if((tGroundReads[0].Value<0.3 && tGroundReads[0].Value>0.28) 
                                    || (tGroundReads[1].Value<0.3 && tGroundReads[1].Value>0.28) ){
                                      spiral =1;
                                      //std::cout<<"spiral-color"<<std::endl;
                                      SpiralSearch();}
                                else{
                               // std::cout<<"obstacle!"<<std::endl;
                                m_sStateData.State = SStateData::STATE_TURNING;
                                m_sStateData.EnergyState = SStateData::STATE_AVOIDANCE;
                                CollisionAvoidance();

				m_pcWheels->SetLinearVelocity(10.0f, 10.0f);}
			}
			++m_sStateData.TimeExploringUnsuccessfully;
			
		   }
		   else
		   {
                        if(m_sFoodData.HasFoodItem){
                           m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
                           m_sStateData.EnergyState = SStateData::STATE_MOVE_TO_HOME;  
                        }else{
			   m_sStateData.EnergyState = SStateData::STATE_HOMING;
                           m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
                             }
			m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;	    		
		   }

		 
		break;
	   }//end of case	
    }//end of switch

}

/****************************************/
/****************************************/

void CFootBotForaging::ReturnToNest() {

    /* This hepls to implement different kind of controllers*/  
	switch (m_sStateData.RobotType){
    /* This is for explorers */
	   case 1: {
		   UpdateState();
		   /* Are we in the nest? */
		   if(m_sStateData.InNest) {
                          m_sStateData.EnergyState = SStateData::STATE_DEPOSIT_FOOD;
			 //m_sFoodData.HasFoodItem = false;
			 /* Have we looked for a place long enough? */
			 if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime) {
			    /* Yes, stop the wheels... */
			    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
			    /* Tell people about the last exploration attempt */
			    TRangeAndBearingReceivedPacket::TRangeAndBearingData tData;
                            if(m_eLastExplorationResult==LAST_EXPLORATION_SUCCESSFUL){
			    tData[0] = m_eLastExplorationResult;
                            std::cout<<"Food x , Food y ="<<m_sStateData.Food.GetX()<<","<<m_sStateData.Food.GetY() << std::endl;
                            UInt32 x_convert=(Abs(m_sStateData.Food.GetX())*100);
                            UInt32 y_convert=(Abs(m_sStateData.Food.GetY())*100);
                            UInt8 a ;
                            if(m_sStateData.Food.GetX()>=0&&m_sStateData.Food.GetY()>=0)
                                a=1;
                            else if(m_sStateData.Food.GetX()>=0&&m_sStateData.Food.GetY()<0)
                                    a=2;
                                  else if(m_sStateData.Food.GetX()<0&&m_sStateData.Food.GetY()>=0)
                                        a=3;
                                        else if(m_sStateData.Food.GetX()<0&&m_sStateData.Food.GetY()<0)
                                                a=4;
			    tData[1] = (UInt8)(x_convert/100);
			    tData[2] = (UInt8)(x_convert%100);
			    tData[3] = a;
			    tData[4] = (UInt8)(y_convert/100);
                            tData[5] = (UInt8)(y_convert%100);
                            tData[6] = m_sStateData.food_color;
                            tData[7] = 2; 
                            
                            tData[9] = 1;
                            //std::cout<<"tData[1] , tData[2] ="<<(Real)tData[4]<<","<<tData[5] << std::endl;
			    m_pcRABA->SetData(tData);
			    /* ... and switch to state 'resting' */}
			    m_pcLEDs->SetAllColors(CColor::RED);
			    m_sStateData.State = SStateData::STATE_RESTING;
			    m_sStateData.TimeSearchingForPlaceInNest = 0;
			    m_eLastExplorationResult = LAST_EXPLORATION_NONE;
                            m_sStateData.EnergyState = SStateData::STATE_RESTING_ROBOT;	
			    return;
			 }
			 else {
			    /* No, keep looking */
			    ++m_sStateData.TimeSearchingForPlaceInNest;
			 }
		   }
		   else {
			 /* Still outside the nest */
			 m_sStateData.TimeSearchingForPlaceInNest = 0;
		   }
		   /* Keep going */
		   bool bCollision;
                   if(bCollision)
                     m_sStateData.EnergyState = SStateData::STATE_AVOIDANCE;
		   SetWheelSpeedsFromVector(
			 m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
			 m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
	   	break;
	   }
    /* This is for coordinators */
	   case 2: {
                   UpdateState();
		   /* Are we in the nest? */
		   if(m_sStateData.InNest) {
			 //m_sFoodData.HasFoodItem = false;
			 /* Have we looked for a place long enough? */
			 if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime) {
			    /* Yes, stop the wheels... */
			    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
			    /* Tell people about the last exploration attempt */
			    
			    m_pcLEDs->SetAllColors(CColor::YELLOW);
			    m_sStateData.State = SStateData::STATE_RESTING;
                            m_sStateData.EnergyState = SStateData::STATE_RESTING_ROBOT;
			    m_sStateData.TimeSearchingForPlaceInNest = 0;
			    m_eLastExplorationResult = LAST_EXPLORATION_NONE;
			  	
			    return;
			 }
			 else {
			    /* No, keep looking */
			    ++m_sStateData.TimeSearchingForPlaceInNest;
			 }
		   }
		   else {
			 /* Still outside the nest */
			 m_sStateData.TimeSearchingForPlaceInNest = 0;
		   }
		   /* Keep going */
		   bool bCollision;
                   if(bCollision)
                     m_sStateData.EnergyState = SStateData::STATE_AVOIDANCE;
		   SetWheelSpeedsFromVector(
			 m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
			 m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
	   	
		break;
	   }
    /* This is for foragers */
	   case 3: {
   /* As soon as you get to the nest, switch to 'resting' */
		   UpdateState();
		   /* Are we in the nest? */
		   if(m_sStateData.InNest) {
			 /* Have we looked for a place long enough? */
                         m_sStateData.EnergyState = SStateData::STATE_HOMING;
			 if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime) {
			    /* Yes, stop the wheels... */ //  w= cos(@/2),z=sin(@/2)

	                 	if (TurningToZero())
		                   {
                                        
			    		m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
				
				    /* Tell people about the last exploration attempt */
				    TRangeAndBearingReceivedPacket::TRangeAndBearingData tData;
				    std::string str= m_sStateData.robot_id.substr(m_sStateData.robot_id.find("_")+1
                                                     ,m_sStateData.robot_id.size()-m_sStateData.robot_id.find("_")); 
                                    const char * c = str.c_str();
                                    UInt8 id = atoi(c);
			            tData[0] = m_eLastExplorationResult;
		                    UInt32 x_convert=(Abs(m_sStateData.Food.GetX())*100);
		                    UInt32 y_convert=(Abs(m_sStateData.Food.GetY())*100);
		                    UInt8 a ;
		                    if(m_sStateData.Food.GetX()>=0&&m_sStateData.Food.GetY()>=0)
		                        a=1;
		                    else if(m_sStateData.Food.GetX()>=0&&m_sStateData.Food.GetY()<0)
		                            a=2;
		                          else if(m_sStateData.Food.GetX()<0&&m_sStateData.Food.GetY()>=0)
		                                a=3;
		                                else if(m_sStateData.Food.GetX()<0&&m_sStateData.Food.GetY()<0)
		                                        a=4;
				    tData[1] = (UInt8)(x_convert/100);
				    tData[2] = (UInt8)(x_convert%100);
				    tData[3] = a;
				    tData[4] = (UInt8)(y_convert/100);
		                    tData[5] = (UInt8)(y_convert%100);
		                    tData[6] = m_sStateData.food_color;
                                    tData[7] = 6;
                                    tData[8] = id;
		                    tData[9] = 3;
				    m_pcRABA->SetData(tData);
				    /* ... and switch to state 'resting' */
				    //Reset();
				    m_pcLEDs->SetAllColors(CColor::RED);
                                    m_sStateData.EnergyState = SStateData::STATE_RESTING_ROBOT;
				    m_sStateData.TimeSearchingForPlaceInNest = 0;
                                    m_sStateData.TimeRested=0;
				    m_eLastExplorationResult = LAST_EXPLORATION_NONE;
                                    m_sStateData.State = SStateData::STATE_RESTING;
				    return;
				}
			 }
			 else {
			    /* No, keep looking */
			   // m_pcWheels->SetLinearVelocity(0.5f, 0.5f);
                       
			    ++m_sStateData.TimeSearchingForPlaceInNest;
                	    bool bCollision;
                            if(bCollision)
                               m_sStateData.EnergyState = SStateData::STATE_AVOIDANCE;
		            SetWheelSpeedsFromVector(
			    m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
			    m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());

			 }
		   }
		   else {
			 /* Still outside the nest */
			 m_sStateData.TimeSearchingForPlaceInNest = 0;
		  	bool bCollision;
                        if(bCollision)
                               m_sStateData.EnergyState = SStateData::STATE_AVOIDANCE;
		   	SetWheelSpeedsFromVector(
			 m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
			 m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());

			// SetWheelSpeedsFromVector(m_sWheelTurningParams.MaxSpeed *CalculateVectorToLight());
		   }
		   /* Keep going */
		   
		break;
	   }// end of case	
    }//end of switch

}
/****************************************/
/****************************************/
void CFootBotForaging::Turning() {
   
   if (m_sStateData.angle != 0){
        /*//CRadians turn_angel=CRadians::PI_OVER_TWO;
                 size_t a;
                 a=1;
                 CDegrees turn_angel=CDegrees(10.0f);
                 CDegrees angel=CDegrees(0.0f);
                 turn_angel=(turn_angel+angel)/2;
                 if(turn_angel>CDegrees(90.0f))
                      a=1;
                 Real cos_r = Cos(ToRadians(turn_angel));
                 Real sin_r = Sin(ToRadians(turn_angel));
                 //CQuaternion n_direction(cos_r,m_sStateData.direction.GetX(),m_sStateData.direction.GetY(),sin_r);
                 //m_sStateData.new_direction=n_direction;
                 //m_sStateData.setorientation=1;
                 if(m_sStateData.direction.GetW()>cos_r-0.03f && m_sStateData.direction.GetZ()>sin_r-0.03f && m_sStateData.direction.GetW()<cos_r+0.01f && m_sStateData.direction.GetZ()<sin_r+0.03f){
                   m_pcWheels->SetLinearVelocity(0.00f, 0.00f);
                   m_sStateData.setorientation=1;
                   std::cout<<"okkkkkkkkGetW() "<<m_sStateData.direction.GetW()<<",getz()"<< m_sStateData.direction.GetZ()<<std::endl;
                                      m_pcWheels->SetLinearVelocity(0.00f, 0.00f);}
                 else if(m_sStateData.setorientation==0)if(a==1) m_pcWheels->SetLinearVelocity(-10.0f, 10.0f);else m_pcWheels->SetLinearVelocity(5.0f, -5.0f);
                      else  {  m_pcWheels->SetLinearVelocity(0.00f, 0.00f);}*/
                 //std::cout<<"GetW() "<<m_sStateData.direction.GetW()<<",getz()"<< m_sStateData.direction.GetZ()<<std::endl;
                 //m_pcWheels->SetLinearVelocity(2.0f, -2.0f);
       //  std::cout<<"m_sStateData.angle"<<m_sStateData.angle<<std::endl;
         if (m_sStateData.angle >= 10){
		 m_pcWheels->SetLinearVelocity(-10.0f, 10.0f);
                 m_sStateData.angle -=10;}
         else{
                 if (m_sStateData.angle > 0){
		 m_pcWheels->SetLinearVelocity(-1.0f, 1.0f);
                 m_sStateData.angle --;}
                 else{
                     if (m_sStateData.angle < -10){
		        m_pcWheels->SetLinearVelocity(10.0f, -10.0f);
                        m_sStateData.angle +=10;}
                     else{
                        m_pcWheels->SetLinearVelocity(1.0f, -1.0f);
	                m_sStateData.angle++;}
                     }
             }
    }
   else{ switch(key){
         case 1:{
              key=2;
               if(spiral)
                    SpiralSearch();
                 else
                    CollisionAvoidance();
              break;
          }
         case 3:{
                 key=4;
                 CollisionAvoidance();
                 break;
          }
         case 5:{
                 key=6;
                 CollisionAvoidance();
                 break;
          }
         case 7:{
                 key=1;
                 m_sStateData.State = SStateData::STATE_EXPLORING;
                 m_sStateData.EnergyState = SStateData::STATE_SCAN_AREA;
                 //std::cout<<"exploring key"<<key <<std::endl;
                 Explore();
                 break;
         }

        }//end of switch
       }//end of else
   if(GoStraight!=0){
                 //std::cout<<"GoStraight"<<GoStraight<<std::endl;
                 m_pcWheels->SetLinearVelocity(10.0f, 10.0f);
                 GoStraight--;
                 if(key==4)
                    m_sStateData.distance --;
         }
   else{ switch(key){
          case 2:{
                 if(spiral){
                   key=1;
                   SpiralSearch();
                 }else{
                   key=3;
                   CollisionAvoidance();}
                 break;
           }
          case 4:{
                 key=5;
                 CollisionAvoidance();
                 break;
          }
          case 6:{
                 key=7;
                 CollisionAvoidance();
                 break;
          }}//end of switch
        }//end of else


}// end of Turning
/****************************************/
/****************************************/
bool CFootBotForaging::TurningToZero()
{

//turning to left z=sin(90/2)=Sin(CRadians::PI_OVER_FOUR)  w=cos(90/2)=Cos(CRadians::PI_OVER_FOUR)
//CRadians turn_angel=CRadians::ZERO;
//Real cos_r = Cos(turn_angel);
//Real sin_r = Sin(turn_angel);
//if(!(m_sStateData.direction.GetW()<=(cos_r + 0.01) && m_sStateData.direction.GetW()>=(cos_r-0.01))||!(m_sStateData.direction.GetZ()<=(sin_r + 0.01) && m_sStateData.direction.GetZ()>= (sin_r-0.01))){
if(!(m_sStateData.direction.GetW()<=1.0 && m_sStateData.direction.GetW()>=(0.98))||!(m_sStateData.direction.GetZ()>=0.0 && m_sStateData.direction.GetZ()< (0.02))){
                                    if(	m_sStateData.direction.GetW()<0.98 || m_sStateData.direction.GetZ()>0.02)
		   	      	    m_pcWheels->SetLinearVelocity(-5.0f, 5.0f);
                                    else
                                    m_pcWheels->SetLinearVelocity(-1.0f, 1.0f);
				return false;	
			           
		               }else{return true;}
}
/****************************************/
/****************************************/
void CFootBotForaging::CollisionAvoidance()
{
 switch(key){
 case 1:{
     // std::cout<<"collision avoidance key="<<key <<std::endl;
      TurningRight();
      break;
 }
 case 2:{
       Go_Straight(40);
      // std::cout<<"collision avoidance key="<<key <<std::endl;
       break;
 } 
 case 3:{
      // std::cout<<"collision avoidance key="<<key <<std::endl;
       TurningLeft();
       break;
 }
 case 4:{
       Go_Straight(50);
       
      // std::cout<<"collision avoidance key="<<key <<std::endl;
       break;
 }
 case 5:{
      // std::cout<<"collision avoidance key="<<key <<std::endl;
       TurningLeft();
       break;
        }
 case 6:{
       Go_Straight(20);
      // std::cout<<"collision avoidance key="<<key <<std::endl;
       break;
 }
 case 7:{
      // std::cout<<"collision avoidance key="<<key <<std::endl;
      TurningRight();
      break;
 }

 default: {
        std::cout<<"collision avoidance key="<<key <<std::endl;
        LOGERR << "We can't be here, there's a bug in collision avoidance switch!" << std::endl;
      }
   }

}
/****************************************/
/****************************************/
bool CFootBotForaging::TurningLeft()
{
  m_sStateData.angle = 90 ;
  //std::cout<<"turning left"<<std::endl;
 //m_sStateData.rev_angle = 135 + (135/4);
  m_sStateData.heading_angle += 90 ;
  Turning();
//m_sStateData.State = SStateData::STATE_TURNING;
return true;

/*
//turning to left z=sin(90/2)=Sin(CRadians::PI_OVER_FOUR)  w=cos(90/2)=Cos(CRadians::PI_OVER_FOUR)
CRadians turn_angel=CRadians::PI_OVER_FOUR;
Real cos_r = Cos(turn_angel);
Real sin_r = Sin(turn_angel);
if(!(m_sStateData.direction.GetW()<=0.71 && m_sStateData.direction.GetW()>=(0.68))||!(m_sStateData.direction.GetZ()<=0.71 && m_sStateData.direction.GetZ()>= (0.68))){
				         std::cout<<cos_r<<","<<sin_r<<std::endl;	
                                         //std::cout<<m_sStateData.direction.GetW()<<","<<m_sStateData.direction.GetZ()<<std::endl;
		   	      	         m_pcWheels->SetLinearVelocity(-1.0f, 1.0f);
				        return true;	
			                }else{return false;}*/
}
/****************************************/
/****************************************/
bool CFootBotForaging::TurningRight()
{
 //std::cout<<"turning right"<<std::endl;
 m_sStateData.angle = -90;
 //m_sStateData.rev_angle = 135 + (135/4);
  m_sStateData.heading_angle -= 90 ;
  Turning();
//m_sStateData.State = SStateData::STATE_TURNING;
return true;
/*if(!(m_sStateData.direction.GetW()<=0.71 && m_sStateData.direction.GetW()>=(0.68))||!(m_sStateData.direction.GetZ()<=-0.68 && m_sStateData.direction.GetZ()>= (-0.71))){				 
                                          std::cout<<m_sStateData.direction.GetW()<<","<<m_sStateData.direction.GetZ()<<std::endl;
		   	      	          m_pcWheels->SetLinearVelocity(-1.0f, 1.0f);
				          return true;	
			                 }else{return false;}*/
}
/****************************************/
/****************************************/
 void CFootBotForaging::Go_Straight(Real distance)
{
 GoStraight = distance;
 m_pcWheels->SetLinearVelocity(10.0f, 10.0f);
 Turning();
}


/****************************************/
/****************************************/
bool CFootBotForaging::SpiralSearch()
{ 
      if(key==1){
         m_sStateData.State = SStateData::STATE_TURNING;
         TurningRight();}
      if(key==2){
         Go_Straight(3+spiral_inc_step);
       }
      spiral_key++; 
      if(spiral_key==2){
         spiral_inc_step+=5;
         spiral_key=0;
       }
     spiral_state++;
    // std::cout<<"spiral_state"<<spiral_state<<std::endl;
    // m_sStateData.State = SStateData::STATE_EXPLORING;
 /* m_pcWheels->SetLinearVelocity(i, 10.0f);
  j = ((UInt8)spiral_state)%50;
  if(j==0 && spiral_state <= 400 )
      i++;
  spiral_state++;*/
  if(spiral_state==60||m_sFoodData.HasFoodItem == true ){
       spiral_inc_step=0;
       key=1;
       spiral_state=0;
       spiral=false;
       if(m_sFoodData.HasFoodItem == true){
          m_sStateData.EnergyState = SStateData::STATE_MOVE_TO_HOME;
          m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
       }else{
          m_sStateData.EnergyState = SStateData::STATE_HOMING;
          m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
          }
       m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
   }   
  return true;
}

/****************************************/
/****************************************/
void CFootBotForaging::FindFood(){
   delta_x = m_sStateData.Food.GetX() - m_sStateData.RoboPos.GetX();
   delta_y = m_sStateData.Food.GetY() - m_sStateData.RoboPos.GetY();
   //std::cout<<" find,Food x , Food y ="<<m_sStateData.Food.GetX()<<","<<m_sStateData.Food.GetY() << std::endl;
   //std::cout<<"delta x , delta y ="<<delta_x<<","<<delta_y << std::endl;
   m_sStateData.rev_angle = (Real)(atan2(delta_y,delta_x));
   m_sStateData.rev_angle = Floor(m_sStateData.rev_angle *(180/ARGOS_PI)); 
   // std::cout<<"angle"<<m_sStateData.rev_angle<< std::endl;
   //m_sStateData.heading_angle= m_sStateData.rev_angle;
   
    m_sStateData.rev_angle = Floor(m_sStateData.rev_angle);// + (m_sStateData.rev_angle/8)
    m_sStateData.angle =  m_sStateData.rev_angle - m_sStateData.heading_angle;
    m_sStateData.angle =Floor(m_sStateData.angle*1.2);
    if(m_sStateData.angle<0)
       m_sStateData.angle+=360;
    m_sStateData.heading_angle =m_sStateData.rev_angle;
    if(m_sStateData.angle>180){
      m_sStateData.angle -=360;
   }
   //std::cout<<"angle ="<<m_sStateData.angle<< std::endl;
   m_sStateData.distance =Floor( sqrt((delta_x*delta_x)+(delta_y *delta_y ))* 105 ); //4*4=90 8*8=100
   //std::cout<<"distance ="<< m_sStateData.distance<< std::endl;
}
/****************************************/
/****************************************/
UInt32 CFootBotForaging::Area(Real f_x ,Real f_y ){
if( f_x > 1.3 && f_y > 1.3)
   return 0;
if( f_x > -1.3 && f_x <1.3  && f_y > 1.3)
   return 1;
if( f_x < -1.3 && f_y > 1.3)
   return 2;
if( f_x < -1.3 && f_y < 1.3 && f_y > -1.3)
   return 3;
if( f_x < -1.3 && f_y < -1.3)
   return 4;
if( f_x < 1.3 && f_x > -1.3 && f_y < -1.3)
   return 5;
if( f_x > 1.3 && f_y < -1.3)
   return 6;
if( f_x > 1.3 && f_y < 1.3 && f_y > -1.3)
   return 7;
else return 8;

}
/****************************************/
/****************************************/
void CFootBotForaging::EstimateQuantity(){
 //feed forward
 for(size_t i = 0; i < m_unNumberOfHiddens; ++i) {
      // Add the bias (weighted by the first weight to the i'th output node)
      m_pfHiddens[i] = m_pfWeightsIH[i * (m_unNumberOfInputs + 1)];

      for(size_t j = 0; j < m_unNumberOfInputs; ++j) {
         // Compute the weight number
         size_t ji = i * (m_unNumberOfInputs + 1) + (j + 1);
         // Add the weighted input
         m_pfHiddens[i] += m_pfWeightsIH[ji] * m_pfInputs[j];

      }

      // Apply the transfer function (sigmoid with output in [0,1])
      m_pfHiddens[i] = 1.0f / ( 1.0f + ::exp( -m_pfHiddens[i]) );
   }
   for(size_t i = 0; i < m_unNumberOfOutputs; ++i) {
      // Add the bias (weighted by the first weight to the i'th output node)
      m_pfOutputs[i] = m_pfWeightsHO[i * (m_unNumberOfHiddens + 1)];

      for(size_t j = 0; j < m_unNumberOfHiddens; ++j) {
         // Compute the weight number
         size_t ji = i * (m_unNumberOfHiddens + 1) + (j + 1);
         // Add the weighted input
         m_pfOutputs[i] += m_pfWeightsHO[ji] * m_pfHiddens[j];

      }//end of for j

      // Apply the transfer function (sigmoid with output in [0,1])
      m_pfOutputs[i] = 1.0f / ( 1.0f + ::exp( -m_pfOutputs[i]) );
     // std::cout<<"m_pfOutputs "<<i<<"="<< m_pfOutputs[i]<< std::endl;
   }//end of for i
 }
/****************************************/
/****************************************/
void CFootBotForaging::BackPropagateError(Real output,Real target,UInt32 index){
   for(size_t i = 0; i < m_unNumberOfOutputs; ++i) 
       DeltaO[i]=0;
   DeltaO[index] = (target-output)*output*(1-output);
   
   Real sumDOW=0;
   for(size_t j = 0; j< m_unNumberOfOutputs; j++) {
         sumDOW = m_pfWeightsHO[j * (m_unNumberOfHiddens + 1)];
         for(size_t i = 0; i< m_unNumberOfHiddens; i++) {
             size_t ji = j * (m_unNumberOfHiddens + 1) + (i + 1);
   	     sumDOW +=m_pfWeightsHO[ji]*DeltaO[i];}
  	DeltaH[j]=sumDOW * m_pfHiddens[j]*(1-m_pfHiddens[j]);
        sumDOW=0;
   }
//update weights weightsIH
   for(size_t j = 0; j < m_unNumberOfHiddens; j++) {
   	//delta_weights[j * (m_unNumberOfInputs + 1)] = eta * DeltaH[j]+ alpha * delta_weights[j * (m_unNumberOfInputs + 1)];
   	//m_pfWeightsIH[j * (m_unNumberOfInputs + 1)] += delta_weights[j * (m_unNumberOfInputs + 1)];
        m_pfWeightsIH[j * (m_unNumberOfInputs + 1)] += eta * DeltaH[j];
   	for(size_t k = 0; k < m_unNumberOfInputs; k++) {
       	    size_t kj = j * (m_unNumberOfInputs + 1) + (k + 1);
            //delta_weights[kj] = eta * m_pfInputs[k] * DeltaH[j]+ alpha * delta_weights[kj];
            //m_pfWeightsIH[kj] +=  delta_weights[kj];
            m_pfWeightsIH[kj] += eta * m_pfInputs[k] * DeltaH[j];
        }
   }
//update weights weightsHO
   for(size_t j = 0; j < m_unNumberOfOutputs; j++) {
   	//delta_weights[j * (m_unNumberOfInputs + 1)] = eta * DeltaH[j]+ alpha * delta_weights[j * (m_unNumberOfInputs + 1)];
   	//m_pfWeightsIH[j * (m_unNumberOfInputs + 1)] += delta_weights[j * (m_unNumberOfInputs + 1)];
        m_pfWeightsHO[j * (m_unNumberOfHiddens + 1)] += eta * DeltaO[j];
   	for(size_t k = 0; k <  m_unNumberOfHiddens; k++) {
       	    size_t kj = j * (m_unNumberOfHiddens + 1) + (k + 1);
            //delta_weights[kj] = eta * m_pfInputs[k] * DeltaH[j]+ alpha * delta_weights[kj];
            //m_pfWeightsIH[kj] +=  delta_weights[kj];
            m_pfWeightsHO[kj] += eta * m_pfHiddens[k] * DeltaO[j];
        }
   }
      // std::cout<<"back propagate exactly "<< std::endl;
/*
    //delta_weights[m_unNumberOfWeightsIH] = eta * DeltaO+ alpha * delta_weights[m_unNumberOfWeightsIH];
    //m_pfWeightsHO[0] += delta_weights[m_unNumberOfWeightsIH];
    m_pfWeightsHO[0] += eta*DeltaO;
    for(size_t j = 0; j <= m_unNumberOfHiddens; j++){
       //delta_weights[m_unNumberOfWeightsIH + j + 1] = eta * m_pfHiddens[j+1] * DeltaO + alpha * delta_weights[m_unNumberOfWeightsIH + j + 1];
        //m_pfWeightsHO[j+1] += delta_weights[m_unNumberOfWeightsIH + j + 1];
        m_pfWeightsHO[j+1] += eta * m_pfHiddens[j] * DeltaO;
    }*/

}
/****************************************/
/****************************************/
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotForaging, "footbot_foraging_controller")
