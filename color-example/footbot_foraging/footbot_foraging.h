
#ifndef FOOTBOT_FORAGING_H
#define FOOTBOT_FORAGING_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos2/common/control_interface/ci_controller.h>
/* Definition of the foot-bot wheel actuator */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
/* Definition of the foot-bot LEDs actuator */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_light_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_motor_ground_sensor.h>
/* Definitions for random number generation */
#include <argos2/common/utility/argos_random.h>
#include <argos2/common/utility/math/quaternion.h>
#include <cmath>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotForaging : public CCI_Controller {

public:

   /*
    * This structure holds data about food collecting by the robots
    */
   struct SFoodData {
      bool HasFoodItem;      // true when the robot is carrying a food item
      size_t FoodItemIdx;    // the index of the current food item in the array of available food items
      size_t TotalFoodItems; // the total number of food items carried by this robot during the experiment
      CVector2 FoodPos; 	 /**********************************my code**********************************/
      SFoodData();
      void Reset();
   };

   /*
    * The following variables are used as parameters for the
    * diffusion algorithm. You can set their value in the <parameters>
    * section of the XML configuration file, under the
    * <controllers><footbot_foraging_controller><parameters><diffusion>
    * section.
    */
   struct SDiffusionParams {
      /*
       * Maximum tolerance for the proximity reading between
       * the robot and the closest obstacle.
       * The proximity reading is 0 when nothing is detected
       * and grows exponentially to 1 when the obstacle is
       * touching the robot.
       */
      Real Delta;
      /* Angle tolerance range to go straight. */
      CRange<CRadians> GoStraightAngleRange;

      /* Constructor */
      SDiffusionParams();

      /* Parses the XML section for diffusion */
      void Init(TConfigurationNode& t_tree);
   };

   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_foraging_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /*
    * Contains all the state information about the controller.
    */
   struct SStateData {
      /* The three possible states in which the controller can be */
      enum EState {
         STATE_RESTING = 0,
         STATE_EXPLORING,
         STATE_RETURN_TO_NEST,
         STATE_TURNING
      } State;
      enum EnState {
         STATE_RESTING_ROBOT = 0,
         STATE_LEAVING_HOME,
         STATE_RANDOM_WALK,
         STATE_MOVE_TO_FOOD,
         STATE_GRAB_FOOD,
         STATE_SCAN_AREA,
         STATE_MOVE_TO_HOME,
         STATE_DEPOSIT_FOOD,
         STATE_HOMING,
         STATE_AVOIDANCE
      } EnergyState;
 	 /**********************************my code**********************************/
      /* True when the robot is in the nest */
      bool InNest;
	 /* True when the robot is on a food */
	 bool OnFood;
	 /* True when the robot is on the Ground */
	 bool OnGround;
 	 /**********************************my code**********************************/
      Real NumberOfForagers;
      /* Initial probability to switch from resting to exploring */
      Real InitialRestToExploreProb;
      /* Current probability to switch from resting to exploring */
      Real RestToExploreProb;
      /* Initial probability to switch from exploring to resting */
      Real InitialExploreToRestProb;
      /* Current probability to switch from exploring to resting */
      Real ExploreToRestProb;
      /* Used as a range for uniform number generation */
      CRange<Real> ProbRange;
      /* The increase of ExploreToRestProb due to the food rule */
      Real FoodRuleExploreToRestDeltaProb;
      /* The increase of RestToExploreProb due to the food rule */
      Real FoodRuleRestToExploreDeltaProb;
      /* The increase of ExploreToRestProb due to the collision rule */
      Real CollisionRuleExploreToRestDeltaProb;
      /* The increase of RestToExploreProb due to the social rule */
      Real SocialRuleRestToExploreDeltaProb;
      /* The increase of ExploreToRestProb due to the social rule */
      Real SocialRuleExploreToRestDeltaProb;
      /* The minimum number of steps in resting state before the robots
         starts thinking that it's time to move */
      size_t MinimumRestingTime;
      /* The number of steps in resting state */
      size_t TimeRested;
      /* The number of exploration steps without finding food after which
         a foot-bot starts thinking about going back to the nest */
      size_t MinimumUnsuccessfulExploreTime;
      /* The number of exploration steps without finding food */
      size_t TimeExploringUnsuccessfully;
      /* If the robots switched to resting as soon as it enters the nest,
         there would be overcrowding of robots in the border between the
         nest and the rest of the arena. To overcome this issue, the robot
         spends some time looking for a place in the nest before finally
         settling. The following variable contains the minimum time the robot
         must spend in state 'return to nest' looking for a place in the nest
         before switching to the resting state. */
      size_t MinimumSearchForPlaceInNestTime;
      /* The time spent searching for a place in the nest */
      size_t TimeSearchingForPlaceInNest;
 	 /**********************************my code**********************************/
	 CVector2 RoboPos;
	 CVector2 Food;
         Real food_color;
         Real employers;
	 Real angle;
	 Real rev_angle;
         Real heading_angle;
         Real distance;
         std::string robot_id;
	 CQuaternion direction;
         CQuaternion new_direction;
         size_t setorientation;
 	 size_t RobotType;
	 /**********************************my code**********************************/
      SStateData();
      void Init(TConfigurationNode& t_node);
      void Reset();
   };

public:

   /* Class constructor. */
   CFootBotForaging();
   /* Class destructor. */
   virtual ~CFootBotForaging() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_foraging_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy();

   /*
    * Returns true if the robot is currently exploring.
    */
   inline bool IsExploring() const {
      return m_sStateData.State == SStateData::STATE_EXPLORING;
   }

   /*
    * Returns true if the robot is currently resting.
    */
   inline bool IsResting() const {
      return m_sStateData.State == SStateData::STATE_RESTING;
   }

   /*
    * Returns true if the robot is currently returning to the nest.
    */
   inline bool IsReturningToNest() const {
      return m_sStateData.State == SStateData::STATE_RETURN_TO_NEST;
   }
   /*
    * Returns true if the robot is currently turning.
    */
   inline bool IsTurning() const {
      return m_sStateData.State == SStateData::STATE_TURNING;
   }
    /*****************************energy states*************************/
   /*
    * Returns true if the robot is currently leaving home.
    */
   inline bool IsLeavingHome() const {
      return m_sStateData.EnergyState == SStateData::STATE_LEAVING_HOME;
   }
   /*
    * Returns true if the robot is currently random walking in area.
    */
   inline bool IsRandomWalking() const {
      return m_sStateData.EnergyState == SStateData::STATE_RANDOM_WALK;
   }
   
   /*
    * Returns true if the robot is currently grabing food.
    */
   inline bool IsGrabingFood() const {
      return m_sStateData.EnergyState == SStateData::STATE_GRAB_FOOD;
   }
   /*
    * Returns true if the robot is currently scaning area.
    */
   inline bool IsScanningArea() const {
      return m_sStateData.EnergyState == SStateData::STATE_SCAN_AREA;
   }
   /*
    * Returns true if the robot is currently grabing food.
    */
   inline bool IsMovingHome() const {
      return m_sStateData.EnergyState == SStateData::STATE_MOVE_TO_HOME;
   }
    /*
    * Returns true if the robot is currently grabing food.
    */
   inline bool IsMovingToFood() const {
      return m_sStateData.EnergyState == SStateData::STATE_MOVE_TO_FOOD;
   }
   /*
    * Returns true if the robot is currently grabing food.
    */
   inline bool IsDepositingFood() const {
      return m_sStateData.EnergyState == SStateData::STATE_DEPOSIT_FOOD;
   }
   /*
    * Returns true if the robot is currently grabing food.
    */
   inline bool IsAvoidancing() const {
      return m_sStateData.EnergyState == SStateData::STATE_AVOIDANCE;
   }
   /*
    * Returns true if the robot is currently grabing food.
    */
   inline bool IsHoming() const {
      return m_sStateData.EnergyState == SStateData::STATE_HOMING;
   }
   /*
    * Returns true if the robot is currently grabing food.
    */
   inline bool IsRestingRobot() const {
      return m_sStateData.EnergyState == SStateData::STATE_RESTING_ROBOT;
   }
         
   /*
    * Returns the food data
    */
   inline SFoodData& GetFoodData() {
      return m_sFoodData;
   }

   /*
    * Returns the state data by my code
    */	
   inline SStateData& GetStateData() {
      return m_sStateData;
   }
  /* inline Real SStateData& NumForagers(){
      return m_sStateData.NumberOfForagers;
   }*/
private:

   /*
    * Updates the state information.
    * In pratice, it sets the SStateData::InNest flag.
    * Future, more complex implementations should add their
    * state update code here.
    */
   void UpdateState();

   /*
    * Calculates the vector to the light. Used to perform
    * phototaxis and antiphototaxis.
    */
   CVector2 CalculateVectorToLight();

   /*
    * Calculates the diffusion vector. If there is a close obstacle,
    * it points away from it; it there is none, it points forwards.
    * The b_collision parameter is used to return true or false whether
    * a collision avoidance just happened or not. It is necessary for the
    * collision rule.
    */
   CVector2 DiffusionVector(bool& b_collision);
   CVector2 DiffusionVector2(bool& b_collision);
   /* 
    *Executes the collision avoidance function
    */
    void CollisionAvoidance();

   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   /*
    * Executes the resting state.
    */
   void Rest();

   /*
    * Executes the exploring state.
    */
   void Explore();

   /*
    * Executes the return to nest state.
    */
   void ReturnToNest();

   /*
    * Executes the turning state.
    */
   void Turning();

   /*
    * turn robot to left.
    */
   bool TurningLeft();

   /*
    * turn robot to right.
    */
   bool TurningRight();

   /*
    * set robot orientation to zero.
    */
   bool TurningToZero();

   /*
    * set robot going straight.
    */
   void Go_Straight(Real distance);

   /*
    * robot going to spiral search.
    */
   bool SpiralSearch();

   /*
    * robot go out of home for find food.
    */
   void FindFood();

//neural network function
   Real EstimateQuantity();
   void BackPropagateError(Real& output,Real& target);

private:

   /* Pointer to the foot-bot wheels actuator */
   CCI_FootBotWheelsActuator*  m_pcWheels;
   /* Pointer to the foot-bot LEDs actuator */
   CCI_FootBotLedsActuator*  m_pcLEDs;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the foot-bot motor ground sensor */
   CCI_FootBotMotorGroundSensor* m_pcGround;

   /* The random number generator */
   CARGoSRandom::CRNG* m_pcRNG;

   /* Used in the social rule to communicate the result of the last exploration attempt */
   enum ELastExplorationResult {
      LAST_EXPLORATION_NONE = 0,    // nothing to report
      LAST_EXPLORATION_SUCCESSFUL,  // the last exploration resulted in a food item found
      LAST_EXPLORATION_UNSUCCESSFUL // no food found in the last exploration
   } m_eLastExplorationResult;

   /* The controller state information */
   SStateData m_sStateData;
   /* The turning parameters */
   SWheelTurningParams m_sWheelTurningParams;
   /* The diffusion parameters */
   SDiffusionParams m_sDiffusionParams;
   /* The food data */
   SFoodData m_sFoodData;
   size_t key;
   Real delta_x;
   Real delta_y; 
   Real GoStraight;
   Real spiral_state;
   Real spiral_inc_step;
   Real spiral_key;
   bool spiral;
   
   std::vector<CVector2> send_id;
   size_t robots_send ;

////////////////// neural network /////////////////

   UInt32 m_unNumberOfInputs;
   UInt32 m_unNumberOfHiddens;
   UInt32 m_unNumberOfWeightsIH;
   UInt32 m_unNumberOfWeightsHO;

   Real* m_pfInputs;
   Real* m_pfHiddens;
   Real* m_pfWeightsIH;
   Real* m_pfWeightsHO;
   Real* delta_weights;
   Real* DeltaH;
   Real m_pfOutput;

};

#endif
