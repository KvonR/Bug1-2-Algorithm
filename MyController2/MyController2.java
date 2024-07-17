/**
  * BUG2 algorithm implementation - Kevin Rahimi 201563237 01/2024
  **/

import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.PositionSensor;
import java.lang.Math; 
 


public class MyController2 {
  /**
   * Enumeration for the MoveStates of the robot
   * MOVE_TO_TARGET: Robot is actively moving towards the designated target
   * WALL_FOLLOW: Robot is following along a wall, typically used for obstacle avoidance
   */
  public static enum GlobalState {
    MOVE_TO_TARGET,
    WALL_FOLLOW};

//Main Loop:
  public static void main(String[] args) {
// Create a Supervisor to access the node to get orientation and location of robot
    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());// Calculate the world timestep

    // Define the start and target coordinates for the robot
    double[] start = {-4.5,3,0};
    double[] target = {2.75 ,-3.26 };
    double targetThreshold = 0.4;//eliminates at the cone with the tolerance of 0.4

// Initialize the controller movement state to MOVE TO TARGET and navigation state to ARC (turning)
    PioneerNav2.MoveState state; 
    String displayMoveState = "Robot initilised - heading towards target";
    GlobalState global_state = GlobalState.MOVE_TO_TARGET;///Setting starting controller state to move to target
    state = PioneerNav2.MoveState.ARC;///Setting starting navigation state to arc
    double  start_time = robot.getTime();//getting start time
    
    // Initialize the robot's pose based on the starting coordinates
    Pose robot_pose = new Pose(start[0], start[1], start[2]);
    //PioneerNav2 nav = new PioneerNav2(robot, robot_pose);
    //PioneerProxSensors1 prox_sensors = new PioneerProxSensors1(robot, "sensor_display", robot_pose);
    
    // Initialize and enable the position sensors for the robot's wheels initailzing position sensors
    PositionSensor left_position_sensor = robot.getPositionSensor("left wheel sensor");
    left_position_sensor.enable(timeStep);
    PositionSensor right_position_sensor = robot.getPositionSensor("right wheel sensor");
    right_position_sensor.enable(timeStep);
    
    // Initialize the proximity sensors and navigation system for the robot
    PioneerProxSensors1 prox_sensors = new PioneerProxSensors1(robot, "sensor_display", robot_pose);
    PioneerNav2 nav = new PioneerNav2(robot, robot_pose, prox_sensors);
 
    // Initialize variables for navigation control
    double time_elapsed = 0;
    double target_time = 0;
    double time_elapsed_2 = 0;
    double target_time_2 = 0;
    double robot_velocity = 0.3;
    
    // Calculate the start angle the robot should follow to reach the target directly
    double s_y = target[1] - robot_pose.getY();
    double s_x = target[0] - robot_pose.getX();
    double start_angle= Math.atan2(s_y, s_x) - robot_pose.getTheta();//Getting the start angle of the robot should follow to reach
    
    // Calculate the initial minimum distance to the target based on the start and target coordinates
    double distance_to_travel =  Math.sqrt(Math.pow((target[0] - start[0]), 2) + Math.pow(target[1] - start[1], 2)); ///Variable to save minimum distance
    double theta = 0;//Variable for calculating the robots orientation, iF theta is almost 0 then the robot is traveling towards the target
   
// Main control loop of the robot:
    while (robot.step(timeStep) != -1) {
      // Update the robot's pose and sensor display    
      robot_pose = nav.get_real_pose();
      prox_sensors.set_pose(robot_pose);
      prox_sensors.paint();  // Render sensor Display
      prox_sensors.paint2(displayMoveState,robot.getTime() - start_time,nav.getLeftVelocity(),nav.getRightVelocity(),(0.0957*(right_position_sensor.getValue() + left_position_sensor.getValue())/2)-48.44);
      
      // Check if the robot is close enough to the target to stop      
      if (Math.sqrt(Math.pow((target[0] - robot_pose.getX()), 2) + Math.pow(target[1] - robot_pose.getY(), 2)) < targetThreshold)
      { 
          
           state = PioneerNav2.MoveState.STOP; // Setting navigation state to stop#
           displayMoveState = "Goal has been reached - Terminating Robot";
           break;
             }
             
      // Determine the robot's behavior based on the current movement state      
      if (global_state == GlobalState.MOVE_TO_TARGET)//state where the robot turns and moves towards the goal
      {
        double y = target[1] - robot_pose.getY();
        double x = target[0] - robot_pose.getX();
        double tmp_angle = Math.atan2(y, x);//Getting the angle towards the target
        
        theta = tmp_angle - robot_pose.getTheta();
        if (Math.abs(theta)>0.02)
        {
          state = PioneerNav2.MoveState.ARC ;
        }
        else//Else moving towards the target
        {
          state = PioneerNav2.MoveState.FORWARD ;
          if (prox_sensors.get_value(4)  <0.2)//IF there is an obstacle ahead getting ready to do wallfollowing
          {
            System.out.println("Object Detected following wall!");
            System.out.println("Current Location: " + robot_pose);
            displayMoveState = "Object Detected Following Wall";
            time_elapsed = target_time+1; //Removing form the MoveState.FORWARD by completeing its time
            global_state = GlobalState.WALL_FOLLOW;//Changing global state to wall following
            
            target_time_2 = 3200; //Setting target time soo that the robot clears the starting area
            time_elapsed_2 = 0;
          }
        }
      } 
       if (global_state == GlobalState.WALL_FOLLOW)
      {
        state = PioneerNav2.MoveState.FOLLOW_WALL;//Setting navigation state to follow wall 
         if (time_elapsed_2 > target_time_2)  ///If time elapsed then starting to look for the start_angle
         { 
         ///Calclating robots current angle with target
         double  a_y = target[1] - robot_pose.getY();
         double a_x = target[0] - robot_pose.getX();
         double alpha  = Math.atan2(a_y, a_x) ;//Getting the current angle with the target   
         if ( Math.abs(start_angle-alpha)<0.01)
         {
            System.out.println("Path Clear Heading towards Goal!!!");
            System.out.println("Current Location: " + robot_pose);
            displayMoveState = "Path Clear Heading towards Goal!!!";


            global_state = GlobalState.MOVE_TO_TARGET;///Switching the state to move to target since the robot is ready to start doing wall following again
         }
        }
        else
        {
           time_elapsed_2 += timeStep;  
        }
      }
      
      if (time_elapsed > target_time) {
        time_elapsed = 0;
            
       
        if (state == PioneerNav2.MoveState.FOLLOW_WALL)
        {
        target_time = 0;
        nav.follow_wall(robot_velocity, 0.25, true);
        }
        else
        if (state == PioneerNav2.MoveState.FORWARD) {
          target_time = nav.forward(distance_to_travel, robot_velocity);
        } 
        else 
        if (state == PioneerNav2.MoveState.ARC) {
          target_time = nav.arc(theta, 0.0, robot_velocity);
        } 
        else
        if (state == PioneerNav2.MoveState.STOP) {
          nav.stop();
          target_time = 60 * 1000; // This doesn't really stop, but pauses for 1 minute
          
        }
        
      } else
        time_elapsed += timeStep;    // Increment by the time state
      
    };
    // Enter here exit cleanup code.
    System.out.println("Goal has been reached:" + robot_pose + " with a threshold of: "+ targetThreshold);
    nav.stop();
    System.out.print("Time taken :");
    System.out.print(robot.getTime() - start_time);
    System.out.println(" seconds");
    System.out.print("Distance traveled :");
    System.out.print((0.0957*(right_position_sensor.getValue() + left_position_sensor.getValue())/2)-48.44);
    System.out.println(" meters");
    prox_sensors.paint2(displayMoveState,robot.getTime() - start_time,
    nav.getLeftVelocity(),
    nav.getRightVelocity(),
    (0.0957*(right_position_sensor.getValue() + left_position_sensor.getValue())/2)-48.44);

    robot.step(timeStep);
  }

      //Pose true_pose = nav.get_real_pose();
      //System.out.println("Action: " + display_action + " \t" + "True Pose: "+true_pose);}
  
}
