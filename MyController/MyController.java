/**
  * BUG1 algorithm implementation - Kevin Rahimi 201563237 01/2024
  **/


import com.cyberbotics.webots.controller.Supervisor;



import com.cyberbotics.webots.controller.PositionSensor;////Getting postion sensors to track the distance robot traveled. This is done using the multiplying the value given by the position sensor with the wheel radius
import java.lang.Math; 


public class MyController {
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
    double targetThreshold = 0.4;// reduce to 0 to reach the goal exactly
  
    // Initialize the controller movement state to MOVE TO TARGET and navigation state to ARC (turning)
    PioneerNav2.MoveState state;
    String displayMoveState = "Robot initilised - heading towards target";
    GlobalState global_state = GlobalState.MOVE_TO_TARGET;
    state = PioneerNav2.MoveState.ARC; 
    double  start_time = robot.getTime();
    
    Pose robot_pose = new Pose(start[0], start[1], start[2]);///Starting coordinates
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
    double robot_velocity = 0.3;

    
    // Calculate the start angle the robot should follow to reach the target directly
    double min_distance = Math.sqrt(Math.pow((target[0] - start[0]), 2) + Math.pow(target[1] - start[1], 2)); ///Minimum distance to target
    double distance_to_travel = min_distance;//Distance variable given for the move foward state, if there are no obstacles the robot will move directly towards the target
    double[] wall_follow_start_cord = {0,0}; //Location coordinates of wall follow starting
    int counter = 0;/// Counter to count the times the robot was in the wall following start area
    boolean update_min = true; //Updates minimum distance when true
    double theta = 0;//Variable for calculating the robots orientation, iF theta is almost 0 then the robot is traveling towards the target
    double cord_at_min_distance[] = {0,0};///Coordinates at the minimum distance to target when wall following
    
// Main control loop of the robot:
    while (robot.step(timeStep) != -1) {
      // Update the robot's pose and sensor display         
      robot_pose = nav.get_real_pose();
      prox_sensors.set_pose(robot_pose);
      prox_sensors.paint();  // Render sensor Display
      prox_sensors.paint2(displayMoveState,robot.getTime() - start_time,nav.getLeftVelocity(),nav.getRightVelocity(),(0.0957*(right_position_sensor.getValue() + left_position_sensor.getValue())/2)-48.44);

      
      // Check if the robot is close enough to the target to stop            
      if (Math.sqrt(Math.pow((target[0] - robot_pose.getX()), 2) + Math.pow(target[1] - robot_pose.getY(), 2)) < targetThreshold)
      { //When close to the target then terminate the while loop
           
           state = PioneerNav2.MoveState.STOP; // Setting navigation state to stop
           displayMoveState = "Goal has been reached - Terminating Robot";
           break;
             }
      
      if (global_state == GlobalState.MOVE_TO_TARGET)//state where the robot turns and moves towards the goal
      {
        
        double y = target[1] - robot_pose.getY();
        double x = target[0] - robot_pose.getX();
        double alpha = Math.atan2(y, x);//Getting the angle towards the target
        
        theta = alpha - robot_pose.getTheta();//Getting the orientation difference with the target
        
        if (Math.abs(theta)>0.02)
        {
          state = PioneerNav2.MoveState.ARC ;
        }
        else
        {
          state = PioneerNav2.MoveState.FORWARD ;
          if (prox_sensors.get_value(4)  <0.2)//IF there is an obstacle ahead getting ready to do wall follow
          {
            time_elapsed = target_time+1; //Increment time
            global_state = GlobalState.WALL_FOLLOW;//Changing controller state to wall following
            
            //Setting wall follow start cords
            wall_follow_start_cord[0] = robot_pose.getX();
            wall_follow_start_cord[1] = robot_pose.getY();
            
            update_min = true; //Update the minimum distance to target
            counter = 0 ;//reset on new obstacle
          }
        }
      } 
       if (global_state == GlobalState.WALL_FOLLOW)///Controller state wall follow 
      {
        state = PioneerNav2.MoveState.FOLLOW_WALL;//Setting navigation state to follow wall 
         double distance = Math.sqrt(Math.pow((target[0] - robot_pose.getX()), 2) + Math.pow(target[1] - robot_pose.getY(), 2));//getting distance to target
         counter +=1 ; 
         if ((Math.sqrt(Math.pow((wall_follow_start_cord[0] - robot_pose.getX()), 2) + Math.pow(wall_follow_start_cord[1] - robot_pose.getY(), 2)) <0.25) && (counter >100) )
         {
               ///IF the counter variable if greater than 100 and the robot is in the wall follow start area then robot has completed a round 
               update_min = false; ///Stop updating the minimum distance to target
        }
        else //else the robot is still near the start area so reset counter
        if (Math.sqrt(Math.pow((wall_follow_start_cord[0] - robot_pose.getX()), 2) + Math.pow(wall_follow_start_cord[1] - robot_pose.getY(), 2)) <0.25 )
         {
                counter = 0; //making the update variabke 0 
        }
        
         if ((min_distance> distance) && update_min )///updating the min dist
         {
               min_distance  = distance ; 
               cord_at_min_distance[0] = robot_pose.getX();
               cord_at_min_distance[1] = robot_pose.getY();
               }
          ///if the robot is not updating the minimum  anymore and meets the minimum then change state to move to target    
          if ((Math.sqrt(Math.pow((cord_at_min_distance[0] - robot_pose.getX()), 2) + Math.pow(cord_at_min_distance[1] - robot_pose.getY(), 2)) <0.3) && !update_min  )
         {
             System.out.println("Shortest point to target is here heading towards goal!");
             System.out.println("Current Location: " + robot_pose);
             displayMoveState = "Shortest point to target is here heading towards goal!";
             global_state = GlobalState.MOVE_TO_TARGET;///Switching the state to move to target since the robot is ready to start doing wall following again
        }
      }
      
      if (time_elapsed > target_time) {
        time_elapsed = 0;
            
        if (state == PioneerNav2.MoveState.FOLLOW_WALL)
        {
        displayMoveState = "Object Detected Following Wall";
        target_time = 0;
        nav.follow_wall(robot_velocity, 0.25, true);
        }else
        if (state == PioneerNav2.MoveState.FORWARD) {
          target_time = nav.forward(distance_to_travel, robot_velocity);
        } else 
        if (state == PioneerNav2.MoveState.ARC) {
          target_time = nav.arc(theta, 0.0, robot_velocity);
        } else
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
  
}
