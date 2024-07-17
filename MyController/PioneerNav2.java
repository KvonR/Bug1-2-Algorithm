
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import java.lang.Math;


public class PioneerNav2 {

  public static enum MoveState {
    STOP,
    FORWARD,
    ARC,
    FOLLOW_WALL };

  private Supervisor robot;   // reference to the robot
  private Node robot_node;    // reference to the robot node
  private Pose robot_pose;    // the robots percieved pose
  private Motor left_motor;
  private Motor right_motor;
  private double max_vel;
  private double prev_error;
  private double total_error;
  private MoveState state;
  private PioneerProxSensors1 prox_sensors;

  private final double WHEEL_RADIUS = 0.0957; // in meters 
  private final double AXEL_LENGTH = 0.323;   // in meters 


  // ==================================================================================
  // Constructor
  // ==================================================================================
  public PioneerNav2(Supervisor robot, Pose init_pose,PioneerProxSensors1 ps) {
    this.robot = robot;    
    this.robot_node = this.robot.getSelf();   // reference to the robot node
    this.state = MoveState.STOP;

    // enable motors
    this.left_motor = robot.getMotor("left wheel");
    this.right_motor = robot.getMotor("right wheel");
    this.left_motor.setPosition(Double.POSITIVE_INFINITY);
    this.right_motor.setPosition(Double.POSITIVE_INFINITY);
    
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    this.prox_sensors = ps;
    
    // set up pose
    this.robot_pose = new Pose(init_pose.getX(), init_pose.getY(), init_pose.getTheta());

    // Initialise motor velocity
    this.left_motor.setVelocity(0.0);
    this.right_motor.setVelocity(0.0);  
    
    this.max_vel = this.left_motor.getMaxVelocity() - 0.1; 
    this.prev_error = 0;
    this.total_error = 0;
  } 
  

  // The following method only works if in supervisor mode
  public Pose get_real_pose() {
    if (this.robot_node == null){
      return new Pose(0,0,0);
      }
    double[] realPos = robot_node.getPosition();
    double[] rot = this.robot_node.getOrientation(); // 3x3 Rotation matrix as vector of length 9
    double theta1 = Math.atan2(-rot[0], rot[3]);
    double halfPi = Math.PI/2;
    double theta2 = theta1 + halfPi;
    if (theta1 > halfPi){
        theta2 = -(3*halfPi)+theta1;
    }
    return new Pose(realPos[0], realPos[1], theta2);
  }

  public int forward(double target_dist, double robot_linearvelocity) {
    double wheel_av = (robot_linearvelocity/this.WHEEL_RADIUS);
    double target_time = target_dist/robot_linearvelocity;
    
    this.left_motor.setVelocity(wheel_av);
    this.right_motor.setVelocity(wheel_av);
    this.state = MoveState.FORWARD;
        
    // return target_time as millisecs          
    return (int) (1000.0*target_time);
  }

  public int arc(double icr_angle, double icr_r, double icr_omega) {
    double target_time = icr_angle / icr_omega;
   
    // Calculate each wheel velocity around ICR
    
    double vl = icr_omega * (icr_r - (this.AXEL_LENGTH / 2));
    double vr = icr_omega * (icr_r + (this.AXEL_LENGTH / 2));
     
    double leftwheel_av = (vl/this.WHEEL_RADIUS);
    double rightwheel_av = (vr/this.WHEEL_RADIUS);
        
    this.left_motor.setVelocity(leftwheel_av);
    this.right_motor.setVelocity(rightwheel_av);
    this.state = MoveState.ARC;

    // return target_time as millisecs          
    return (int) (1000.0*target_time);
  }
  
  public void stop() {
    
    this.left_motor.setVelocity(0.0);
    this.right_motor.setVelocity(0.0);
    this.state = MoveState.STOP;
  }
  public double getRightVelocity() {
    return this.right_motor.getVelocity();
  }
    public double getLeftVelocity() {
    return this.left_motor.getVelocity();
  }
  
  public MoveState getState() {
    return this.state;
  }
  
  public void set_velocity(double base, double control) {
  
  // base gives the velocity of the wheels in m/s
  // control is an adjustment on the main velocity
        double base_av = (base/this.WHEEL_RADIUS);
  //System.out.println(base_av);
    double lv = base_av;
    double rv = base_av;
    if (control != 0) {
      double control_av = (control/this.WHEEL_RADIUS);
  // Check if we exceed max velocity and fix if so
      double correction = 1;
      lv = base_av - control_av;
      rv = base_av + control_av;
    if (lv > this.max_vel) {
      correction = this.max_vel / lv;
      lv = lv * correction;
      rv = rv * correction;
    }
    if (rv > this.max_vel) {
      correction = this.max_vel / rv;
      lv = lv * correction;
      rv = rv * correction;
    }
  }

  
  this.left_motor.setVelocity(lv);
  this.right_motor.setVelocity(rv);
  }
  
  private double pid(double error) {
    double kp = 0.8; // proportional weight (may need tuning)
    double kd = 3.0; // differential weight (may need tuning)
    double ki = 0.0; // integral weight (may need tuning)
  
    double prop = error;
    double diff = error - this.prev_error;
    this.total_error += error;
    double control = (kp * prop) + (ki * this.total_error) + (kd * diff);
    this.prev_error = error;
    return control;
  }
  
  public void follow_wall(double robot_linearvelocity, double set_point, boolean right) {
    int direction_coeff = 1;
    double error;
    double control;
    double wall_dist;
    if (right) {
      direction_coeff = -1; 
      } 
  
    if (Math.min(this.prox_sensors.get_value(1),
      Math.min(this.prox_sensors.get_value(2),
      Math.min(this.prox_sensors.get_value(3),
      Math.min(this.prox_sensors.get_value(4),
      Math.min(this.prox_sensors.get_value(5),
      this.prox_sensors.get_value(6)))))) < set_point)
      {
      this.set_velocity(robot_linearvelocity, -0.8*direction_coeff);
      }
  
    else{
      if (!right){
       wall_dist = Math.min(this.prox_sensors.get_value(1),this.prox_sensors.get_value(0));
       }
      else{
        wall_dist = Math.min(this.prox_sensors.get_value(7),this.prox_sensors.get_value(8));
       }
  //System.out.println(wall_dist); 
    if (wall_dist < this.prox_sensors.get_maxRange()) {
  //System.out.println("heybuddy");
      error = wall_dist - set_point;
      control = this.pid(error);
      this.set_velocity(robot_linearvelocity, control*direction_coeff);
      }
    else{
    this.set_velocity(robot_linearvelocity, 0.1*direction_coeff);
     }
    }
    this.state = MoveState.FOLLOW_WALL;
  }
}    