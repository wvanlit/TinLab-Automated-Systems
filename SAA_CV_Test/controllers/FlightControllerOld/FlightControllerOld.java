// File:          FlightController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;

/*Current problems:
  - Instant stop will crash the robot
  - Impossible coords given will be executed, the drone does not know it is impossible
  - Obstacle avoidance not implemented yet
*/


import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Speaker;
import com.cyberbotics.webots.controller.Lidar;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.Camera;
import java.lang.Math.*;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class FlightController {

  private Robot robot;
  private InertialUnit imu;
  private GPS gps;
  private Compass compass;
  private Gyro gyro;
  private Camera camera;
  private Lidar lidar;
  
  private Motor frontLeftMotor;
  private Motor rearLeftMotor;
  private Motor frontRightMotor;
  private Motor rearRightMotor;
  
  private int timeStep;
  
  // Constants, empirically found.
  private double kVerticalThrust = 68.5;  // with this thrust, the drone lifts.
  private double kVerticalOffset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  private double kVerticalP = 3.0;        // P constant of the vertical PID.
  private double kRollP = 50.0;           // P constant of the roll PID.
  private double kPitchP = 30.0;          // P constant of the pitch PID.
  
  // Variables
  private double targetAltitude = 1.5;  
  private boolean hover = false;
  private double[] targetXZ = {0.0, 0.0};
  private double startTime = 0;
  private static double time;
  private boolean isFlyingToLocation = false;
  private double negBoundary = -1.0;
  private double posBoundary = 1.0;
  // private double startSpeed = 4; Maximale snelheden Beginnend vanaf 4 met increments van 0.1 kan max 10 behaald worden
  private double maxSpeed = 2; //Zonder dat de drone begint te dalen
  private boolean isRotated = false;
  
  static boolean targetOne = false;
  static boolean targetTwo = false;
  static boolean targetThree = false;
  static boolean targetFour = false;
  
  
  
  public FlightController(Robot robot, int timeStep){
    // create the Robot instance.
    this.robot = robot;
    this.timeStep = timeStep;

    // Get Devices
    imu = robot.getInertialUnit("inertial unit");
    imu.enable(timeStep);
    gps = robot.getGPS("gps");
    gps.enable(timeStep);
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    gyro = robot.getGyro("gyro");
    gyro.enable(timeStep);
    camera = robot.getCamera("camera");
    camera.enable(timeStep);
    lidar = robot.getLidar("LDS-01");
    lidar.enable(timeStep);
    
    // Get Motors
    frontLeftMotor = robot.getMotor("front left propeller");
    frontRightMotor = robot.getMotor("front right propeller");
    rearLeftMotor = robot.getMotor("rear left propeller");
    rearRightMotor = robot.getMotor("rear right propeller");   
    Motor[] motors = {frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor};
    
    for(Motor m : motors){
      m.setPosition(Double.POSITIVE_INFINITY);
      m.setVelocity(1.0);
    }
  }
  
  public static void main(String[] args) {
    Robot robot = new Robot();
    FlightController fc = new FlightController(robot, (int) Math.round(robot.getBasicTimeStep()));
    
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (fc.robot.step(fc.timeStep) != -1) {
      time = fc.robot.getTime(); // In seconds
      
      // Disturbances, used to control the drone path
      double rollDisturbance = 0.0;
      double pitchDisturbance = 0.0;
      double yawDisturbance = 0.0;
      
      // fc.ToggleHoverDrone(true);
      
      if (time < 5){
        fc.FlyDrone(0.0, 0.0, 0.0);
      } else {
       if(!targetOne){
         targetOne = fc.FlyDroneToLocation(-3, 1.5, -3);
        //  targetOne = fc.FlyDroneToLocation(30, 15, 30);
       } else if (!targetTwo) {
         targetTwo = fc.FlyDroneToLocation(3, 1.5, -3);
        //  targetTwo = fc.FlyDroneToLocation(-23, 15, 35);
       } else if (!targetThree) {
         targetThree = fc.FlyDroneToLocation(-3, 1.5, 3);
        //  targetThree = fc.FlyDroneToLocation(22, 15, -4);
       } else if (!targetFour) {
         targetFour = fc.FlyDroneToLocation(3, 1.5, 3);
        //  targetFour = fc.FlyDroneToLocation(28, 15, 6);
       } else {
         fc.ToggleHoverDrone(true);
         
       }
      }
    };

    // Enter here exit cleanup code.
  }
  
  private boolean FlyDroneToLocation(double targetX, double targetY, double targetZ){
    // sets start variables when flying
    if(!isFlyingToLocation)
      isRotated = false;
      startTime = time;
    isFlyingToLocation = true;

    // Slightly increases maxspeed
    if (negBoundary > -maxSpeed)
      negBoundary -= 0.05;
    if (posBoundary < maxSpeed)
      posBoundary += 0.05;
    
    // Verplaats Stelsel (Dont know english translation, dutch description for math)
    double correctedX = targetX - gps.getValues()[0];
    double correctedZ = targetZ - gps.getValues()[2];
    
    double xDiff = correctedX;
    double zDiff = correctedZ;
    
    // Bepaal kwadrant
    int kwadrant = 0;
    double targetDegrees = 0;
    
    // System.out.print("correctedX: ");
    // System.out.println(correctedX);
    // System.out.print("correctedZ: ");
    // System.out.println(correctedZ);
    
    
    // kwadrant 1
    if(correctedX > 0 && correctedZ >0){
      targetDegrees = 270 + (Math.atan((correctedX/correctedZ)) * 180 / Math.PI);
      // System.out.println("Kwadrant 1");
    }
  
    // kwadrant 3
    if(correctedX <0 && correctedZ < 0){
      targetDegrees = 90 + (Math.atan((correctedX/correctedZ)) * 180 / Math.PI);
      // System.out.println("Kwadrant 3");
    }
    
    // kwadrant 2
    if(correctedX >0 && correctedZ < 0){
      targetDegrees = 90 + (Math.atan((correctedX/correctedZ)) * 180 / Math.PI);
      // System.out.println("Kwadrant 2");
      }
    
    // kwadrant 4
    if(correctedX < 0 && correctedZ > 0){
      targetDegrees = 270 + (Math.atan((correctedX/correctedZ)) * 180 / Math.PI); 
      // System.out.println("Kwadrant 4");
    }
    
    // Edge cases, die niet in een kwadrant vallen
    if( (int) correctedZ == 0){
      if(correctedX > 0){
        targetDegrees = 0;
      } else {
        targetDegrees = 180;
      }
    }
    
    if( (int) correctedX == 0){
      if(correctedZ > 0){
        targetDegrees = 270;
      } else {
        targetDegrees = 90;
      }
    }
    
    double distanceToTarget = Math.sqrt(Math.pow(xDiff,2) + Math.pow(zDiff, 2));
    double bearingDiff = 0.0;
    distanceToTarget = Math.max(negBoundary, Math.min(posBoundary, distanceToTarget));
    targetAltitude = targetY;
    
    double[] obstacleMessage = obstacleDetection();
    double rollDisturbance = 0.0;
    if(obstacleMessage[0] > 0.0){
      switch ((int)obstacleMessage[0]){
        case 1:
          // Adjust Roll (left/right forward)
          System.out.println("Case 1");
          rollDisturbance = obstacleMessage[1];
          break;
        case 2:
          // Adjust Roll and reset Forward (left/right)
          System.out.println("Case 2");
          rollDisturbance = obstacleMessage[1];
          distanceToTarget = 0.0;
          break;
        case 3:
          // Revert forward vector (backwards)
          System.out.println("Case 3");
          distanceToTarget = -1.0;
          break;
      }
    }
    /*
    Notes:
      - Optimal flight path may be impossible, what do we do about this?
      - Drone may bounce between dodging and going face first into obstacle again.
    */

    // Decision making based on if the drone still holds right trajectory
    // Rotating is done if target degree deviates to much from trajectory degree
    if(!RotateDroneToDegrees(targetDegrees) && isRotated == false){
      negBoundary = -1.0;
      posBoundary = 1.0;
    } else if (!RotateDroneToDegrees(targetDegrees) && isRotated == true){
      // Only intervenes when rotation deviation is too big
      bearingDiff = CalculateBearingDifference(targetDegrees);
      if (Math.abs(bearingDiff) > 20){
        isRotated = false;
      } else {
        FlyDrone(rollDisturbance, distanceToTarget, bearingDiff*0.01);
      }
    } else {
      // Straight trajectory if everything is perfect
      FlyDrone(rollDisturbance, distanceToTarget, 0.0);
      isRotated = true;
    }

    System.out.print("Distance to point");
    System.out.println(distanceToTarget);
    System.out.println("---");

    // Checks if destination is reached, magic numbers tweakable for precision
    if (Math.abs(xDiff) < 0.75 && Math.abs(zDiff) < 0.75){
      isFlyingToLocation = false;
      negBoundary = -1.0;
      posBoundary = 1.0;
      isRotated = false;
      return true;
    } else {
      return false;
    }
  }
  
  private double CalculateBearingDifference(double targetDegrees){
    double bearingDiff = 0;
      double currentDegree = getBearingInDegrees();
      if (targetDegrees > currentDegree){
        if(targetDegrees >= 270.0 && currentDegree <= 90.0){
          bearingDiff = (360 - targetDegrees + currentDegree);
        } else {
          bearingDiff = -(targetDegrees - currentDegree);
        }
      } else if (currentDegree > targetDegrees){
        if(currentDegree >= 270.0 && targetDegrees <= 90.0){
          bearingDiff = -(360 - currentDegree + targetDegrees);
        } else {
          bearingDiff = (currentDegree - targetDegrees);
        }
      }
      // System.out.print("Target Degrees: ");
      // System.out.println(targetDegrees);
      // System.out.print("Current Degrees: ");
      // System.out.println(currentDegree);
      // System.out.print("bearingDiff: ");
      // System.out.println(bearingDiff);
      return bearingDiff;
  }
  
  private boolean RotateDroneToDegrees(double targetDegrees){
    
    if(Math.round(targetDegrees) != Math.round(getBearingInDegrees())){
      double bearingDiff = CalculateBearingDifference(targetDegrees);
      bearingDiff = 0.01*bearingDiff;
      if (bearingDiff < 100)
        bearingDiff = bearingDiff/2;
      if (!isRotated){
        FlyDrone(0.0, 0.0, Math.max(-1.0, Math.min(1.0, bearingDiff)));
        // System.out.println("Rotating The Drone");
      }
      return false;
    } else {
      return true;
    }
  }
  
  private void FlyDrone(double rollDisturbance, double pitchDisturbance, double yawDisturbance){
    // Retrieve robot position using the sensors.
    double roll = imu.getRollPitchYaw()[0] + Math.PI / 2.0;
    double pitch = imu.getRollPitchYaw()[1];
    double altitude = gps.getValues()[1];
    double rollAcceleration = gyro.getValues()[0];
    double pitchAcceleration = gyro.getValues()[1];
    
    // Compute the roll, pitch, yaw and vertical inputs.
    double rollInput = kRollP * Math.max(-1.0, Math.min(1.0, roll)) + rollAcceleration + rollDisturbance;
    double pitchInput = kPitchP * Math.max(-1.0, Math.min(1.0, pitch)) - pitchAcceleration + pitchDisturbance;
    double yawInput = yawDisturbance;
    double clampedDifferenceAltitude = Math.max(-1.0, Math.min(1.0, (targetAltitude - altitude + kVerticalOffset)));
    double verticalInput = kVerticalP * Math.pow(clampedDifferenceAltitude, 3.0);
    
    // Actuate the motors taking into consideration all the computed inputs.
    double frontLeftMotorInput = kVerticalThrust + verticalInput - rollInput - pitchInput + yawInput;
    double frontRightMotorInput = kVerticalThrust + verticalInput + rollInput - pitchInput - yawInput;
    double rearLeftMotorInput = kVerticalThrust + verticalInput - rollInput + pitchInput - yawInput;
    double rearRightMotorInput = kVerticalThrust + verticalInput + rollInput + pitchInput + yawInput;
    
    frontLeftMotor.setVelocity(frontLeftMotorInput);
    frontRightMotor.setVelocity(-frontRightMotorInput);
    rearLeftMotor.setVelocity(-rearLeftMotorInput);
    rearRightMotor.setVelocity(rearRightMotorInput);
    
  }
  
  private void ToggleHoverDrone(boolean mustHover){
    if (mustHover){
      if(!hover){
        hover = true;
        targetXZ[0] = gps.getValues()[0];
        targetXZ[1] = gps.getValues()[2];
      }
      double xDiff = gps.getValues()[0] - targetXZ[0];
      double yDiff = gps.getValues()[2] - targetXZ[1];
      double bearingDiff;
      if (getBearingInDegrees() > 180){
        bearingDiff = (getBearingInDegrees() - 360);
      } else {
        bearingDiff = getBearingInDegrees();
      }
      FlyDrone(yDiff,-xDiff,0.1 * bearingDiff / 10);
    } else {
      hover = false;
    }
    
  }
  
  private double getBearingInDegrees(){
    double[] north = compass.getValues();
    double rad = Math.atan2(north[0], north[1]);
    double bearing = (rad - 1.5708) / Math.PI * 180.0;
    if (bearing < 0.0)
      bearing = bearing + 360.0;
    return bearing;
  }

  private double[] obstacleDetection(){
    float[] lidarData = lidar.getLayerRangeImage(0);

    boolean ahead = false;
    boolean left = false;
    boolean right = false;
    boolean behind = false;
    
    for(int i = 0; i < lidarData.length; i++){
      // Checks if Obstacle is detected for every ray
      if (lidarData[i] < 3.3 && lidarData[i] > 2.5){
        System.out.println("Cunt");
      }
      
      if (lidarData[i] < 2.5){
        // Gets direction of detected obstacle from drones perspective 
        if (i <= 45 || i >315){
          ahead = true;
          continue;
        }
        if (i <= 135){
          left = true;
          i = 136;
          continue;
        }
        if (i <= 225){
          behind = true;
          i = 226;
          continue;
        }
        if (i <= 315){
          right = true;
          i = 316;
          continue;
        }
      }
    }
    
    // System.out.println(ahead);
    // System.out.println(left);
    // System.out.println(right);
    // System.out.println(behind);
    
    // Decision making based on detected obstacles, gives best course of action.
    if(!ahead){
      if(!left && right){
        System.out.println("Left - Forward");
        return new double[] {1.0, 1.0};
      }
      if(left){
        if(!right){
          System.out.println("Right - Forward");
          return new double[] {1.0, -1.0};
        } else{
          System.out.println("Forward");
          return new double[] {0.0, 0.0}; 
        }
      }        
    } else {
      if(behind){
        if(left){
          if(right){
            System.out.println("Request Human -- Stuck -- Upwards"); // Only activates when the drone is fully blocked
            return new double[] {-1.0, 0.0};
          } else {
            System.out.println("Right");
            return new double[] {2.0, -1.0};
          }
        } else {
          if(right){
            System.out.println("Left");
            return new double[] {2.0, 1.0};
          } else {
            System.out.println("Right");
            return new double[] {2.0, -1.0};
          }
        }
      } else {
        if(left){
          if(!right){
            System.out.println("Right");
            return new double[] {2.0, -1.0};
          } else {
            System.out.println("Backwards");
            return new double[] {3.0, 0.0};
          }
        } else {
          if(right){
            System.out.println("Left");
            return new double[] {2.0, 1.0};
          } else {
            System.out.println("Left");
            return new double[] {2.0, 1.0};
          }
        }
      }
    }
    return new double[] {0.0, 0.0};
}
  
  
}
