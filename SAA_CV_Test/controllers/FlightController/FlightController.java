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
  private Motor cameraMotor;
  
  private int timeStep;
  
  // Constants, empirically found.
  private double kVerticalThrust = 68.5;  // with this thrust, the drone lifts.
  private double kVerticalOffset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  private double kVerticalP = 3.0;        // P constant of the vertical PID.
  private double kRollP = 50.0;           // P constant of the roll PID.
  private double kPitchP = 30.0;          // P constant of the pitch PID.
  
  // Variables
  private double targetAltitude = 15;  
  private boolean hover = false;
  private double[] targetXZ = {0.0, 0.0};
  private static double time;
  private boolean isFlyingToLocation = false;
  private double negBoundary = -1.0;
  private double posBoundary = 1.0;
  // private double startSpeed = 4; Maximale snelheden Beginnend vanaf 4 met increments van 0.1 kan max 10 behaald worden
  private double maxSpeed = 10.0; //Zonder dat de drone begint te dalen
  private double safeSpeed = 1.5; // Special Manouvers speed (Obstacle avoidance)
  private boolean isRotated = false;
  private double currentRoll = 0.0;
  private boolean targetReached = false;
  
  static boolean targetOne = false;
  static boolean targetTwo = false;
  static boolean targetThree = false;
  static boolean targetFour = false;
  static boolean mappingDone = false;
  
  
  
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
    cameraMotor = robot.getMotor("camera pitch");
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
      
      if (time < 5){
        fc.FlyDrone(0.0, 0.0, 0.0);
      } else {
        /*
        // Get new target if available       
        double[] targets = fc.getTargets();
        // Fly to target written in targetXZ
        boolean targetReached = fc.FlyDroneToLocation(targets[0], 20, targets[1]);
        // Hover drone if targetReached
        if (targetReached){
          fc.ToggleHoverDrone(true);
          fc.targetReached();
        } else {
          fc.ToggleHoverDrone(false);
        }
        */
        /* Test Set*/
        if(!targetOne){
          //  targetOne = fc.FlyDroneToLocation(-3, 1.5, -3);
          //  targetOne = fc.FlyDroneToLocation(30, 15, 30);
          targetOne = fc.FlyDroneToLocation(45, 15, 30);
          // targetOne = true;
        } else if (!targetTwo) {
          //  targetTwo = fc.FlyDroneToLocation(3, 1.5, -3);
          //  targetTwo = fc.FlyDroneToLocation(-25, 15, 35);
          targetTwo = fc.FlyDroneToLocation(-45, 15, 30);
        } else if (!targetThree) {
          //  targetThree = fc.FlyDroneToLocation(-3, 1.5, 3);
          //  targetThree = fc.FlyDroneToLocation(22, 15, -4);
          targetThree = fc.FlyDroneToLocation(-45, 15, -45);
        } else if (!targetFour) {
          //  targetFour = fc.FlyDroneToLocation(3, 1.5, 3);
          targetFour = fc.FlyDroneToLocation(40, 15, -45);
        } else {
          // fc.ToggleHoverDrone(true);
          targetOne = false;
          targetTwo = false;
          targetThree = false;
          targetFour = false;
        }
        // */
      }
    };

  }

  public void targetReached(){
    targetReached = true;
  }
  
  private double calculateTargetDegrees(double x, double z){
    double targetDegrees = 0;
    // kwadrant 1
    if(x > 0 && z >0){
      targetDegrees = 270 + (Math.atan((x/z)) * 180 / Math.PI);
    }
  
    // kwadrant 3
    if(x <0 && z < 0){
      targetDegrees = 90 + (Math.atan((x/z)) * 180 / Math.PI);
    }
    
    // kwadrant 2
    if(x >0 && z < 0){
      targetDegrees = 90 + (Math.atan((x/z)) * 180 / Math.PI);
      }
    
    // kwadrant 4
    if(x < 0 && z > 0){
      targetDegrees = 270 + (Math.atan((x/z)) * 180 / Math.PI); 
    }
    
    // Edge cases, die niet in een kwadrant vallen
    if( (int) z == 0){
      if(x > 0){
        targetDegrees = 0;
      } else {
        targetDegrees = 180;
      }
    }
    
    if( (int) x == 0){
      if(z > 0){
        targetDegrees = 270;
      } else {
        targetDegrees = 90;
      }
    }
    return targetDegrees;
  }

  private boolean FlyDroneToLocation(double targetX, double targetY, double targetZ){
    // sets start variables when flying
    boolean mustAccelerate;
    if(!isFlyingToLocation)
      isRotated = false;
      mustAccelerate = true;
    isFlyingToLocation = true;

    // Slightly increases maxspeed
    if (mustAccelerate){
      if (negBoundary > -maxSpeed)
        negBoundary -= 0.05;
      if (posBoundary < maxSpeed)
        posBoundary += 0.05;
    }
    
    // Verplaats Stelsel (Dont know english translation, dutch description for math)
    double correctedX = targetX - gps.getValues()[0];
    double correctedZ = targetZ - gps.getValues()[2];
    // Bepaal kwadrant
    double targetDegrees = calculateTargetDegrees(correctedX, correctedZ);
    double distanceToTarget = Math.sqrt(Math.pow(correctedX,2) + Math.pow(correctedZ, 2));
    double bearingDiff = 0.0;
    targetAltitude = targetY;    
    double[] obstacleMessage = obstacleDetection();
    double rollDisturbance = 0.0;
    boolean objectNear = false;
    if(obstacleMessage[0] > 0.0){
      switch ((int)obstacleMessage[0]){
        case 1:
          // Adjust Roll (left/right forward)
          rollDisturbance = obstacleMessage[1];
          objectNear = true;
          mustAccelerate = false;
          break;
        case 2:
          // Adjust Roll and reset Forward (left/right)
          rollDisturbance = obstacleMessage[1];
          distanceToTarget = 0.0;
          objectNear = true;
          mustAccelerate = false;
          break;
        case 3:
          // Revert forward vector (backwards)
          rollDisturbance = 0.0;
          distanceToTarget = -1.0;
          objectNear = true;
          mustAccelerate = false;
          break;
        case 4:
          // Object near, not yet in deflecting range
          rollDisturbance = 0.0;
          objectNear = true;
          mustAccelerate = false;
          break;
        default:
          rollDisturbance = 0.0;
          objectNear = false;
          mustAccelerate = true;
          break;
      }
    }
    if (objectNear){
      // Slow the drone down
      if (negBoundary < -safeSpeed)
        negBoundary += 0.2;
      if (posBoundary > safeSpeed)
        posBoundary -= 0.2;
    }
    if (rollDisturbance > 0){
      if (currentRoll < rollDisturbance){
        currentRoll += 0.005;
      }
    } else if (rollDisturbance < 0){
      if (currentRoll > rollDisturbance){
        currentRoll -= 0.005;
      }
    }
    // Cast distance to a usable speed 
    double dronePitch = Math.max(negBoundary, Math.min(posBoundary, distanceToTarget));

    //Rotating the camera to the best viewpoint
    double clampedDronePitch = Math.max(0, Math.min(1.7, dronePitch));
    double cameraPitch = Math.abs(1.7 - clampedDronePitch);
    cameraMotor.setPosition(cameraPitch);
    System.out.println(cameraPitch);

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
        FlyDrone(currentRoll, dronePitch, bearingDiff*0.01);
      }
    } else {
      // Straight trajectory if everything is perfect
      FlyDrone(currentRoll, dronePitch, 0.0);
      isRotated = true;
    }

    // Checks if destination is reached, magic numbers tweakable for precision
    if (Math.abs(correctedX) < 0.75 && Math.abs(correctedZ) < 0.75){
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
    boolean objectNear = false;
    
    for(int i = 0; i < lidarData.length; i++){
      // Checks if Obstacle is detected for every ray
      if (lidarData[i] < 3.3 && lidarData[i] > 1.2){
        objectNear =  true;
      }
      
      if (lidarData[i] < 1.0){
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

    // Decision making based on detected obstacles, gives best course of action.
    if(!ahead){
      if(!left && right){
        // Left - Forward
        return new double[] {1.0, 1.0};
      }
      if(left){
        if(!right){
          // Right - Forward
          return new double[] {1.0, -1.0};
        } else{
          // Forward
          return new double[] {0.0, 0.0}; 
        }
      }        
    } else {
      if(behind){
        if(left){
          if(right){
            // Only activates when the drone is fully blocked
            return new double[] {-1.0, 0.0};
          } else {
            // Right
            return new double[] {2.0, -1.0};
          }
        } else {
          if(right){
            // Left
            return new double[] {2.0, 1.0};
          } else {
            // Right
            return new double[] {2.0, -1.0};
          }
        }
      } else {
        if(left){
          if(!right){
            // Right;
            return new double[] {2.0, -1.0};
          } else {
            // Backwards
            return new double[] {3.0, 0.0};
          }
        } else {
          if(right){
            // Left
            return new double[] {2.0, 1.0};
          } else {
            // Left
            return new double[] {2.0, 1.0};
          }
        }
      }
    }
    if (objectNear){
      return new double[] {4.0, 0.0};
    }
    return new double[] {0.0, 0.0};
} 
}