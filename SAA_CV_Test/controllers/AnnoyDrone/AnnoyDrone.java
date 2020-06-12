// File:          AnnoyDrone.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;


import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Speaker;
import com.cyberbotics.webots.controller.Lidar;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class AnnoyDrone {

  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {

    // create the Robot instance.
    Robot robot = new Robot();

    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Get Devices
    Speaker speaker = robot.getSpeaker("speaker");
    
    Lidar lidar = robot.getLidar("LDS-01");
    lidar.enable(timeStep);
    
    FlightController fc = new FlightController(robot, timeStep);
    
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {
    
      // Read the sensors:
      float[] lidarData = lidar.getLayerRangeImage(0);
      
      
      // Process sensor data here.
      printLidarData(lidarData);

      // Enter here functions to send actuator commands, like:
      if(!speaker.isSpeaking())
        speaker.speak("Fuck off and go home, you bloody idiots", 1);
    };

    // Enter here exit cleanup code.
  }
  
  public static void printLidarData(float[] lidarData){
      for(float f : lidarData){
        System.out.print(f);
        System.out.print(" ");   
      }
      System.out.print("| ");
      System.out.println(lidarData.length);
  }
}
