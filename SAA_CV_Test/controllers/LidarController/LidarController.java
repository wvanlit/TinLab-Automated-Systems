// File:          LidarController.java
// Date:
// Description:
// Author:
// Modifications:

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Lidar;

public class LidarController {
  public static void main(String[] args) {

    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());  
    Lidar lidar = robot.getLidar("LDS-01");
    lidar.enable(timeStep);

    while (robot.step(timeStep) != -1) {
      float[] lidarData = lidar.getLayerRangeImage(0);
      printLidarData(lidarData);
    };

    // Enter here exit cleanup code.
  }
  
  public static void printLidarData(float[] lidarData){
      boolean ahead = false;
      boolean left = false;
      boolean right = false;
      boolean behind = false;
      
      for(int i = 0; i < lidarData.length; i++){
        if (lidarData[i] < 0.3){
          //Obstacle is detected
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
      
      System.out.println(ahead);
      System.out.println(left);
      System.out.println(right);
      System.out.println(behind);
      
      if(!ahead){
        if(!left && right){
          System.out.println("Left - Forward"); 
        }
        if(left){
          if(!right){
            System.out.println("Right - Forward"); 
          } else{
            System.out.println("Forward"); 
          }
        }        
      } else {
        if(behind){
          if(left){
            if(right){
              System.out.println("Request Human -- Stuck -- Upwards");
            } else {
              System.out.println("Right");
            }
          } else {
            if(right){
              System.out.println("Left");
            } else {
              System.out.println("Right");
            }
          }
        } else {
          if(left){
            if(!right){
              System.out.println("Right");
            } else {
              System.out.println("Backwards");
            }
          } else {
            if(right){
              System.out.println("Left"); 
            } else {
              System.out.println("Left");
            }
          }
        }
      }
      
      System.out.println(" ");
  }
}
