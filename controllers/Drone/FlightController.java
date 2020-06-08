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
import com.cyberbotics.webots.controller.ImageRef;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.Receiver;
import java.lang.Math.*;
import java.util.HashMap;
import java.util.Map;

import org.opencv.core.*;
import org.opencv.dnn.*;
import org.opencv.utils.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.videoio.VideoCapture;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;

import java.io.*;

import javax.imageio.ImageIO;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class FlightController {

  private Robot robot;
  private InertialUnit imu;
  private GPS gps;
  private Compass compass;
  private Gyro gyro;
  private Lidar lidar;

  private Motor frontLeftMotor;
  private Motor rearLeftMotor;
  private Motor frontRightMotor;
  private Motor rearRightMotor;
  private Motor cameraMotor;

  private int timeStep;
  private int detectionCount;

  // Constants, empirically found.
  private double kVerticalThrust = 68.5; // with this thrust, the drone lifts.
  private double kVerticalOffset = 0.6; // Vertical offset where the robot actually targets to stabilize itself.
  private double kVerticalP = 3.0; // P constant of the vertical PID.
  private double kRollP = 50.0; // P constant of the roll PID.
  private double kPitchP = 30.0; // P constant of the pitch PID.

  // Variables
  private double targetAltitude = 15;
  private boolean hover = false;
  private double[] targetXZ = { 0.0, 0.0 };
  private static double time;
  private boolean isFlyingToLocation = false;
  private double negBoundary = -1.0;
  private double posBoundary = 1.0;
  // private double startSpeed = 4; Maximale snelheden Beginnend vanaf 4 met
  // increments van 0.1 kan max 10 behaald worden
  private double maxSpeed = 10.0; // Zonder dat de drone begint te dalen
  private double safeSpeed = 1.5; // Special Manouvers speed (Obstacle avoidance)
  private boolean isRotated = false;
  private double currentRoll = 0.0;
  private boolean targetReached = false;
  private Map<Integer, double[]> objectAvoidanceMap;

  static boolean targetOne = false;
  static boolean targetTwo = false;
  static boolean targetThree = false;
  static boolean targetFour = false;
  static boolean mappingDone = false;

  public static String modelWeights = "YOLO\\yolov3.weights"; // Download and load only weights for YOLO , this is
                                                              // obtained from official YOLO site//
  public static String modelConfiguration = "YOLO\\yolov3.cfg.txt";// Download and load cfg file for YOLO , can be
                                                                   // obtained from official site//
  public static String openCvLibPath = "C:\\OpenCV\\opencv\\build\\java\\x64\\opencv_java3410.dll";
  public static String cocoFilePath = "YOLO\\coco.txt";
  public static int preferedCameraAngle = -35;
  public static Vector<String> classes = new Vector<String>();

  public FlightController(Robot robot, int timeStep) {
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
    lidar = robot.getLidar("LDS-01");
    lidar.enable(timeStep);

    // Get Motors
    cameraMotor = robot.getMotor("camera pitch");
    frontLeftMotor = robot.getMotor("front left propeller");
    frontRightMotor = robot.getMotor("front right propeller");
    rearLeftMotor = robot.getMotor("rear left propeller");
    rearRightMotor = robot.getMotor("rear right propeller");

    cameraMotor.setPosition(1.7);

    Motor[] motors = { frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor };

    for (Motor m : motors) {
      m.setPosition(Double.POSITIVE_INFINITY);
      m.setVelocity(1.0);
    }

    objectAvoidanceMap = new HashMap<>();
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(false, false, false, false),new double[]{0.0, 0.0});
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(true, false, false, false), new double[]{2.0, 1.0}); // Left
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(false, true, false, false), new double[]{3.0, 0.0}); // back
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(true, true, false, false), new double[]{2.0, 1.0}); // Left
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(false, false, true, false), new double[]{1.0, -1.0}); // Ahead - Right 
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(true, false, true, false), new double[]{2.0, -1.0}); // Right
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(false, true, true, false), new double[]{1.0, -1.0}); // Ahead - Right 
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(true, true, true, false), new double[]{2.0, -1.0}); // Right
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(false, false, false, true), new double[]{1.0, 1.0}); // Ahead - Left 
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(true, false, false, true), new double[]{2.0, 1.0}); // Left
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(false, true, false, true), new double[]{1.0, 1.0}); // Ahead - Left
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(true, true, false, true), new double[]{2.0, 1.0}); // Left
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(false, false, true, true), new double[]{0.0, 0.0}); // Ahead
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(true, false, true, true), new double[]{3.0, 0.0}); // Backwards
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(false, true, true, true), new double[]{0.0, 0.0}); // Ahead
    objectAvoidanceMap.put(turnObstacleDirectionsToInt(true, true, true, true), new double[]{-1.0, 0.0}); // Blocked
  }
/*
  public static void main(String[] args) {
    Robot robot = new Robot();
    int tempTimeStep = (int) Math.round(robot.getBasicTimeStep());

    FlightController fc = new FlightController(robot, tempTimeStep);
    fc.initYolo();

    Camera camera = robot.getCamera("camera");
    camera.enable(fc.timeStep);

    Display display = robot.getDisplay("display");
    display.attachCamera(camera);
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (fc.robot.step(fc.timeStep) != -1) {
      time = fc.robot.getTime(); // In seconds

      if (time % 0.5 < 0.01) {
        try {

          // fc.ReturnCoordsOfDetectedHumans(display, camera, fc);
        } catch (Exception e) {
          // System.out.println("Nothing to detect "+ e);
        }
      }

      // Disturbances, used to control the drone path
      double rollDisturbance = 0.0;
      double pitchDisturbance = 0.0;
      double yawDisturbance = 0.0;

      if (time < 5) {
        fc.FlyDrone(0, 0, 0);
      } else {
        /*
         * // voor integratie met de servers // Get new target if available double[]
         * targets = fc.getTargets(); // Fly to target written in targetXZ boolean
         * targetReached = fc.FlyDroneToLocation(targets[0], 20, targets[1]); // Hover
         * drone if targetReached if (targetReached){ fc.ToggleHoverDrone(true);
         * fc.targetReached(); } else { fc.ToggleHoverDrone(false); }
         */
        /* Test Set 
        if (!targetOne) {
          targetOne = fc.FlyDroneToLocation(45, 15, 30);
          // targetOne = true;
        } else if (!targetTwo) {
          targetTwo = fc.FlyDroneToLocation(-45, 15, 30);
        } else if (!targetThree) {
          targetThree = fc.FlyDroneToLocation(-40, 15, -40);
        } else if (!targetFour) {
          targetFour = fc.FlyDroneToLocation(40, 15, -40);
        } else {
          // fc.ToggleHoverDrone(true);
          targetOne = false;
          targetTwo = false;
          targetThree = false;
          targetFour = false;
        }
        // 
      }
    }
  }
  */

  public void initYolo() {
    // Load the openCV 3 dll //
    System.load(openCvLibPath); 
    try {
      File classesFile = new File(cocoFilePath);
      BufferedReader br = new BufferedReader(new FileReader(classesFile));
      String st;
      while ((st = br.readLine()) != null) {
        classes.addElement(st);
      }
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  // output layer name
  private List<String> getOutputNames(Net net) { 
    List<String> names = new ArrayList<>();

    List<Integer> outLayers = net.getUnconnectedOutLayers().toList();
    List<String> layersNames = net.getLayerNames();
    // unfold and create R-CNN layers from the loaded YOLO model//
    outLayers.forEach((item) -> names.add(layersNames.get(item - 1)));
    return names;
  }

  public List<double[]> ReturnCoordsOfDetectedHumans(Display display, Camera camera, FlightController fc) {
    // define a matrix to extract and store pixel info from video//
    Mat frame = new Mat(camera.getWidth(), camera.getHeight(), CvType.CV_8UC3); 

    ImageRef ir = display.imageNew(camera.getWidth(), camera.getHeight(), camera.getImage(), 5);
    String imageName = "image"+Integer.toString(robot.getReceiver("receiver").getChannel())+".png";
    display.imageSave(ir, imageName);

    frame = Imgcodecs.imread(imageName);

    Mat dst = new Mat();
    // OpenCV DNN supports models trained from various frameworks like Caffe and TensorFlow 
    // It also supports various networks architectures based on YOLO//
    Net net = Dnn.readNetFromDarknet(modelConfiguration, modelWeights); 
    Size sz = new Size(camera.getWidth(), camera.getHeight());
    List<Mat> result = new ArrayList<>();
    List<String> outBlobNames = getOutputNames(net);

    int timer = 0;
    byte[] imageFrame = toByteArray(camera);
    int camWidth = camera.getWidth();
    // We feed one frame of video into the network at a time, we have to convert the image to a blob.
    // A blob is a pre-processed image that serves as the input.//
    Mat blob = Dnn.blobFromImage(frame, 0.00392, sz, new Scalar(0), true, false); 
    net.setInput(blob);
    // Feed forward the model to get output //
    net.forward(result, outBlobNames); 

    // Insert thresholding beyond which the model will ReturnCoordsOfDetectedHumans objects//
    float confThreshold = 0.5f; 
    List<Integer> clsIds = new ArrayList<>();
    List<Float> confs = new ArrayList<>();
    List<Rect2d> rects = new ArrayList<>();
    List<double[]> humanCoords = new ArrayList<>();
    for (int i = 0; i < result.size(); ++i) {
      // each row is a candidate detection, the 1st 4 numbers are
      // [center_x, center_y, width, height], followed by (N-4) class probabilities
      Mat level = result.get(i);
      for (int j = 0; j < level.rows(); ++j) {

        Mat row = level.row(j);
        Mat scores = row.colRange(5, level.cols());
        Core.MinMaxLocResult mm = Core.minMaxLoc(scores);

        float confidence = (float) mm.maxVal;
        Point classIdPoint = mm.maxLoc;

        if (confidence > confThreshold) {
          // scaling for drawing the bounding boxes//
          int centerX = (int) (row.get(0, 0)[0] * frame.cols()); 
          int centerY = (int) (row.get(0, 1)[0] * frame.rows());
          int width = (int) (row.get(0, 2)[0] * frame.cols());
          int height = (int) (row.get(0, 3)[0] * frame.rows());
          int left = centerX - width / 2;
          int top = centerY - height / 2;
          if ((int) classIdPoint.x == 0) {

            if (height > 44) {
              // height in pix * mesured distance / real hight(prox)
              double focalLength = (116 * 8.29) / 1.8;
              double dis = (1.8 * focalLength) / height;
              clsIds.add((int) classIdPoint.x);
              confs.add(confidence);
              rects.add(new Rect2d(left, top, width, height));

              // System.out.println("distance= " + dis);

              int degrees = (int) getBearingInDegrees();
              System.out.print("'Cunt'    ");
              System.out.println(degrees);
              if (degrees > 175 && degrees < 182) {
                double x = fc.gps.getValues()[0];
                double newX = x - dis;
                System.out.println("Coords from detectd humans: " + newX + " z =" + fc.gps.getValues()[2]);
                humanCoords.add(new double[] {newX, fc.gps.getValues()[2]});
              }
              if ((degrees > 355 && degrees < 360) || (degrees > 0 && degrees < 5)) {
                humanCoords.add(new double[] {fc.gps.getValues()[0] + dis, fc.gps.getValues()[2]});
                System.out
                    .println("Coords from detectd humans: " + fc.gps.getValues()[0] + dis + " z =" + fc.gps.getValues()[2]);
              }

              if (degrees > 85 && degrees < 95) {
                humanCoords.add(new double[] {fc.gps.getValues()[0], fc.gps.getValues()[2]+dis});
                System.out
                    .println("Coords from detectd humans: " + fc.gps.getValues()[0] + " z =" + fc.gps.getValues()[2] + dis);
              }
              if ((degrees > 265 && degrees < 275)) {
                double z = fc.gps.getValues()[2];
                double newZ = z - dis;
                humanCoords.add(new double[] {fc.gps.getValues()[0], newZ});
                System.out.println("Coords from detectd humans: " + fc.gps.getValues()[0] + " z =" + newZ);
              }
            }
          }
        }
      }
    }
    float nmsThresh = 0.5f;

    MatOfFloat confidences = new MatOfFloat(Converters.vector_float_to_Mat(confs));
    Rect2d[] boxesArray = rects.toArray(new Rect2d[0]);
    MatOfRect2d boxes = new MatOfRect2d(boxesArray);
    MatOfInt indices = new MatOfInt();
    // We draw the bounding boxes for objects here//
    Dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThresh, indices); 

    int[] ind = indices.toArray();
    int j = 0;

    for (int i = 0; i < ind.length; ++i) {
      int idx = ind[i];
      Rect2d box = boxesArray[idx];
      Imgproc.rectangle(frame, box.tl(), box.br(), new Scalar(0, 0, 255), 2);
      // System.out.println("coordinaten "+ box.width + " " + box.height);
    }
    detectionCount++;
    // Save result to file
    Imgcodecs.imwrite("detected" + detectionCount + ".jpg", frame); 
    

    return humanCoords;
    
    /*
     * Output conversion for webots
     * 
     * MatOfInt rgb = new MatOfInt(CvType.CV_32SC3); frame.convertTo(rgb,
     * CvType.CV_32SC3); int[] processedImage = new
     * int[(int)(rgb.total()*rgb.channels())]; rgb.get(0,0,processedImage); ImageRef
     * ir = display.imageNew(camera.getWidth(), camera.getHeight(),
     * processedImage,3); display.imageSave(ir, "test1234.png");
     * System.out.println("saved");
     */
  }

  private void SendCoordinatesToServer(int _bodyCount, GPS _gps) {
    System.out.println("Coordinates: " + _gps.getValues()[0] + " - " + _gps.getValues()[1] + " - " + _gps.getValues()[2]);
    System.out.println(_bodyCount + " Persons detected");
  }

  // convert camera image to opencv standard
  public static byte[] toByteArray(Camera cam) { 

    int[] ints = cam.getImage();
    byte[] bytes = new byte[ints.length * 4];
    byte r = 0, g = 0, b = 0, gr = 0;
    for (int i = 0; i < ints.length; i += 4) {
      int pixel = ints[i];
      r = (byte) Camera.pixelGetRed(pixel);
      g = (byte) Camera.pixelGetGreen(pixel);
      b = (byte) Camera.pixelGetBlue(pixel);
      gr = (byte) Camera.pixelGetGray(pixel);
      bytes[i] = r;
      bytes[i + 1] = g;
      bytes[i + 2] = b;
      bytes[i + 3] = gr;

    }
    return bytes;
  }

  ////////////////////////////////////////////////// FLIGHT////////////////////////////////////////////
  public void targetReached() {
    targetReached = true;
  }

  private double calculateTargetDegrees(double x, double z) {
    double targetDegrees = 0;
    // quadrant 1
    if (x > 0 && z > 0) {
      targetDegrees = 270 + (Math.atan((x / z)) * 180 / Math.PI);
    }

    // quadrant 3
    if (x < 0 && z < 0) {
      targetDegrees = 90 + (Math.atan((x / z)) * 180 / Math.PI);
    }

    // quadrant 2
    if (x > 0 && z < 0) {
      targetDegrees = 90 + (Math.atan((x / z)) * 180 / Math.PI);
    }

    // quadrant 4
    if (x < 0 && z > 0) {
      targetDegrees = 270 + (Math.atan((x / z)) * 180 / Math.PI);
    }

    // Edge cases, if coords are not in a quadrant
    if ((int) z == 0) {
      if (x > 0) {
        targetDegrees = 0;
      } else {
        targetDegrees = 180;
      }
    }

    if ((int) x == 0) {
      if (z > 0) {
        targetDegrees = 270;
      } else {
        targetDegrees = 90;
      }
    }
    return targetDegrees;
  }

  public boolean FlyDroneToLocation(double targetX, double targetY, double targetZ) {
    // sets start variables when flying
    boolean mustAccelerate = false;

    if (!isFlyingToLocation)
      isRotated = false;
    mustAccelerate = true;
    isFlyingToLocation = true;

    // Slightly increases maxspeed
    if (mustAccelerate) {
      if (negBoundary > -maxSpeed)
        negBoundary -= 0.05;
      if (posBoundary < maxSpeed)
        posBoundary += 0.05;
    }

    // Recalculate global coordinate system to relational coordinate system (Drone is at 0,0)
    double correctedX = targetX - gps.getValues()[0];
    double correctedZ = targetZ - gps.getValues()[2];
    // Define quadrant
    double targetDegrees = calculateTargetDegrees(correctedX, correctedZ);
    double distanceToTarget = Math.sqrt(Math.pow(correctedX, 2) + Math.pow(correctedZ, 2));
    double bearingDiff = 0.0;
    targetAltitude = targetY;
    double[] obstacleMessage = obstacleDetection();
    double rollDisturbance = 0.0;
    boolean objectNear = false;
    if (obstacleMessage[0] > 0.0) {
      switch ((int) obstacleMessage[0]) {
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
    if (objectNear) {
      // Slow the drone down
      if (negBoundary < -safeSpeed)
        negBoundary += 0.2;
      if (posBoundary > safeSpeed)
        posBoundary -= 0.2;
    }
    if (rollDisturbance > 0) {
      if (currentRoll < rollDisturbance) {
        currentRoll += 0.005;
      }
    } else if (rollDisturbance < 0) {
      if (currentRoll > rollDisturbance) {
        currentRoll -= 0.005;
      }
    }
    // Cast distance to a usable speed
    double dronePitch = Math.max(negBoundary, Math.min(posBoundary, distanceToTarget));

    // Rotating the camera to the best viewpoint
    double dronePitchInRadians = dronePitch * (Math.PI / 180);
    double cameraPitch = -(preferedCameraAngle * (Math.PI / 180)) - dronePitchInRadians;
    cameraMotor.setPosition(cameraPitch);

    // Decision making based on if the drone still holds right trajectory
    // Rotating is done if target degree deviates to much from trajectory degree
    if (!RotateDroneToDegrees(targetDegrees) && isRotated == false) {
      negBoundary = -1.0;
      posBoundary = 1.0;
    } else if (!RotateDroneToDegrees(targetDegrees) && isRotated == true) {
      // Only intervenes when rotation deviation is too big
      bearingDiff = CalculateBearingDifference(targetDegrees);
      if (Math.abs(bearingDiff) > 20) {
        isRotated = false;
      } else {
        FlyDrone(currentRoll, dronePitch, bearingDiff * 0.01);
      }
    } else {
      // Straight trajectory if everything is perfect
      FlyDrone(currentRoll, dronePitch, 0.0);
      isRotated = true;
    }

    // Checks if destination is reached, magic numbers tweakable for precision
    if (Math.abs(correctedX) < 0.75 && Math.abs(correctedZ) < 0.75) {
      isFlyingToLocation = false;
      negBoundary = -1.0;
      posBoundary = 1.0;
      isRotated = false;
      return true;
    } else {
      return false;
    }
  }

  private double CalculateBearingDifference(double targetDegrees) {
    double bearingDiff = 0;
    double currentDegree = getBearingInDegrees();
    if (targetDegrees > currentDegree) {
      if (targetDegrees >= 270.0 && currentDegree <= 90.0) {
        bearingDiff = (360 - targetDegrees + currentDegree);
      } else {
        bearingDiff = -(targetDegrees - currentDegree);
      }
    } else if (currentDegree > targetDegrees) {
      if (currentDegree >= 270.0 && targetDegrees <= 90.0) {
        bearingDiff = -(360 - currentDegree + targetDegrees);
      } else {
        bearingDiff = (currentDegree - targetDegrees);
      }
    }
    return bearingDiff;
  }

  private boolean RotateDroneToDegrees(double targetDegrees) {
    if (Math.round(targetDegrees) != Math.round(getBearingInDegrees())) {
      double bearingDiff = CalculateBearingDifference(targetDegrees);
      bearingDiff = 0.01 * bearingDiff;
      if (bearingDiff < 100)
        bearingDiff = bearingDiff / 2;
      if (!isRotated) {
        FlyDrone(0.0, 0.0, Math.max(-1.0, Math.min(1.0, bearingDiff)));
      }
      return false;
    } else {
      return true;
    }
  }

  public void FlyDrone(double rollDisturbance, double pitchDisturbance, double yawDisturbance) {
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

  public void ToggleHoverDrone(boolean mustHover) {
    if (mustHover) {
      if (!hover) {
        hover = true;
        targetXZ[0] = gps.getValues()[0];
        targetXZ[1] = gps.getValues()[2];
      }
      double xDiff = gps.getValues()[0] - targetXZ[0];
      double yDiff = gps.getValues()[2] - targetXZ[1];
      double bearingDiff;
      if (getBearingInDegrees() > 180) {
        bearingDiff = (getBearingInDegrees() - 360);
      } else {
        bearingDiff = getBearingInDegrees();
      }
      FlyDrone(yDiff, -xDiff, 0.1 * bearingDiff / 10);
    } else {
      hover = false;
    }
  }

  public double[] getCurrentLocation(){
    return gps.getValues();
  }

  private double getBearingInDegrees() {
    double[] north = compass.getValues();
    double rad = Math.atan2(north[0], north[1]);
    double bearing = (rad - 1.5708) / Math.PI * 180.0;
    if (bearing < 0.0)
      bearing = bearing + 360.0;
    return bearing;
  }

  private double[] obstacleDetection() {
    float[] lidarData = lidar.getLayerRangeImage(0);

    boolean ahead = false;
    boolean left = false;
    boolean right = false;
    boolean behind = false;
    boolean objectNear = false;

    for (int i = 0; i < lidarData.length; i++) {
      // Checks if Obstacle is detected for every ray
      if (lidarData[i] < 3.3 && lidarData[i] > 1.2) {
        objectNear = true;
      }

      if (lidarData[i] < 1.0) {
        objectNear = false;
        // Gets direction of detected obstacle from drones perspective
        if (i <= 45 || i > 315) {
          ahead = true;
          continue;
        }
        if (i <= 135) {
          left = true;
          i = 136;
          continue;
        }
        if (i <= 225) {
          behind = true;
          i = 226;
          continue;
        }
        if (i <= 315) {
          right = true;
          i = 316;
          continue;
        }
      }
    }

    // Decision making based on detected obstacles, gives best course of action.
    if (objectNear) {
      return new double[] { 4.0, 0.0 };
    } else {
      int obstacleDirection = turnObstacleDirectionsToInt(ahead, behind, left, right);
      // System.out.println(obstacleDirection);
      return objectAvoidanceMap.get(obstacleDirection);
    }
  }

  private int turnObstacleDirectionsToInt(boolean ahead, boolean behind, boolean left, boolean right){
    int ahead_i = ahead ? 1 : 0;
    int behind_i = behind ? 2 : 0;
    int left_i = left ? 4 : 0;
    int right_i = right ? 8 : 0;
    return ahead_i + behind_i + left_i + right_i;
  }
}