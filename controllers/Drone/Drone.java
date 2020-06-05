import com.cyberbotics.webots.controller.*;
import commands.HoverCommand;
import commands.ICommand;
import commands.LocationCommand;

import java.util.List;

public class Drone {

    public static void main(String[] args) {

        // create the Robot instance.
        Robot robot = new Robot();
        int timeStep = (int) Math.round(robot.getBasicTimeStep());

        FlightController fc = new FlightController(robot, tempTimeStep);
        fc.initYolo();

        // Get Devices
        Speaker speaker = robot.getSpeaker("speaker");

        Receiver receiver = robot.getReceiver("receiver");
        Emitter emitter = robot.getEmitter("emitter");

        Camera cam = robot.getCamera("camera");
        cam.enable(timeStep);

        Display display = robot.getDisplay("display");
        display.attachCamera(camera);

        // Get type
        String type = args[0];
        IDrone drone;
        switch (type){
            case "annoy":
                drone = new AnnoyDrone();
                break;
            case "search":
                drone = new SearchDrone();
                break;
            default:
                throw new IllegalArgumentException("Not a valid drone type");
        };

        DroneCommunicator dc = new DroneCommunicator(receiver, emitter, type, timeStep);

        // Main loop:
        // - perform simulation steps until Webots is stopping the controller
        while (robot.step(timeStep) != -1) {
            time = fc.robot.getTime(); // In seconds

            if (time % 0.5 < 0.01) {
                try {

                // fc.DetectHumans(display, camera, fc);
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
                /* Test Set */
                if (!targetOne) {
                // targetOne = fc.FlyDroneToLocation(45, 15, 30);
                targetOne = fc.FlyDroneToLocation(45, 15, 30);
                // targetOne = true;
                } else if (!targetTwo) {
                // targetTwo = fc.FlyDroneToLocation(-45, 15, 30);
                targetTwo = fc.FlyDroneToLocation(-45, 15, 45);
                } else if (!targetThree) {
                // targetThree = fc.FlyDroneToLocation(-40, 15, -40);
                targetThree = fc.FlyDroneToLocation(-40, 15, -40);
                } else if (!targetFour) {
                // targetFour = fc.FlyDroneToLocation(40, 15, -40);
                targetFour = fc.FlyDroneToLocation(40, 15, -40);
                } else {
                // fc.ToggleHoverDrone(true);
                targetOne = false;
                targetTwo = false;
                targetThree = false;
                targetFour = false;
                }
                // */
            }
            /*
            List<ICommand> commandList = dc.HandleIncomingData();

            // Commands can be in any order
            for (ICommand command : commandList) {
                if(command == null)
                    continue;

                switch (command.getType()) {
                    case HOVER:
                        HoverCommand hc = (HoverCommand) command;
                        System.out.println(hc.isB());
                        // Hover or something
                        break;
                    case LOCATION:
                        LocationCommand lc = (LocationCommand) command;
                        System.out.println(lc.getX() + " " +  lc.getY() + " " + lc.getZ());
                        // Go to location
                        break;
                    default:
                        throw new IllegalArgumentException();
                }
            }

            // Enter here functions to send actuator commands, like:
            if (!speaker.isSpeaking())
                speaker.speak("Fuck off and go home, you bloody idiots", 0.1);

            drone.doSomething(); // Example function

            dc.SendLocationToServer(0, 1, 2); // TODO Make this work plz, thank you
            */

        }

        // Enter here exit cleanup code.
    }


}
