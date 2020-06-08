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

        FlightController fc = new FlightController(robot, timeStep);
        fc.initYolo();

        // Get Devices
        Speaker speaker = robot.getSpeaker("speaker");

        Receiver receiver = robot.getReceiver("receiver");
        Emitter emitter = robot.getEmitter("emitter");

        Camera cam = robot.getCamera("camera");
        cam.enable(timeStep);

        Display display = robot.getDisplay("display");
        display.attachCamera(cam);

        //Targets
        LocationCommand lc = null;
        boolean mustHover = false;

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
            double time = robot.getTime(); // In seconds

            if (time % 0.5 < 0.01) {
                try {
                    // fc.ReturnCoordsOfDetectedHumans(display, cam, fc);
                    // for (double[] humanCoord : fc.ReturnCoordsOfDetectedHumans(display, cam, fc)) {
                    //     dc.SendPersonFound(humanCoord[0], fc.getCurrentLocation()[1], humanCoord[1]);  
                    // }
                } catch (Exception e) {
                // System.out.println("Nothing to detect "+ e);
                }
            }
            if (time < 5) {
                fc.FlyDrone(0, 0, 0);
            } else if (lc != null && !mustHover){
                // System.out.print("Coords: ");
                // System.out.print(lc.getX());
                // System.out.print("   ");
                // System.out.println(lc.getZ());
                fc.FlyDroneToLocation(lc.getX(), lc.getY(), lc.getZ());
            } else {
                fc.ToggleHoverDrone(mustHover);
            }
            
            List<ICommand> commandList = dc.HandleIncomingData();

            // Commands can be in any order
            for (ICommand command : commandList) {
                if(command == null)
                    continue;

                switch (command.getType()) {
                    case HOVER:
                        HoverCommand hc = (HoverCommand) command;
                        System.out.println(hc.isB());
                        mustHover = hc.isB();
                        break;
                    case LOCATION:
                        lc = (LocationCommand) command;
                        System.out.println(lc.getX() + " " +  lc.getY() + " " + lc.getZ());
                        break;
                    default:
                        throw new IllegalArgumentException();
                }
            }

            // Enter here functions to send actuator commands, like:
            if (!speaker.isSpeaking())
                speaker.speak("Fuck off and go home, you bloody idiots", 0.1);

            drone.doSomething(); // Example function
            double[] currentLocation = fc.getCurrentLocation();
            dc.SendLocationToServer(currentLocation[0], currentLocation[1], currentLocation[2]); // TODO Make this work plz, thank you

        }

        // Enter here exit cleanup code.
    }


}
