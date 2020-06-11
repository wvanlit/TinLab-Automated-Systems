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
        HoverCommand hc = new HoverCommand(false, 5);
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
                    if(type.equals("search")){
                        for (double[] humanCoord : fc.ReturnCoordsOfDetectedHumans(display, cam, fc)) {
                            dc.SendPersonFound(humanCoord[0], fc.getCurrentLocation()[1], humanCoord[1]);  
                        }
                    }
                } catch (Exception e) {
                // System.out.println("Nothing to detect "+ e);
                }
            }
            
            if (time < 5) {
                fc.FlyDrone(0, 0, 0);
            } else if (lc != null && !hc.isB()){
                fc.ToggleHoverDrone(false, hc.getY());
                fc.FlyDroneToLocation(lc.getX(), lc.getY(), lc.getZ());
            } else if(lc == null){
                fc.ToggleHoverDrone(true, hc.getY());
            } 
            else{
                fc.ToggleHoverDrone(false, hc.getY());
                fc.FlyDroneToLocation(lc.getX(), hc.getY(), lc.getZ());                
            }
            
            List<ICommand> commandList = dc.HandleIncomingData();

            // Commands can be in any order
            for (ICommand command : commandList) {
                if(command == null)
                    continue;

                switch (command.getType()) {
                    case HOVER:
                        hc = (HoverCommand) command;              
                        break;
                    case LOCATION:
                        lc = (LocationCommand) command;
                        break;
                    default:
                        throw new IllegalArgumentException();
                }
            }

            if (!speaker.isSpeaking() && type.equals("annoy"))
                speaker.speak("Please return to your homes", 0.5);
                
            double[] currentLocation = fc.getCurrentLocation();
            dc.SendLocationToServer(currentLocation[0], currentLocation[1], currentLocation[2]);

        }
    }


}
