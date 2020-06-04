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

        // Get Devices
        Speaker speaker = robot.getSpeaker("speaker");

        Receiver receiver = robot.getReceiver("receiver");
        Emitter emitter = robot.getEmitter("emitter");

        Camera cam = robot.getCamera("camera");
        cam.enable(timeStep);

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
        }

        // Enter here exit cleanup code.
    }


}
