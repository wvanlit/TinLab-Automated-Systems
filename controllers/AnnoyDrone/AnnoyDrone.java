import com.cyberbotics.webots.controller.*;

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

        Receiver receiver = robot.getReceiver("receiver");
        Emitter emitter = robot.getEmitter("emitter");

        Camera cam = robot.getCamera("camera");
        cam.enable(timeStep);

        DroneCommunicator dc = new DroneCommunicator(receiver, emitter, "annoy", timeStep);

        // Main loop:
        // - perform simulation steps until Webots is stopping the controller
        while (robot.step(timeStep) != -1) {
            dc.HandleIncomingData();
            // Enter here functions to send actuator commands, like:
            if (!speaker.isSpeaking())
                speaker.speak("Fuck off and go home, you bloody idiots", 0.1);
        }

        // Enter here exit cleanup code.
    }


}
