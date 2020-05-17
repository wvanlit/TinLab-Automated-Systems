import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;
import communication.drones.AnnoyServer;
import communication.drones.IServer;
import communication.drones.SearchServer;
import communication.drones.ServerData;


public class Server {
    static int MAX_ROBOTS_IN_SWARM = 5;

    public static void main(String[] args) {
        Robot robot = new Robot();
        int timeStep = (int) Math.round(robot.getBasicTimeStep());

        Receiver receiver = robot.getReceiver("receiver");

        // Create Server Data
        ServerData sd = new ServerData(robot, timeStep, robot.getEmitter("emitter"), receiver);


        // Get Server Type
        String type = args[0];

        // Create the correct Server Object
        IServer server = null;
        switch (type) {
            case "annoy":
                server = new AnnoyServer(sd);
                break;
            case "search":
                server = new SearchServer(sd);
                break;
            default:
                throw new IllegalArgumentException("Server Input Invalid");
        }

        // Find Matching Drones
        server.FindMatchedDrones(MAX_ROBOTS_IN_SWARM);

        while (robot.step(timeStep) != -1) {
            server.Run();
        }
    }
}

