import com.cyberbotics.webots.controller.Robot;
import communication.drones.AnnoyServer;
import communication.drones.IServer;
import communication.drones.SearchServer;
import communication.drones.ServerData;
import communication.servers.DisCommunicator;
import data.DroneData;
import edu.nps.moves.dis7.EntityID;
import edu.nps.moves.dis7.EntityStatePdu;
import edu.nps.moves.dis7.Vector3Double;

import java.io.IOException;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;


public class Server {
    static int MAX_ROBOTS_IN_SWARM = 5;

    public static void main(String[] args) throws IOException {
        if (args.length < 3)
            throw new IllegalArgumentException("Not Enough Arguments");

        Robot robot = new Robot();
        int timeStep = (int) Math.round(robot.getBasicTimeStep());

        // Create Server Data
        ServerData sd = new ServerData(robot, timeStep, robot.getEmitter("emitter"), robot.getReceiver("receiver"));

        // Get Server Type
        String type = args[0];
        IServer server = getServer(sd, type);

        // Find Matching Drones
        server.FindMatchedDrones(MAX_ROBOTS_IN_SWARM);



        // Setup DIS
        InetAddress addresses = InetAddress.getByName("127.0.0.1");
        int receivingPort = Integer.parseInt(args[1]);

        int[] ports = getPorts(args);

        DisCommunicator disCommunicator = new DisCommunicator(addresses, ports, receivingPort, timeStep);
        disCommunicator.SetupEntities(server.GetMatchedDrones(), server.GetDrones());

        while (robot.step(timeStep) != -1) {
            server.Run(); // Get messages and handle them

            List<Vector3Double> humanTargets = new ArrayList<>();
            disCommunicator.SendCurrentData(robot.getTime(), server.GetDrones(), humanTargets);
            // Wait for answers
            robot.step(timeStep);

            List<EntityStatePdu> pduList = disCommunicator.ReceiveData();
            printPduInformation(type, pduList);
        }
    }

    private static void printPduInformation(String type, List<EntityStatePdu> pduList) {
        System.out.println("=== Server " + type + " ===");
        for (EntityStatePdu esp: pduList) {
            // Do something with the gathered data
            EntityID entityID = esp.getEntityID();
            System.out.println("Timestamp:" + esp.getTimestamp());
            System.out.println("ID:" + entityID.getEntityID());
            System.out.println("Force:" + esp.getForceId());
            Vector3Double vec = esp.getEntityLocation();
            System.out.println("Location: "+vec.getX()+" "+vec.getY()+" "+vec.getZ());
            System.out.println("");
        }
    }

    private static IServer getServer(ServerData sd, String type) {
        IServer server;
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
        return server;
    }

    private static int[] getPorts(String[] args) {
        int[] ports = new int[args.length - 2];
        for (int i = 2; i < args.length; i++) {
            ports[i - 2] = Integer.parseInt(args[i]);
        }
        return ports;
    }
}



