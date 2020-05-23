import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;
import communication.drones.AnnoyServer;
import communication.drones.IServer;
import communication.drones.SearchServer;
import communication.drones.ServerData;
import communication.servers.DisCommunicator;
import edu.nps.moves.dis7.EntityID;
import edu.nps.moves.dis7.EntityStatePdu;
import edu.nps.moves.dis7.Pdu;
import edu.nps.moves.dis7.Vector3Double;

import java.io.IOException;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.util.List;


public class Server {
    static int MAX_ROBOTS_IN_SWARM = 5;

    public static void main(String[] args) throws IOException {
        if (args.length < 3)
            throw new IllegalArgumentException("Not Enough Arguments");

        Robot robot = new Robot();
        int timeStep = (int) Math.round(robot.getBasicTimeStep());

        Receiver receiver = robot.getReceiver("receiver");

        // Create Server Data
        ServerData sd = new ServerData(robot, timeStep, robot.getEmitter("emitter"), receiver);

        // Get Server Type
        String type = args[0];

        // Create the correct Server Object
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

        // Find Matching Drones
        server.FindMatchedDrones(MAX_ROBOTS_IN_SWARM);

        // Setup DIS
        InetAddress addresses = InetAddress.getByName("127.0.0.1");
        int receivingPort = Integer.parseInt(args[1]);

        int[] ports = new int[args.length - 2];
        for (int i = 2; i < args.length; i++) {
            ports[i - 2] = Integer.parseInt(args[i]);
        }


        DisCommunicator disCommunicator = new DisCommunicator(addresses, ports, receivingPort, timeStep);
        disCommunicator.SetupEntities(server.GetMatchedDrones());

        while (robot.step(timeStep) != -1) {
            server.Run();

            // Dummy Location Gathering
            for (int channel : server.GetMatchedDrones()) {
                Vector3Double vec = new Vector3Double();

                switch (type) {
                    case "annoy":
                        vec.setX(100);
                        break;
                    case "search":
                        vec.setX(1);
                        break;
                }

                vec.setY(channel);
                vec.setZ(channel * 2);

                disCommunicator.SetDroneLocation(vec, channel);
            }

            disCommunicator.SendCurrentData(robot.getTime());

            // Wait for answers
            robot.step(timeStep);
            System.out.println("=== Server " + type + " ===");
            List<EntityStatePdu> pduList = disCommunicator.ReceiveData();
            for (EntityStatePdu esp: pduList) {
                EntityID entityID = esp.getEntityID();
                System.out.println("Timestamp:" + esp.getTimestamp());
                System.out.println("ID:" + entityID.getEntityID());
                Vector3Double vec = esp.getEntityLocation();
                System.out.println("Location: "+vec.getX()+" "+vec.getY()+" "+vec.getZ());

            }
        }

         
    }
}

