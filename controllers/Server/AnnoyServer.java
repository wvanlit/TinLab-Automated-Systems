import com.cyberbotics.webots.controller.Robot;


public class AnnoyServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    int[] DroneIDs;

    public AnnoyServer(ServerData s) {
        server = s;
        communicator = new ServerCommunicator(s, "annoy");
    }

    public void FindMatchedDrones(int maximum) {
        DroneIDs = communicator.GetMatchedDrones(100, 100 + maximum);
        for ( int id : DroneIDs) {
            System.out.print(id+" ");
        }
        System.out.println("!");
    }

    public void Run() {

    }
}
