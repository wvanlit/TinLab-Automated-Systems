package communication.drones;

import java.util.List;

public class AnnoyServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    List<Integer> DroneIDs;

    public AnnoyServer(ServerData s) {
        server = s;
        communicator = new ServerCommunicator(s, "annoy");
    }

    public void FindMatchedDrones(int maximum) {
        DroneIDs = communicator.GetMatchedDrones(100, 100 + maximum);
        System.out.print("Drones connected to Annoy Server: ");
        for (int id : DroneIDs) {
            System.out.print(id + ", ");
        }
        System.out.println(" Total Length: "+DroneIDs.size());
    }

    public void Run() {
        communicator.HandleIncomingData();
    }

    public List<Integer> GetMatchedDrones() {
        return DroneIDs;
    }
}
