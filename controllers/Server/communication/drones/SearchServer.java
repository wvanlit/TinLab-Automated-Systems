package communication.drones;

import java.util.List;

public class SearchServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    List<Integer> DroneIDs;

    public SearchServer(ServerData s) {
        server = s;
        communicator = new ServerCommunicator(s, "search");
    }

    public void FindMatchedDrones(int maximum) {
        DroneIDs = communicator.GetMatchedDrones(0, maximum);
        System.out.print("Drones connected to Search Server: ");
        for (int id : DroneIDs) {
            System.out.print(id + ", ");
        }
        System.out.println(" Total Length: " + DroneIDs.size());
    }

    public void Run() {
        communicator.HandleIncomingData();
    }

    public List<Integer> GetMatchedDrones() {
        return DroneIDs;
    }
}
