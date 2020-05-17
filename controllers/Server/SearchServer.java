public class SearchServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    int[] DroneIDs;

    public SearchServer(ServerData s) {
        server = s;
        communicator = new ServerCommunicator(s, "search");
    }

    public void FindMatchedDrones(int maximum) {
        DroneIDs = communicator.GetMatchedDrones(0, maximum);
        System.out.print("Drones connected to Search Server: ");
        for (int id : DroneIDs) {
            if (id != 0)
                System.out.print(id + ", ");
        }
        System.out.println();
    }

    public void Run() {

    }
}
