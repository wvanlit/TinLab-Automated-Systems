package communication.drones;

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
        System.out.print("Drones connected to Annoy Server: ");
        for (int id : DroneIDs) {
            if (id != 0)
                System.out.print(id + ", ");
        }
        System.out.println();
    }

    public void Run() {
        communicator.HandleIncomingData();

        communicator.SendRequests();
    }
}
