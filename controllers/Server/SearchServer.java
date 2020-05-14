import com.cyberbotics.webots.controller.Robot;


public class SearchServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    int[] DroneIDs;

    public SearchServer(ServerData s) {
        server = s;
        communicator = new ServerCommunicator(s, "annoy");
    }

    public void FindMatchedDrones(int maximum) {
        //DroneIDs = Server.GetMatchedDrones(0, maximum);
    }

    public void Run() {

    }
}
