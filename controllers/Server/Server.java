import com.cyberbotics.webots.controller.Robot;


public class Server {
  static int MAX_ROBOTS_IN_SWARM = 10;


  public static void main(String[] args) {
    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Create Server Data
    ServerData sd = new ServerData(robot, timeStep);
    
    // Get Server Type
    String type = args[0];
    System.out.println(type);
    
    // Create the correct Server Object
    IServer server = null;
    switch(type){
      case "annoy":
        server = new AnnoyServer(sd);
        break;
      case "search":
        server = new SearchServer(sd);
        break;
      default:
        throw new IllegalArgumentException("Server Input Invalid");
    }
    
    while (robot.step(timeStep) != -1) {
      server.Run();
    };
  }
  
  public static boolean[] GetMatchedDrones(int start, int end, String confirmation){
    boolean[] confirmations = new boolean[end-start];
    //TODO
    return confirmations;
  }
  
}

