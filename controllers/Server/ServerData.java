import com.cyberbotics.webots.controller.*;

public class ServerData {
  Robot robot;
  int timeStep;
  
  Emitter emitter;
  Receiver receiver;
  
  public ServerData(Robot robot, int timeStep){
    this.robot = robot;
    this.timeStep = timeStep;
    
    emitter = robot.getEmitter("emitter");
    
    receiver = robot.getReceiver("receiver");
    receiver.enable(timeStep);
    
  }
}
