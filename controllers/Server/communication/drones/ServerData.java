package communication.drones;

import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;

public class ServerData {
    Robot robot;
    int timeStep;

    Emitter emitter;
    Receiver receiver;

    public ServerData(Robot robot, int timeStep, Emitter emitter, Receiver receiver) {
        this.robot = robot;
        this.timeStep = timeStep;
        this.emitter = emitter;
        this.receiver = receiver;

        receiver.enable(timeStep);
    }
}
