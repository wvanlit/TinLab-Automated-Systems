package communication.drones;

import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.Receiver;
import exceptions.NoNewDataException;

import java.util.ArrayList;
import java.util.List;

public class ServerCommunicator {

    int timeStep;
    Receiver receiver;
    Emitter emitter;
    ServerData serverData;
    String type;

    String identifier;

    public ServerCommunicator(ServerData sd, String type) {
        serverData = sd;
        timeStep = sd.timeStep;
        this.type = type;

        emitter = sd.emitter;
        receiver = sd.receiver;

        identifier = "[" + type + "]";

        if (type.equals("search"))
            receiver.setChannel(0);
        else
            receiver.setChannel(100);
    }

    public List<Integer> GetMatchedDrones(int start, int end) {
        List<Integer> drones = new ArrayList<>();
        String question = identifier + " type";
        for (int i = start; i < end; i++) {
            // Send Request
            emitter.setChannel(i);
            emitter.send(question.getBytes());

            // Wait for answers
            serverData.robot.step(timeStep*2);

            // Get Answers
            while (true) {
                try {
                    receiveData(); // Ignore result, as long as there is no exception there is something listening on this channel
                    drones.add(i);
                } catch (NoNewDataException nnde) {
                    break;
                }
            }
        }
        return drones;
    }

    public void HandleIncomingData() {
        while (receiver.getQueueLength() > 0) {
            //Do something with the data
            try {
                String data = receiveData();
            } catch (NoNewDataException nnde) {
                break;
            }

        }
    }

    public void SendRequests() {

    }

    public String receiveData() throws NoNewDataException {
        if (receiver.getQueueLength() > 0) {
            String data = new String(receiver.getData());
            receiver.nextPacket();
            return data;
        }
        throw new NoNewDataException();
    }
}

