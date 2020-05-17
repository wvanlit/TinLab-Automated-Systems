import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.Receiver;
import exceptions.NoNewDataException;

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

    public int[] GetMatchedDrones(int start, int end) {
        int[] confirmations = new int[end - start];

        String question = identifier + " type";
        for (int i = start; i < end; i++) {
            // Send Request
            emitter.setChannel(i);
            emitter.send(question.getBytes());

            // Wait for answers
            serverData.robot.step(timeStep);

            // Get Answers
            while (true) {
                try {
                    receiveData(); // Ignore result, as long as there is no exception there is something listening on this channel
                    confirmations[i - start] = i;
                } catch (NoNewDataException nnde) {
                    break;
                }
            }
        }

        return confirmations;
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

