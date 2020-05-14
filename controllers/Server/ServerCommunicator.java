import com.cyberbotics.webots.controller.*;
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
    }

    public int[] GetMatchedDrones(int start, int end) {
        int[] confirmations = new int[end - start];

        String question = identifier + " type";
        for (int i = start; i < end; i++) {
            // Send Request
            emitter.setChannel(i);
            emitter.send(question.getBytes());

            System.out.println("Sending '" + question + "' to #" + emitter.getChannel());

            receiver.setChannel(i);

            // Wait
            serverData.robot.step(timeStep * 2);

            try {
                String data = receiveData(i);
                System.out.println("Received '" + data + "' from " + i);
                confirmations[i - start] = i;
            } catch (NoNewDataException noNewDataException) {
                continue;
            }
        }

        return confirmations;
    }

    private String previousString = "";

    public String receiveData(int channel) throws NoNewDataException {
        if (receiver.getQueueLength() > 0) {
            String data = new String(receiver.getData());
            if (!data.equals(previousString)) {
                System.out.println("Received '" + data + "' from " + channel);
                previousString = data;
                return data;
            }

        }

        throw new exceptions.NoNewDataException();
    }

    public boolean isFrom(String data, int channel) {
        return data.contains("[" + channel + "]");
    }
}

