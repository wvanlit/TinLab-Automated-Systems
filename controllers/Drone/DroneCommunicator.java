import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.Receiver;

public class DroneCommunicator {

    private final String type;
    Receiver receiver;
    Emitter emitter;
    private String previousString;

    public DroneCommunicator(Receiver rec, Emitter em, String type, int timeStep) {
        this.type = type;
        previousString = "";

        receiver = rec;
        receiver.enable(timeStep);
        // Receiver's channel is set in the object properties

        emitter = em;
        if (type.equals("search"))
            emitter.setChannel(0);
        else
            emitter.setChannel(100);
    }

    public void HandleIncomingData() {
        while (receiver.getQueueLength() > 0) {

            String data = new String(receiver.getData());

            if (data.equals(previousString)) {
                return;
            } else {
                previousString = data;
            }

            if (isFromServer(data)) {
                data = trimIdentifier(data);
            } else {
                System.out.println("Data not [" + type + "]");
            }

            handleCommand(data);

        }
    }

    private void handleCommand(String data) {
        switch (data) {
            case "type":
                sendToServer(type);
                break;
            default:
                System.out.println("Unknown data received: '" + data + "'");
                break;
        }
    }

    private boolean isFromServer(String data) {
        return data.contains("[" + type + "]");
    }

    private String trimIdentifier(String data) {
        return data.replace("[" + type + "]", "").trim();
    }

    private void sendToServer(String data) {
        emitter.send((receiver.getChannel() + "|" + data).getBytes());
    }
}