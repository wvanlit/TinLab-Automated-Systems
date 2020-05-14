import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Emitter;

public class DroneCommunicator {

    private String previousString;
    private final String type;

    Receiver receiver;
    Emitter emitter;

    String identifier;

    public DroneCommunicator(Receiver rec, Emitter em, String type) {
        this.type = type;
        previousString = "";

        receiver = rec;
        emitter = em;

        identifier = "[" + rec.getChannel() + "]";
    }

    public void HandleIncomingData() {
        if (receiver.getQueueLength() <= 0) {
            return;
        }

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

        switch (data) {
            case "type":
                sendToServer(type);
                break;
            default:
                System.out.println("Unknown data received: '" + data + "'");
                break;
        }


    }

    public boolean isFromServer(String data) {
        return data.contains("[" + type + "]");
    }

    public String trimIdentifier(String data) {
        return data.replace("[" + type + "]", "").trim();
    }

    public void sendToServer(String data) {
        emitter.send((identifier + " " + data).getBytes());
        System.out.println("SENDING");
    }
}