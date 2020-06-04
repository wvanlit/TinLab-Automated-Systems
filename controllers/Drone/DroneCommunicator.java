import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.Receiver;
import commands.HoverCommand;
import commands.ICommand;
import commands.LocationCommand;

import java.util.ArrayList;
import java.util.List;

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

    public List<ICommand> HandleIncomingData() {
        List<ICommand> commands = new ArrayList<>();
        while (receiver.getQueueLength() > 0) {

            String data = new String(receiver.getData());

            if (data.equals(previousString)) {
                break;
            } else {
                previousString = data;
            }

            if (isFromServer(data)) {
                data = trimIdentifier(data);
            } else {
                System.out.println("Data not [" + type + "]");
            }

            String[] split = data.split("\\|");

            commands.add(handleCommand(split[0], split));

        }
        return commands;
    }

    private ICommand handleCommand(String command, String[] parameters) {
        switch (command) {
            case "type":
                sendToServer(type);
                return null;
            case "go_to_location":
                // GO TO XYZ doubles
                return new LocationCommand(Double.parseDouble(parameters[1]),Double.parseDouble(parameters[2]),Double.parseDouble(parameters[3]));
            case "hover":
                // True / False
                return new HoverCommand(Boolean.parseBoolean(parameters[1]));
            default:
                System.out.println("Unknown command received: '" + command + "'");
                return null;
        }
    }


    public void SendLocationToServer(double x, double y, double z){
        sendToServer("location|"+x+"|"+y+"|"+z);
    }

    public void SendTargetReached(boolean b){
        sendToServer("reached_target|"+b);
    }

    public void SendPersonFound(double x, double y, double z){
        sendToServer("found_person|"+x+"|"+y+"|"+z);
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