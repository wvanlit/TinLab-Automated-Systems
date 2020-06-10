package communication.drones;

import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.Receiver;

import communication.command.ICommand;
import communication.command.LocationCommand;
import communication.command.PersonFoundCommand;
import communication.command.ReachedTargetCommand;
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
            while (receiver.getQueueLength() > 0) {
                try {
                    // Ignore result, as long as there is no exception there is something listening on this channel
                    if( receiveData().contains(type))
                        drones.add(i);
                } catch (NoNewDataException nnde) {
                    break;
                }
            }
        }
        return drones;
    }

    public List<ICommand> HandleIncomingData() {
        List<ICommand> commands = new ArrayList<>();
        while (receiver.getQueueLength() > 0) {
            try {
                String[] data = receiveData().split("\\|");
                commands.add(handleCommand(data[0], data[1], data));
            } catch (NoNewDataException nnde) {
                break;
            }
        }

        return commands;
    }

    public void SendGoToLocation(int channel, double x, double y, double z){
        emitter.setChannel(channel);
        String s = "location|"+x+"|"+y+"|"+z;
        // System.out.println(s);
        emitter.send(s.getBytes());
    }

    public void SendHover(int channel, boolean b, double targetY){
        emitter.setChannel(channel);
        String s = "hover|"+b+"|"+targetY;
        emitter.send(s.getBytes());
    }



    private String receiveData() throws NoNewDataException {
        if (receiver.getQueueLength() > 0) {
            String data = new String(receiver.getData());
            receiver.nextPacket();
            return data;
        }
        throw new NoNewDataException();
    }

    private ICommand handleCommand(String channel, String command, String[] parameters){
        switch (command) {
            case "location":
                // Drones is at XYZ doubles
                return new LocationCommand(Integer.parseInt(channel),Double.parseDouble(parameters[2]),Double.parseDouble(parameters[3]),Double.parseDouble(parameters[4]));
            case "reached_target":
                // True / False
               return new ReachedTargetCommand(Integer.parseInt(channel), Boolean.parseBoolean(parameters[2]));
            case "found_person":
                // Person is at XYZ doubles
                return new PersonFoundCommand(Integer.parseInt(channel),Double.parseDouble(parameters[2]),Double.parseDouble(parameters[3]),Double.parseDouble(parameters[4]));
            default:
                System.out.println("Unknown command received: '" + command + "'");
                return null;
        }
    }
}

