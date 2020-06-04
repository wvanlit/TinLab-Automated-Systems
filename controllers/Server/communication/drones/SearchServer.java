package communication.drones;

import communication.command.ICommand;
import communication.command.LocationCommand;
import communication.command.PersonFoundCommand;
import communication.command.ReachedTargetCommand;
import data.DroneData;
import edu.nps.moves.dis7.Vector3Double;

import java.util.ArrayList;
import java.util.List;

public class SearchServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    List<Integer> DroneIDs;
    List<DroneData> droneDataList;

    public SearchServer(ServerData s) {
        server = s;
        communicator = new ServerCommunicator(s, "search");
        droneDataList = new ArrayList<>();
    }

    public void FindMatchedDrones(int maximum) {
        DroneIDs = communicator.GetMatchedDrones(0, maximum);
        System.out.print("Drones connected to Search Server: ");
        for (int id : DroneIDs) {
            System.out.print(id + ", ");
        }
        System.out.println(" Total Length: " + DroneIDs.size());
    }

    public void Run() {
        List<ICommand> commandList = communicator.HandleIncomingData();

        for (ICommand command : commandList) {
            switch (command.getType()){
                case LOCATION:
                    LocationCommand lc = (LocationCommand) command;
                    SetDroneLocation(lc.getLocation(), lc.getChannel());
                    break;
                case PERSON_FOUND:
                    PersonFoundCommand pfc = (PersonFoundCommand) command;
                    // Make some kind of Person Found list or smthing
                    break;
                case TARGET_REACHED:
                    ReachedTargetCommand rtc = (ReachedTargetCommand) command;
                    // do something I guess idk
                    break;
            }
        }
    }

    public List<Integer> GetMatchedDrones() {
        return DroneIDs;
    }

    public List<DroneData> GetDrones() {
        return droneDataList;
    }

    public void SetDroneLocation(Vector3Double location, int channel) {
        for (DroneData drone : droneDataList) {
            if (drone.channel != channel)
                continue;
            drone.SetDroneLocation(location);
            break;
        }
    }

}
