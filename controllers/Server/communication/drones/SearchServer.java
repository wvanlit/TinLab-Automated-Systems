package communication.drones;

import communication.command.ICommand;
import communication.command.LocationCommand;
import communication.command.PersonFoundCommand;
import communication.command.ReachedTargetCommand;
import data.DroneData;
import edu.nps.moves.dis7.Vector3Double;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Queue;

public class SearchServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    List<Integer> DroneIDs;
    List<DroneData> droneDataList;
    List<double[]> checkpoints;
    HashMap<Integer, Integer> currentCheckpoints;  

    public SearchServer(ServerData s) {
        server = s;
        communicator = new ServerCommunicator(s, "search");
        droneDataList = new ArrayList<>();
        currentCheckpoints = new HashMap<>();
        checkpoints = new ArrayList<>();
        checkpoints.add(new double[] {45, 15, 30});
        checkpoints.add(new double[] {-45, 15, 30});
        checkpoints.add(new double[] {-40, 15, -40});
        checkpoints.add(new double[] {40, 15, -40});
    }

    public void FindMatchedDrones(int maximum) {
        int checkpoint = 0;
        DroneIDs = communicator.GetMatchedDrones(0, maximum);
        System.out.print("Drones connected to Search Server: ");
        for (int id : DroneIDs) {
            System.out.print(id + ", ");
            currentCheckpoints.put(id, checkpoint++);
            if (checkpoint > checkpoints.size()-1)
                checkpoint = 0;
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
        for (DroneData drone : droneDataList) {
            int currentCheckpointIndex = currentCheckpoints.get(drone.channel);
            double[] checkpoint = checkpoints.get(currentCheckpointIndex);
            Vector3Double droneLocation = drone.location;
            if (Math.abs(checkpoint[0] - droneLocation.getX()) < 1 && Math.abs(checkpoint[2] - droneLocation.getZ()) < 1){
                System.out.println("Reached");
                currentCheckpointIndex += 1;
                if (currentCheckpointIndex > checkpoints.size()-1){
                    currentCheckpointIndex = 0;
                }
                currentCheckpoints.replace(drone.channel, currentCheckpointIndex);
            }
            checkpoint = checkpoints.get(currentCheckpointIndex);
            communicator.SendGoToLocation(drone.channel, checkpoint[0], checkpoint[1], checkpoint[2]);
            // if(drone.channel == 1){
            //     for (double check : checkpoint) {
            //         System.out.println(check);
            //     }
            //     System.out.println("---");
            // }
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
