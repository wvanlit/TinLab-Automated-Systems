package communication.drones;

import communication.command.ICommand;
import communication.command.LocationCommand;
import communication.command.PersonFoundCommand;
import communication.command.ReachedTargetCommand;
import data.DroneData;
import edu.nps.moves.dis7.Vector3Double;
import edu.nps.moves.dis7.EntityStatePdu;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.List;
import java.util.Queue;
import java.util.Vector;


public class SearchServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    List<Integer> DroneIDs;
    List<DroneData> droneDataList;
    List<double[]> checkpoints;
    HashMap<Integer, Integer> currentCheckpoints;  
    List<Vector3Double> groupCoords;
    static int MAXGROUPSIZE = 3;

    public SearchServer(ServerData s) {
        server = s;
        
        communicator = new ServerCommunicator(s, "search");
        droneDataList = new ArrayList<>();
        currentCheckpoints = new HashMap<>();
        groupCoords = new ArrayList<>();
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
        HashMap<Integer, List<Vector3Double>> personsFoundMap = new HashMap<>();
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
                    int droneChannel = pfc.getChannel();
                    // Check if channel is in map and has a list associated with it
                    if(!personsFoundMap.containsKey(droneChannel)){
                        personsFoundMap.put(droneChannel, new ArrayList<Vector3Double>());
                    }
                    // Add location to map
                    List<Vector3Double> persons = personsFoundMap.get(droneChannel);
                    persons.add(pfc.getLocation());
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
                currentCheckpointIndex += 1;
                if (currentCheckpointIndex > checkpoints.size()-1){
                    currentCheckpointIndex = 0;
                }
                currentCheckpoints.replace(drone.channel, currentCheckpointIndex);
            }
            checkpoint = checkpoints.get(currentCheckpointIndex);
            communicator.SendGoToLocation(drone.channel, checkpoint[0], checkpoint[1], checkpoint[2]);
        }
        for (Map.Entry<Integer, List<Vector3Double>> entry : personsFoundMap.entrySet()) {
            System.out.print(entry.getKey()+" ");
            List<Vector3Double> listValue = entry.getValue();
            if (listValue.size() < MAXGROUPSIZE) {
                continue;
            }
            double sumXCoord = 0;
            double sumZCoord = 0;
            double yCoord = 15;
            for (Vector3Double vector : listValue) {
                sumXCoord += vector.getX();
                sumZCoord += vector.getZ();
                yCoord = vector.getY(); // Doesnt matter, Y should always be 15
            }
            double avgXCoord = sumXCoord / listValue.size();
            double avgZCoord = sumZCoord / listValue.size();
            //check if coords already exist as a marked group
            boolean isDuplicate = false;
            for (Vector3Double vector : groupCoords) {
                boolean xIsClose = coordIsClose(avgXCoord, vector.getX(), 0.75);
                boolean zIsClose = coordIsClose(avgZCoord, vector.getZ(), 0.75);
                if(xIsClose && zIsClose){
                    isDuplicate = true;
                    break;
                }
            }
            if (!isDuplicate){
                Vector3Double tempVec = new Vector3Double();
                tempVec.setX(avgXCoord);
                tempVec.setY(yCoord);
                tempVec.setZ(avgZCoord);
                groupCoords.add(tempVec);
            }
		}
    }

    public boolean coordIsClose(double v1, double v2, double threshHold){
        return (Math.abs(v1 - v2) > threshHold);
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
    
    public void HandleDisData(EntityStatePdu[] list){
        // System.out.println("[Search] Shouldnt be here");
    }

    public List<Vector3Double> GetGroupCoords(){
        return groupCoords;
    }
}
