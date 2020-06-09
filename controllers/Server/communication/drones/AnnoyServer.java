package communication.drones;

import communication.command.ICommand;
import communication.command.LocationCommand;
import communication.command.PersonFoundCommand;
import communication.command.ReachedTargetCommand;
import data.DroneData;
import edu.nps.moves.dis7.Vector3Double;
import edu.nps.moves.dis7.EntityStatePdu;
import edu.nps.moves.dis7.EntityID;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Vector;

public class AnnoyServer implements IServer {
    ServerData server;
    ServerCommunicator communicator;
    List<Integer> DroneIDs;
    List<DroneData> droneDataList;
    Map<Integer, Vector3Double> droneTaskMap;

    public AnnoyServer(ServerData s) {
        server = s;
        communicator = new ServerCommunicator(s, "annoy");
        droneDataList = new ArrayList<>();
        droneTaskMap = new HashMap<>();
    }

    public void FindMatchedDrones(int maximum) {
        DroneIDs = communicator.GetMatchedDrones(100, 100 + maximum);
        System.out.print("Drones connected to Annoy Server: ");
        for (int id : DroneIDs) {
            System.out.print(id + ", ");
            droneTaskMap.put(id, null);
        }
        System.out.println(" Total Length: "+DroneIDs.size());
    }

    public void Run() {
        List<ICommand> commandList = communicator.HandleIncomingData();
        // Keep sending target location to drone
        for (ICommand command : commandList) {
            switch (command.getType()){
                case LOCATION:
                    LocationCommand lc = (LocationCommand) command;
                    SetDroneLocation(lc.getLocation(), lc.getChannel());
                    break;
                case PERSON_FOUND:
                    PersonFoundCommand pfc = (PersonFoundCommand) command;
                    // Why tf does annoy server get this?
                    throw new IllegalStateException("Fix your environment if you get this");
                case TARGET_REACHED:
                    ReachedTargetCommand rtc = (ReachedTargetCommand) command;
                    // do something I guess idk
                    break;
            }
        }
        for (Map.Entry<Integer,Vector3Double> entry : droneTaskMap.entrySet()) {
            if(entry.getValue() == null){
                communicator.SendHover(entry.getKey(), true);
            } else {
                Vector3Double tempVec = entry.getValue();
                communicator.SendHover(entry.getKey(), false);
                communicator.SendGoToLocation(entry.getKey(), tempVec.getX(), tempVec.getY(), tempVec.getZ());
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

    public void HandleDisData(EntityStatePdu[] list){
        printPduInformation(list);
        // find drone without a task
        for (EntityStatePdu esp : list){
            if(esp.getForceId() == 0)
                continue;
            boolean freeDrone = false;
            for (Map.Entry<Integer,Vector3Double> entry : droneTaskMap.entrySet()) {
                if(entry.getValue() == null){
                    //Found drone without task, set location in map
                    Vector3Double vec = esp.getEntityLocation();
                    entry.setValue(vec);
                    freeDrone = true;
                    break;
                }
            }
            if(!freeDrone)
                System.out.println("[WARNING] EVERY DRONE ALREADY HAS A TASK");
        }

    }

    private void printPduInformation(EntityStatePdu[] pduList) {
        for (EntityStatePdu esp: pduList) {
            // Do something with the gathered data
            if(esp.getForceId() == 0)
                continue;
            EntityID entityID = esp.getEntityID();
            System.out.println("Timestamp:" + esp.getTimestamp());
            System.out.println("ID:" + entityID.getEntityID());
            System.out.println("Force:" + esp.getForceId());
            Vector3Double vec = esp.getEntityLocation();
            System.out.println("Location: "+vec.getX()+" "+vec.getY()+" "+vec.getZ());
            System.out.println("");
        }
    }

    public List<Vector3Double> GetGroupCoords(){
        return null;
    }
}
