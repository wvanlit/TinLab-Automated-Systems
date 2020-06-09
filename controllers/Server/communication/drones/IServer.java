package communication.drones;

import communication.command.ICommand;
import data.DroneData;
import edu.nps.moves.dis7.Vector3Double;
import edu.nps.moves.dis7.EntityStatePdu;

import java.util.List;
import java.util.Vector;

public interface IServer {
    void Run();

    void FindMatchedDrones(int maximum);

    List<Integer> GetMatchedDrones();

    List<DroneData> GetDrones();

    void SetDroneLocation(Vector3Double location, int channel);

    void HandleDisData(EntityStatePdu[] pduList);

    List<Vector3Double> GetGroupCoords();

}
