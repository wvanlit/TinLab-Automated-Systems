package data;

import edu.nps.moves.dis7.EntityStatePdu;
import edu.nps.moves.dis7.Vector3Double;

public class DroneData {
    public Vector3Double location;
    public int channel;
    public EntityStatePdu entityStatePdu;

    public DroneData(Vector3Double location, int channel, EntityStatePdu entityStatePdu) {
        this.location = location;
        this.channel = channel;
        this.entityStatePdu = entityStatePdu;

        this.entityStatePdu.getEntityID().setEntityID(channel);
        this.entityStatePdu.setEntityLocation(location);
    }

    public void SetDroneLocation(Vector3Double location){
        this.location = location;
        entityStatePdu.setEntityLocation(location);
    }
}
