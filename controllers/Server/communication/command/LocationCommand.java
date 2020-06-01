package communication.command;

import edu.nps.moves.dis7.Vector3Double;

public class LocationCommand implements ICommand {
    private Vector3Double location;
    private int channel;

    public LocationCommand(int channel, double x, double y, double z){
        this.channel = channel;

        location = new Vector3Double();
        location.setX(x);
        location.setY(y);
        location.setZ(z);
    }

    public CommandType getType() {
        return CommandType.LOCATION;
    }

    public Vector3Double getLocation() {
        return location;
    }

    public int getChannel(){
        return channel;
    }
}
