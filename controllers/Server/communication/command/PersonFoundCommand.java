package communication.command;

import edu.nps.moves.dis7.Vector3Double;

public class PersonFoundCommand implements ICommand {
    private Vector3Double location;
    private int channel;

    public PersonFoundCommand(int channel, double x, double y, double z){
        this.channel = channel;

        location = new Vector3Double();
        location.setX(x);
        location.setY(y);
        location.setZ(z);
    }

    public CommandType getType() {
        return CommandType.PERSON_FOUND;
    }

    public Vector3Double getLocation() {
        return location;
    }

    public int getChannel(){
        return channel;
    }
}
