package communication.command;

import edu.nps.moves.dis7.Vector3Double;

public class ReachedTargetCommand implements ICommand {
    private boolean b;
    private int channel;

    public ReachedTargetCommand(int channel, boolean b){
        this.channel = channel;

        this.b = b;
    }

    public CommandType getType() {
        return CommandType.TARGET_REACHED;
    }

    public int getChannel(){
        return channel;
    }

    public boolean isB() {
        return b;
    }
}
