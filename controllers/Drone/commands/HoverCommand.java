package commands;

public class HoverCommand implements ICommand{
    private boolean b;

    public HoverCommand(boolean b){
        this.b = b;
    }

    public boolean isB() {
        return b;
    }

    public CommandType getType() {
        return CommandType.HOVER;
    }
}
