package commands;

public class HoverCommand implements ICommand{
    private boolean b;
    double y;

    public HoverCommand(boolean b, double y){
        this.b = b;
        this.y = y;
    }

    public boolean isB() {
        return b;
    }

    public double getY(){
        return y;
    }

    public CommandType getType() {
        return CommandType.HOVER;
    }
}
