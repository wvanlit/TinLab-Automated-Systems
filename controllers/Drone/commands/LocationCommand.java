package commands;


public class LocationCommand implements ICommand {
    private double x;
    private double y;
    private double z;

    public LocationCommand(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public CommandType getType() {
        return CommandType.LOCATION;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }
}
