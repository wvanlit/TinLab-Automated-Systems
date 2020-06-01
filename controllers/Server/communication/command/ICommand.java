package communication.command;

public interface ICommand {
    CommandType getType();
    int getChannel();
}
