package communication.drones;

import java.util.List;

public interface IServer {
    void Run();

    void FindMatchedDrones(int maximum);

    List<Integer> GetMatchedDrones();
}
