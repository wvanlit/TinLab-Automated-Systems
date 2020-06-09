package communication.servers;

import data.DroneData;
import edu.nps.moves.dis7.EntityID;
import edu.nps.moves.dis7.EntityStatePdu;
import edu.nps.moves.dis7.Vector3Double;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

public class DisCommunicator {

    public static final int MAX_PDU_SIZE = 8192;
    int[] destinationPorts;
    DatagramSocket receivingSocket;
    InetAddress address;


    public DisCommunicator(InetAddress addresses, int[] destinationPorts, int port, int timestep) throws IOException {
        this.destinationPorts = destinationPorts;
        this.address = addresses;

        try {
            receivingSocket = new DatagramSocket(port);
            receivingSocket.setSoTimeout(timestep/2);
        } catch (IOException ioe) {
            System.out.println("Exception Thrown while setting up DIS");
            System.out.println(ioe.getMessage());
            throw ioe;
        }
    }

    public void SetupEntities(List<Integer> drones, List<DroneData> droneDataList) {
        for (int droneId : drones) {
            droneDataList.add(
                    createDroneData(droneId, createEntityStatePdu(droneId))
            );
        }
    }

    public void SendCurrentData(double currentTime, List<DroneData> droneDataList) {
        for (DroneData drone : droneDataList) {
            drone.entityStatePdu.setTimestamp((long) (currentTime * 1000));
            sendToEveryAddress(drone.entityStatePdu.marshal());
        }
    }

    public void SendCurrentData(double currentTime, List<DroneData> droneDataList, List<Vector3Double> humanTargets) {
        for (DroneData drone : droneDataList) {
            drone.entityStatePdu.setTimestamp((long) (currentTime * 1000));
            sendToEveryAddress(drone.entityStatePdu.marshal());
        }

        for (Vector3Double target : humanTargets){
            EntityStatePdu entityStatePdu = createEntityStatePdu(200);
            entityStatePdu.setForceId((short) 1);
            entityStatePdu.setEntityLocation(target);
            entityStatePdu.setTimestamp((long) (currentTime * 1000));
            sendToEveryAddress(entityStatePdu.marshal());
        }
    }

    private void sendToEveryAddress(byte[] data) {
        for (int port : destinationPorts) {
            sendPacket(new DatagramPacket(data, data.length, address, port));
        }
    }

    private void sendPacket(DatagramPacket packet) {
        try {
            receivingSocket.send(packet);
        } catch (IOException ioException) {
            System.out.println("Error sending packet: " + ioException.getMessage());
        }
    }

    public List<EntityStatePdu> ReceiveData() {
        List<EntityStatePdu> pduList = new ArrayList<>();
        while (true) {
            try {
                DatagramPacket packet = receivePacket();

                EntityStatePdu pdu = new EntityStatePdu();
                pdu.unmarshal(ByteBuffer.wrap(packet.getData()));

                pduList.add(pdu);
            } catch (SocketTimeoutException e){
                break;
            } catch (IOException ioException) {
                System.out.println("Error receiving packet: " + ioException.getMessage());
                System.out.println(ioException.getClass().getName());
            }
        }
        return pduList;
    }

    private DatagramPacket receivePacket() throws IOException {
        byte[] buffer = new byte[MAX_PDU_SIZE];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
        receivingSocket.receive(packet);
        return packet;
    }

    private DroneData createDroneData(int channel, EntityStatePdu espu) {
        return new DroneData(new Vector3Double(), channel, espu);
    }

    private EntityStatePdu createEntityStatePdu(int id) {
        // Create Base EntityStatePDU
        EntityStatePdu espdu = new EntityStatePdu();
        espdu.setExerciseID((short) 0);
        espdu.setForceId((short) 0);

        // Create Base EntityID
        EntityID eid = espdu.getEntityID();
        eid.setEntityID(id); // EntityID is now equal to the channel
        eid.setSiteID(1);
        eid.setApplicationID(1);

        return espdu;
    }
}
