
package frc.robot.utility;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class tcpHelper {
    private Socket networkSocket;
    private  DataOutputStream dataOut = null;
    private DataInputStream dataIn = null;
    
    public tcpHelper(String address, int port) {
        SmartDashboard.putBoolean("Camera control status", false);
        try {
            networkSocket = new Socket(address, port);

            dataIn = new DataInputStream(System.in);
            dataOut = new DataOutputStream(networkSocket.getOutputStream());

        } catch (IOException e) {
            // TODO: have smart dashboard display an error connecting
            System.out.println("Failed to connect to odroid");
            SmartDashboard.putBoolean("Camera control status", false);
            e.printStackTrace();
        }
    }
    

    /*
     * sends a message to the odroid to switch to the camera number provided
     * @param camNumber currently either a 1 or 2
     */
    public void switchCam(int camNumber) {
        if (camNumber == 1) {
            try {
                dataOut.writeUTF("camera1");
                dataOut.flush();
            } catch (IOException e) {
                System.out.println("encountered exception: " + e);
                SmartDashboard.putBoolean("Camera control status", false);
                e.printStackTrace();
            }
        }
        else if (camNumber == 2) {
            try {
                dataOut.writeUTF("camera2");
                dataOut.flush();
            } catch (IOException e) {
                System.out.println("encountered exception: " + e);
                SmartDashboard.putBoolean("Camera control status", false);
                e.printStackTrace();
            }
        }
    }

    public void closeConnection() {
        try {
            dataIn.close();
            dataOut.close();
            networkSocket.close();
        } catch (IOException e) {
            System.out.println("Failed to close connection! error: " + e);
            SmartDashboard.putBoolean("Camera control status", false);
            e.printStackTrace();
        }
    }
}