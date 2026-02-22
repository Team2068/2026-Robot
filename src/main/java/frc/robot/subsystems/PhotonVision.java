package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  public PhotonCamera frontCamera = new PhotonCamera("frontcamera");
  public PhotonCamera rearCamera = new PhotonCamera("rearcamera");

  public PhotonVision() {
    frontCamera.setDriverMode(true);
    rearCamera.setDriverMode(true);
  }

  public NetworkTable cameraTable(PhotonCamera camera){
    return camera.getCameraTable();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Front Camera Connected", frontCamera.isConnected());
    SmartDashboard.putBoolean("Rear Camera Connected", rearCamera.isConnected());
  }
}
