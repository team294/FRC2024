package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.JeVoisConstants;
import frc.robot.utilities.FileLog;

public class JeVoisCamera extends SubsystemBase {
  private final SerialPort serialPort = new SerialPort(JeVoisConstants.BAUD_RATE, SerialPort.Port.kOnboard);
  private MjpegServer mjpegServer;
  private UsbCamera driveCamera;
  private FileLog log;
  private int logRotationKey;
  private boolean fastLogging = false;

  private int centX, centY;

  public JeVoisCamera(FileLog log, int logRotationKey) {
    this.log = log;
    this.logRotationKey = logRotationKey;

    SmartDashboard.putString("CameraServer Throwable", "OK");
    try {
      // USB drive camera
      driveCamera = CameraServer.startAutomaticCapture();

      // Setting the JeVois resolution will select the corresponding vision module
      driveCamera.setVideoMode(new VideoMode(PixelFormat.kYUYV, JeVoisConstants.width, JeVoisConstants.height, JeVoisConstants.fps)); 
      mjpegServer = new MjpegServer("serve_USB Camera 0", 1181);
    } catch (Throwable t) {
      t.printStackTrace();
      SmartDashboard.putString("CameraServer Throwable", t.toString());
    }

  }

  public int getClosestTargetX() {
    return centX;
  }
  public int getClosestTargetY() {
    return centY;
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */ 
  public void enableFastLogging(boolean enabled) {
    this.fastLogging = enabled;
  }

  public void periodic() {
    String curr = serialPort.readString();

    if(curr.length() >=15 && curr.indexOf("x") == 0){
        curr = curr.substring(0, 15);
        centX = Integer.parseInt(curr.substring(4, 7));
        centY = Integer.parseInt(curr.substring(12, 15));
    }
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      // TODO write log
      log.writeLog(false, "JeVoisCamera", "Periodic", "");
    }
  }
}