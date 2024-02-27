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
import edu.wpi.first.hal.util.UncleanStatusException;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.StringUtil;

public class JeVoisCamera extends SubsystemBase {
  private SerialPort serialPort, serialPort1, serialPort2;
  // private final SerialPort serialPort = new SerialPort(JeVoisConstants.BAUD_RATE, SerialPort.Port.kOnboard);
  // private MjpegServer mjpegServer;
  // private UsbCamera driveCamera;
  private FileLog log;
  private int logRotationKey;
  private boolean fastLogging = false;

  private int centX = -99, centY = -99;

  public JeVoisCamera(FileLog log) {
    this.log = log;
    this.logRotationKey = log.allocateLogRotation();

    SmartDashboard.putString("CameraServer Throwable", "OK");
    SmartDashboard.putNumber("JeVois cX", centX);
    SmartDashboard.putNumber("JeVois cY", centY);
    // try {
    //   // USB drive camera
    //   driveCamera = CameraServer.startAutomaticCapture();

    //   // Setting the JeVois resolution will select the corresponding vision module
    //   driveCamera.setVideoMode(new VideoMode(PixelFormat.kYUYV, JeVoisConstants.width, JeVoisConstants.height, JeVoisConstants.fps)); 
    //   mjpegServer = new MjpegServer("serve_USB Camera 0", 1181);
    // } catch (Throwable t) {
    //   t.printStackTrace();
    //   SmartDashboard.putString("CameraServer Throwable", t.toString());
    // }

    serialPort = null;
    // TODO Test if we need this or not. In 2020 WPIlib had issues with knowing which usb port was which
    SmartDashboard.putString("Camera Port1", "OK");
    log.writeLogEcho(true, "jevois", "port 1 init");
    try {
      serialPort1 = new SerialPort(115200, SerialPort.Port.kUSB1);
      serialPort = serialPort1;
    } catch (Throwable t) {
      t.printStackTrace();
      SmartDashboard.putString("Camera Port1", t.toString());
      log.writeLogEcho(true, "jevois", "port 1 init", t.toString());
    }

    SmartDashboard.putString("Camera Port2", "OK");
    log.writeLogEcho(true, "jevois", "port 2 init");
    try {
      serialPort2 = new SerialPort(115200, SerialPort.Port.kUSB2);
      if (serialPort == null) serialPort = serialPort2;
    } catch (Throwable t) {
      t.printStackTrace();
      SmartDashboard.putString("Camera Port2", t.toString());
      log.writeLogEcho(true, "jevois", "port 1 init", t.toString());
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
    String curr = "";
    if (serialPort != null) {
      curr = serialPort.readString();
      if (log.isMyLogRotation(logRotationKey))
        System.out.println(StringUtil.buildString("jevois: ", curr));
    }

    if(curr.length() >=10 && curr.indexOf("D3") == 0){
        curr = curr.substring(0, 15);
        centX = Integer.parseInt(curr.substring(3, 6));
        centY = Integer.parseInt(curr.substring(7, 10));
    }
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putNumber("JeVois cX", centX);
      SmartDashboard.putNumber("JeVois cY", centY);
      // TODO write log
      log.writeLog(false, "JeVoisCamera", "Periodic", "cX", centX, "cY", centY);
    }
  }
}