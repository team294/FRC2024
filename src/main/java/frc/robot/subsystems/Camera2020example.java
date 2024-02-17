// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.cscore.VideoMode;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.SerialPort;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Camera extends SubsystemBase {

//   // This works for Rio bottom USB port = JeVois serial port, top USB port = empty
//   // private final SerialPort cam = new SerialPort(115200, SerialPort.Port.kUSB);

//   // This works for Rio bottom USB port = JeVois serial port, top USB port = JeVois mini-USB port
//   // private final SerialPort cam = new SerialPort(115200, SerialPort.Port.kUSB1);
//   private SerialPort cameraSerialPort, cameraSerialPort1, cameraSerialPort2;
//   private UsbCamera driveCamera;

//   /**
//    * Creates a new Camera.
//    */
//   public Camera() {

//     // Send camera video stream to Shuffleboard
//     SmartDashboard.putString("CameraServer Throwable", "OK");
//     try {
//       // USB drive camera
//       driveCamera = CameraServer.getInstance().startAutomaticCapture();

//       // Setting the JeVois resolution will select the corresponding vision module
//       driveCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 640, 256, 60); 
//     } catch (Throwable t) {
//       t.printStackTrace();
//       SmartDashboard.putString("CameraServer Throwable", t.toString());
//     }

//     // There was some issues with the RoboRio camera driver in 2018 WPILib.  Don't use this code until the driver is fixed?
// //		driveCamera.setExposureAuto(); // Start in auto exposure mode so that we can set brightness 
// //		driveCamera.setBrightness(30); // Setting brightness only works correctly in auto exposure mode (?)  was 10
// //		driveCamera.getProperty("contrast").set(80);
// //		driveCamera.getProperty("saturation").set(60); 
// //		driveCamera.setExposureManual(20);
// //		driveCamera.setWhiteBalanceManual(2800);

//     cameraSerialPort = null;

//     SmartDashboard.putString("Camera Port1", "OK");
//     try {
//       cameraSerialPort1 = new SerialPort(115200, SerialPort.Port.kUSB1);
//       cameraSerialPort = cameraSerialPort1;
//     } catch (Throwable t) {
//       t.printStackTrace();
//       SmartDashboard.putString("Camera Port1", t.toString());
//     }

//     SmartDashboard.putString("Camera Port2", "OK");
//     try {
//       cameraSerialPort2 = new SerialPort(115200, SerialPort.Port.kUSB2);
//       if (cameraSerialPort == null) cameraSerialPort = cameraSerialPort2;
//     } catch (Throwable t) {
//       t.printStackTrace();
//       SmartDashboard.putString("Camera Port2", t.toString());
//     }

//     // In the JeVois python code, "jevois.sendSerial()" sends data to the serial output stream.
//     // JeVois commands to determine where the serial output stream goes (capitalization must be exact!):
//     //    setpar serout Hard = 4-pin serial port
//     //    setpar serout USB = mini-USB port
//     //    setpar serout None = None
  
//     // Enable serial output (or put this in the JeVois module's "script.cfg" file instead)
//     // cam.writeString("setpar serout Hard\n");
    
//   }

//   @Override
//   public void periodic() {
//     String curr = cameraSerialPort.readString();

//     if(curr.length() >=15 && curr.indexOf("x") == 0){
//         curr = curr.substring(0, 15);
//         int centX = Integer.parseInt(curr.substring(4, 7));
//         int centY = Integer.parseInt(curr.substring(12, 15));

//         // System.out.println("Camera data = " + curr);

//         // RobotContainer.log.writeLogEcho(false, "Camera", "Periodic", curr);
//     }

//   }
// }
