package frc.robot.subsystems;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;

public class usbCamera {
    private UsbCamera usbCam = new UsbCamera("USB Camera", 0);
    private MjpegServer mjpegServer1 = new MjpegServer("Server_USB Camera", 1181);
    
}
