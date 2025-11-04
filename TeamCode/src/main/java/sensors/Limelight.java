package sensors;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Limelight {
    private Limelight3A limelight;
    private IMU imu;
    public  void init(HardwareMap hwdM){
        limelight =hwdM.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        imu = hwdM.get(IMU.class, "llimu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));



    }
}
