package sensors;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import mechanisms.Turret;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class Limelight {
    boolean autotracktoggle, lastbumppressed = false;
    Gamepad Controller;
    private Limelight3A limelight;
    private IMU imu;
    private Turret turret;

    public void init(HardwareMap hwdM, Gamepad controller, int pipeline) {
        limelight = hwdM.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        Controller = controller;


    }

    public void start() {
        limelight.start();
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }
}

