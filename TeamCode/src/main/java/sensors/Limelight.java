package sensors;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import mechanisms.Settings;
import mechanisms.Turret;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


public class Limelight {
    boolean autotracktoggle, lastbumppressed = false;
    Gamepad Controller;
    private Limelight3A limelight;
    private LLResult results;
    public Position robotPos = new Position();
    private IMU imu;
    private Turret turret;


    public void init(HardwareMap hwdM, Gamepad controller, int pipeline) {
        limelight = hwdM.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        results = limelight.getLatestResult();
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        Controller = controller;

    }

    public void start() {
        limelight.start();
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    public final PIDFController turretPID = new PIDFController(
            Settings.turret_P,
            Settings.turret_I,
            Settings.turret_D,
            0.0);

    public double getDistance() {
        results = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fids = results.getFiducialResults();

        for (LLResultTypes.FiducialResult f : fids) {
            int id = f.getFiducialId();
            Pose3D pos = f.getTargetPoseRobotSpace();
            robotPos = pos.getPosition();

            return Math.hypot(robotPos.x, robotPos.z);

        }
        if (results.getFiducialResults().isEmpty()) {
            return 0; }

        if (fids.isEmpty()) {
            return 0; // or -1
            }

        return 0;

    }
    public double predictVelocity(double dist) {
        results = limelight.getLatestResult();
        if (dist != 0) {
            return (92.56652 * Math.pow(dist, 2)) - (379.27046 * dist) + 3505.80142;}
        return 0;
    }
}

