package mechanisms;

import static mechanisms.Settings.FlyPower;
import static mechanisms.Settings.closevel;
import static mechanisms.Settings.farvel;
import static mechanisms.Settings.flyVelocity;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

import Diffy.PIDController;


public class Flywheel {

    boolean autotracktoggle, lastbumppressed = false;
    Gamepad Controller;
    private Limelight3A limelight;
    private LLResult results;
    public Position robotPos = new Position();
    private IMU imu;
    private Turret turret;

    boolean flywheeltoggle, lastBPressed;
    double integralSum = 0;
    public DcMotorEx flywheel;

    public double minDifference, minPosition, maxPosition, minPower, maxPower, initialPower, maxSpeed, tolerance, maxIntegral, maxAcceleration, maxDeceleration; // all of these variables can be changed elsewhere in the code
    public double kP, kI, kD, kP2;

    // public PIDController FlywheelPID;



    public void init(HardwareMap hdwMap, Gamepad controller) {

        limelight = hdwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Settings.pipeline);
        results = limelight.getLatestResult();
        flywheel = hdwMap.get(DcMotorEx.class, "flyM");
        Controller = controller;
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // FlywheelPID = new PIDController(0, 0, 0, () -> flywheel.getCurrentPosition());


    }


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
        return ((92.56652 * Math.pow(dist, 2)) - (379.27046 * dist) + 3505.80142);
    }

    public void setFlywheel(double vel) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Settings.flykP, Settings.flykI, Settings.flykD, 11.7));
        flywheel.setVelocity((predictVelocity(getDistance())) *28 / 60);
    }

    public void autoDistance(){
        double predict = getDistance();
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Settings.flykP, Settings.flykI, Settings.flykD, 11.7));
        flywheel.setVelocity((predictVelocity(predict)) *28 / 60);
    }

    public void controls() {
        if (Controller.left_bumper && !lastBPressed) {
            flywheeltoggle = !flywheeltoggle;
        }
        lastBPressed = Controller.left_bumper;
        if (flywheeltoggle) {
            //flywheel.setVelocity(Settings.flyVelocity / 60 * 20); // Set target speed
            flywheel.setVelocity(flyVelocity);
        } else {
            flywheel.setVelocity(0); // Stop flywheel
        }
        if (Controller.dpadUpWasPressed()) {
            autoDistance();
        }
        if (Controller.dpadDownWasPressed()) {
            FlyPower = FlyPower - (0.05 * FlyPower);

        }
        if (Controller.dpad_left) {
            setFlywheel(farvel);
        }
        if (Controller.dpad_right) {
            setFlywheel(closevel);

        }
    }
}