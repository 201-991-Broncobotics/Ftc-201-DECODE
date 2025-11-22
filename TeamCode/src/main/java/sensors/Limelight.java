package sensors;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import Diffy.PIDController; // Make sure this import matches your PID file location
import mechanisms.Settings;

public class Limelight {
    private Limelight3A limelight;
    Gamepad Controller;

    // Use the PIDController class you provided
    public final PIDController turretPID = new PIDController(
            Settings.turret_P,
            Settings.turret_I,
            Settings.turret_D,
            () -> 0.0 // Placeholder supplier, we will feed error manually or set target
    );

    public void init(HardwareMap hwdM, Gamepad controller, int pipeline) {
        limelight = hwdM.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.setPollRateHz(100);
        Controller = controller;

        // Configure PID
        turretPID.setTolerance(0.5); // Allow 0.5 degree error
    }

    public void start() {
        limelight.start();
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }

    /**
     * Calculates distance to target in inches based on Limelight Ty (vertical offset).
     */
    public double getDistance(double ty) {
        // formula: d = (h_target - h_cam) / tan(a_mount + ty)
        double angleToGoalDegrees = Settings.limelightMountAngleDegrees + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        double heightDiff = Settings.targetHeightInches - Settings.cameraHeightInches;

        // Avoid divide by zero if looking straight at horizon
        if (Math.tan(angleToGoalRadians) == 0) return 0;

        return heightDiff / Math.tan(angleToGoalRadians);
    }

    /**
     * Returns the ideal flywheel power based on distance using a linear regression.
     */
    public double calculateAutoFlywheelPower(double distanceInches) {
        // Power = Base + (Distance * Factor)
        double targetPower = Settings.FlywheelBasePower + (distanceInches * Settings.FlywheelDistMult);
        return Range.clip(targetPower, 0, 1);
    }
}