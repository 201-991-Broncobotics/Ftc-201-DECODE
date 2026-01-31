package Teleop;

import static mechanisms.Settings.SweMax;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import Diffy.DiffySwerveKinematics;
import mechanisms.Flywheel;
import mechanisms.Intake;
import mechanisms.Settings;
import mechanisms.Turret;
import sensors.Limelight;
import sensors.RGB;

@TeleOp(name = "Eli Op", group = "Concept")
public class EliOp extends LinearOpMode {
    DiffySwerveKinematics drive;
    Gamepad mechController;
    Gamepad driveController;

    mechanisms.Turret turret = new mechanisms.Turret();
    Flywheel flywheel = new Flywheel();
    Intake intake = new Intake();
    RGB rgb = new RGB();
    private Limelight limelight = new Limelight();
    private DcMotorEx flyMotorReader;

    boolean turretAutoMode = false;
    boolean lastY = false;
    boolean lastUp = false, lastDown = false, lastRight = false, lastLeft = false;

    @Override
    public void runOpMode() {

        // --- IMPORTANT: Enable Dashboard Telemetry ---
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Controller Setup
        if (Settings.numControllers == 1) {
            driveController = gamepad1;
            mechController = gamepad1;
        } else {
            driveController = gamepad1;
            mechController = gamepad2;
        }

        // Hardware Init
        limelight.init(hardwareMap, mechController, 0);
        turret.init(hardwareMap, mechController);
        flywheel.init(hardwareMap, mechController, limelight);
        intake.init(hardwareMap, mechController);
        rgb.init(hardwareMap, mechController);
        rgb.setRgb(1);

        try { flyMotorReader = hardwareMap.get(DcMotorEx.class, "flyM"); } catch (Exception e) {}

        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                SweMax, telemetry
        );
        drive.zeroModules();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {

            // --- 1. FLYWHEEL CONTROL ---
            // Manual adjustments via D-Pad (Modifies the Dashboard variable directly)
            if (mechController.dpad_up && !lastUp) {
                Settings.fly_targetRPM += 50;
            }
            if (mechController.dpad_down && !lastDown) {
                Settings.fly_targetRPM -= 50;
            }
            if (mechController.dpad_right && !lastRight) {
                Settings.fly_targetRPM = Settings.fly_farPreset;
            }
            if (mechController.dpad_left && !lastLeft) {
                Settings.fly_targetRPM = Settings.fly_closePreset;
            }

            // Toggle Auto Turret with Y
            if (mechController.y && !lastY) {
                turretAutoMode = !turretAutoMode;
            }

            // Get Distance (Fixes fluctuation if Settings.CAMERA_OFFSET is tuned)
            double distance = limelight.getDistance();

            // --- CALCULATE FORMULA RPM (For Graphing/Testing) ---
            // We calculate this so you can see it on the graph, but we don't force the motor
            // to this speed unless you add code to do so. This helps tuning.
            double formulaRPM = 0;
            if (distance > 0) {
                formulaRPM = (Settings.fly_A * Math.pow(distance, 2)) +
                        (Settings.fly_B * distance) +
                        Settings.fly_C;
            }

            // Update button states
            lastUp = mechController.dpad_up;
            lastDown = mechController.dpad_down;
            lastRight = mechController.dpad_right;
            lastLeft = mechController.dpad_left;
            lastY = mechController.y;

            // --- 2. TURRET LOGIC ---
            boolean manualTurret = mechController.right_trigger > 0.1 || mechController.left_trigger > 0.1;

            if (manualTurret) {
                turretAutoMode = false;
                turret.controls();
            }

            if (turretAutoMode && !manualTurret) {
                LLResult llResult = limelight.getResult();
                if (llResult != null && llResult.isValid()) {

                    double currentTx = llResult.getTx();
                    double tagYaw = limelight.getYaw();

                    double targetOffset = tagYaw * Settings.aimOffsetScale;
                    double error = targetOffset - currentTx;

                    if (Math.abs(error) < Settings.tur_tolerance) {
                        turret.autoTrack(0);
                    } else {
                        // PID Control
                        limelight.turretPID.setPIDF(Settings.tur_kP, Settings.tur_kI, Settings.tur_kD, 0);
                        double rawPid = limelight.turretPID.calculate(currentTx, targetOffset);

                        // Static Feedforward (Power to overcome friction)
                        double feedforward = Math.signum(rawPid) * Settings.tur_static_F;

                        double finalPower = Range.clip(rawPid + feedforward, -Settings.tur_maxSpeed, Settings.tur_maxSpeed);
                        turret.autoTrack(finalPower);
                    }
                } else {
                    turret.autoTrack(0);
                }
            } else if (!manualTurret) {
                turret.autoTrack(0);
            }

            // --- 3. OTHER MECHANISMS ---
            flywheel.controls(); // Runs the flywheel at Settings.fly_targetRPM
            intake.control();    // Handles A/X (Manual) and B (Auto Sequence)
            drive.drive(driveController.left_stick_y, driveController.left_stick_x, driveController.right_stick_x, 1.0);

            // --- 4. TELEMETRY (GRAPHING) ---
            double currentRPM = 0;
            if (flyMotorReader != null) {
                double tps = flyMotorReader.getVelocity();
                currentRPM = (tps / Settings.fly_ticksPerRev) * 60.0;
            }

            // Graphing Bounds (Check these in Dashboard to lock Y-axis)
            telemetry.addData("Limit 6000", 6000);
            telemetry.addData("Limit 0", 0);

            // Live Data (Raw doubles for graphing)
            telemetry.addData("RPM Actual", currentRPM);
            telemetry.addData("RPM Target", Settings.fly_targetRPM);
            telemetry.addData("RPM Formula", formulaRPM);

            telemetry.addData("Dist (in)", distance);
            telemetry.addData("Turret Mode", turretAutoMode ? "AUTO" : "MANUAL");

            telemetry.update();
        }
    }
}