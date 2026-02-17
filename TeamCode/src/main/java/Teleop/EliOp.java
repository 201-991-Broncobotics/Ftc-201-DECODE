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
    boolean flywheelAutoMode = false;

    // To track where the tag was last seen
    double lastKnownTx = 0;

    boolean lastRB = false;
    boolean lastLB = false;
    boolean lastUp = false, lastDown = false, lastRight = false, lastLeft = false;

    @Override
    public void runOpMode() {

        Settings.aimOffsetScale = 1.1; // RED Offset

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (Settings.numControllers == 1) {
            driveController = gamepad1;
            mechController = gamepad1;
        } else {
            driveController = gamepad1;
            mechController = gamepad2;
        }

        limelight.init(hardwareMap, mechController, 0); // Pipeline 0
        turret.init(hardwareMap, mechController);
        flywheel.init(hardwareMap, mechController, limelight);
        intake.init(hardwareMap, mechController);
        rgb.init(hardwareMap, mechController);
        rgb.setRgb(Settings.rgb_default);

        try { flyMotorReader = hardwareMap.get(DcMotorEx.class, "fly0"); } catch (Exception e) {}
        try { flyMotorReader = hardwareMap.get(DcMotorEx.class, "fly1"); } catch (Exception e) {}


        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                SweMax, telemetry
        );
        drive.zeroModules();
        telemetry.addData("ALLIANCE", "RED");
        waitForStart();
        limelight.start();

        while (opModeIsActive()) {

            // --- CONTROLS ---

            // Right Bumper: Toggle Flywheel Formula Mode
            if (mechController.right_bumper && !lastRB) {
                flywheelAutoMode = !flywheelAutoMode;
            }

            // Left Bumper: Toggle Turret Auto Aim
            if (mechController.left_bumper && !lastLB) {
                turretAutoMode = !turretAutoMode;
            }

            double distance = limelight.getDistance();

            // Formula Calculation
            double formulaRPM = 0;
            if (distance > 0) {
                formulaRPM = (Settings.fly_A * Math.pow(distance, 2)) +
                        (Settings.fly_B * distance) +
                        Settings.fly_C;
            }

             if (flywheelAutoMode) {
                if (distance > 0) Settings.fly_targetRPM = formulaRPM;
                if (mechController.dpad_up || mechController.dpad_down || mechController.dpad_left || mechController.dpad_right) {
                    flywheelAutoMode = false;
                }
            }

            // D-Pad Overrides
            if (mechController.dpad_up && !lastUp) {
                Settings.fly_targetRPM += 50;
                flywheelAutoMode = false;
            }
            if (mechController.dpad_down && !lastDown) {
                Settings.fly_targetRPM -= 50;
                flywheelAutoMode = false;
            }
            if (mechController.dpad_right && !lastRight) {
                Settings.fly_targetRPM = Settings.fly_farPreset;
                flywheelAutoMode = false;
            }
            if (mechController.dpad_left && !lastLeft) {
                Settings.fly_targetRPM = Settings.fly_closePreset;
                flywheelAutoMode = false;
            }

            lastRB = mechController.right_bumper;
            lastLB = mechController.left_bumper;
            lastUp = mechController.dpad_up;
            lastDown = mechController.dpad_down;
            lastRight = mechController.dpad_right;
            lastLeft = mechController.dpad_left;

            // --- TURRET LOGIC ---
            boolean manualTurret = mechController.right_trigger > 0.1 || mechController.left_trigger > 0.1;

            if (manualTurret) {
                turretAutoMode = false;
                turret.controls();
                lastKnownTx = 0; // Reset search if user takes control
            }

            if (turretAutoMode && !manualTurret) {
                LLResult llResult = limelight.getResult();

                if (llResult != null && llResult.isValid()) {
                    // --- TARGET FOUND ---
                    double currentTx = llResult.getTx();
                    double tagYaw = limelight.getYaw();

                    lastKnownTx = currentTx; // Update history

                    double targetOffset = tagYaw * Settings.aimOffsetScale;
                    double error = targetOffset - currentTx;

                    if (Math.abs(error) < Settings.tur_tolerance) {
                        turret.autoTrack(0);
                    } else {
                        limelight.turretPID.setPIDF(Settings.tur_kP, Settings.tur_kI, Settings.tur_kD, 0);
                        double rawPid = limelight.turretPID.calculate(currentTx, targetOffset);
                        double feedforward = Math.signum(rawPid) * Settings.tur_static_F;
                        double finalPower = Range.clip(rawPid + feedforward, -Settings.tur_maxSpeed, Settings.tur_maxSpeed);
                        turret.autoTrack(finalPower);
                    }
                } else {
                    // --- TARGET LOST (SEARCH MODE) ---
                    // FIXED: Only spin if we have seen it at least once
                    if (lastKnownTx == 0) {
                        turret.autoTrack(0); // Stay still, haven't found it yet
                    }
                    else if (lastKnownTx > 0) {
                        turret.autoTrack(-Settings.tur_searchSpeed); // Spin Left (slowly)
                    }
                    else {
                        turret.autoTrack(Settings.tur_searchSpeed);  // Spin Right (slowly)
                    }
                }
            } else if (!manualTurret) {
                turret.autoTrack(0);
            }

            flywheel.controls();
            intake.control();
            drive.drive(driveController.left_stick_y, driveController.left_stick_x, driveController.right_stick_x, 1.0);

            // Telemetry
            double currentRPM = 0;
            if (flyMotorReader != null) {
                double tps = flyMotorReader.getVelocity();
                currentRPM = (tps / Settings.fly_ticksPerRev) * 60.0;
            }

            telemetry.addData("ALLIANCE", "RED");
            telemetry.addData("Turret Mode (LB)", turretAutoMode ? "AUTO" : "MANUAL");
            telemetry.addData("Flywheel Mode (RB)", flywheelAutoMode ? "AUTO" : "MANUAL");
            telemetry.addData("RPM Actual", currentRPM);
            telemetry.addData("RPM Target", Settings.fly_targetRPM);
            telemetry.addData("Last Tx", lastKnownTx);
            telemetry.update();
        }
    }
}