package Teleop;

import static mechanisms.Settings.SweMax;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor; // Import this

import mechanisms.Flywheel;
import mechanisms.Intake;
import mechanisms.Settings;
import mechanisms.Turret;
import sensors.Limelight;
import sensors.RGB;
import Diffy.DiffySwerveKinematics;

@TeleOp(name = "Eli Op Final", group = "Concept")
public class EliOp extends LinearOpMode {

    // Mechanisms
    DiffySwerveKinematics drive;
    Turret turret = new Turret();
    Flywheel flywheel = new Flywheel();
    Intake intake = new Intake();
    RGB rgb = new RGB();
    Limelight limelight = new Limelight();

    // Inputs & State
    Gamepad operator, driver;
    boolean autoTrackToggle = false;
    boolean lastR3Pressed = false;

    // Logic State
    boolean isRedAlliance = true;
    boolean isShooting = false;

    @Override
    public void runOpMode() {
        operator = gamepad2;
        driver = gamepad1;

        // 1. Hardware Init
        limelight.init(hardwareMap, driver, 0);
        turret.init(hardwareMap, driver);
        flywheel.init(hardwareMap, driver);
        intake.init(hardwareMap, driver);
        rgb.init(hardwareMap, driver);

        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                SweMax,
                telemetry
        );
        drive.zeroModules();

        // --- BATTERY SENSOR SETUP ---
        // Get the voltage sensor (usually on the Control Hub)
        VoltageSensor batterySensor = hardwareMap.voltageSensor.iterator().next();

        // 2. Alliance Selection (Init Loop)
        while (opModeInInit()) {
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Instructions", "Press X for Blue, B for Red");
            telemetry.update();

            if (gamepad1.x) isRedAlliance = false;
            if (gamepad1.b) isRedAlliance = true;

            if (isRedAlliance) rgb.showRedWait();
            else rgb.showBlueWait();
        }

        limelight.start();

        // 3. Main Loop
        while (opModeIsActive()) {
            // Read Battery Voltage
            double currentVoltage = batterySensor.getVoltage();

            // --- INPUTS ---
            if (driver.right_stick_button && !lastR3Pressed) {
                autoTrackToggle = !autoTrackToggle;
            }
            lastR3Pressed = driver.right_stick_button;

            // --- VISION & AUTO-AIM ---
            LLResult llResult = limelight.getResult();
            double autoFlywheelPower = -1.0;
            double tx = 0;
            boolean targetFound = false;

            if (autoTrackToggle && llResult != null && llResult.isValid()) {
                targetFound = true;
                tx = llResult.getTx();
                double ty = llResult.getTy();

                double distance = limelight.getDistance(ty);
                autoFlywheelPower = limelight.calculateAutoFlywheelPower(distance);

                telemetry.addData("AutoAim", "Dist: %.1f in | Pwr: %.2f", distance, autoFlywheelPower);
            }

            // --- MECHANISMS ---
            flywheel.controls(autoFlywheelPower);
            intake.control();

            // --- TURRET LOGIC (Fixed for Battery) ---
            double turretPower = 0;
            boolean manualTurret = (driver.right_trigger > 0.1) || (driver.left_trigger > 0.1);

            if (manualTurret) {
                if (driver.right_trigger > 0.1) turretPower = 0.25;
                else turretPower = -0.25;
            } else if (autoTrackToggle && targetFound) {
                limelight.turretPID.kP = Settings.turret_P;
                limelight.turretPID.setTarget(0);
                double pidOut = limelight.turretPID.getPower(tx);
                turretPower = Math.max(-1, Math.min(1, pidOut));
            }

            // Use the new Voltage Compensated method
            turret.setTurretsWithVoltageComp(turretPower, currentVoltage);

            // --- RGB LOGIC ---
            boolean shootingTrigger = driver.b || operator.b;
            boolean flywheelReady = flywheel.flywheeltoggle && flywheel.flywheel.getPower() > 0;

            if (driver.back) {
                rgb.showRainbow();
            } else if (shootingTrigger) {
                if (isRedAlliance) rgb.showRedShoot();
                else rgb.showBlueShoot();
            } else if (flywheelReady) {
                if (isRedAlliance) rgb.showRedReady();
                else rgb.showBlueReady();
            } else {
                if (isRedAlliance) rgb.showRedDefault();
                else rgb.showBlueDefault();
            }

            // --- DRIVE ---
            drive.drive(-driver.left_stick_y, driver.left_stick_x, driver.right_stick_x, 1.0);

            // --- TELEMETRY ---
            telemetry.addData("Voltage", "%.1f V", currentVoltage);
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Mode", autoTrackToggle ? "AUTO" : "MANUAL");
            telemetry.update();
        }
    }
}