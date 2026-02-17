package Autons;

import static mechanisms.Settings.SweMax;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import Diffy.DiffySwerveKinematics;
import mechanisms.Flywheel;
import mechanisms.Intake;
import mechanisms.Settings;
import mechanisms.Turret;
import sensors.Limelight;

@Autonomous(name = "Auto: Shoot 3 (Formula)", group = "Autonomous")
public class Auto_Shoot3_Formula extends LinearOpMode {

    private DiffySwerveKinematics drive;
    private Turret turret = new Turret();
    private Flywheel flywheel = new Flywheel();
    private Intake intake = new Intake();
    private Limelight limelight = new Limelight();

    @Override
    public void runOpMode() {
        Gamepad dummy = new Gamepad();

        turret.init(hardwareMap, dummy);
        flywheel.init(hardwareMap, dummy, limelight);
        intake.init(hardwareMap, dummy);

        // SWITCHED: Now using Pipeline 1 (Blue)
        limelight.init(hardwareMap, dummy, 1);

        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                SweMax, telemetry
        );
        drive.zeroModules();

        // 1. INIT ACTION: Push Servo DOWN immediately
       /* intake.setBallflick(Settings.flick_DOWN);

        */
        // SWITCHED: Set offset for Blue side
        Settings.aimOffsetScale = -1.1;

        telemetry.addData("Status", "Ready (Pipeline 1 - Blue)");
        telemetry.update();

        waitForStart();
        limelight.start();

        if (opModeIsActive()) {

            // 2. SPIN UP (2 Seconds)
            waitAndTrack(2000);

            // 3. SHOOT SEQUENCE (Hold B for 7s)
            simulateHoldingB(5000);

            // 4. DRIVE FORWARD + INTAKE (Hold A) - Original 2.0s Length
            // -0.5 is forward
            driveAndAction(2800, -0.5, true);

            // 5. DRIVE BACKWARD - Original 2.0s Length
            // 0.5 is backward
            driveAndAction(2800, 0.5, false);

            // 6. SHOOT SEQUENCE (Hold B for 7s)
            simulateHoldingB(5000);

            // 7. DRIVE FORWARD (Original 2.0s)
            // Turn off flywheel/turret for final park
            Settings.fly_targetRPM = 0;
            turret.autoTrack(0);

            ElapsedTime driveTimer = new ElapsedTime();
            while (opModeIsActive() && driveTimer.seconds() < 2.0) {
                drive.drive(-0.5, 0, 0, 1.0);
                flywheel.controls(); // Spin down
            }

            // STOP
            drive.drive(0, 0, 0, 0);
            intake.setIntake(0);
            intake.setHighroll(0);
            intake.lowerRollers(0);
            Settings.fly_targetRPM = 0;
            flywheel.controls();
        }
    }

    // --- HELPER METHODS ---

    /**
     * Simulates holding "B" for a specific duration.
     * It loops the shoot cycle (Flick Up -> Flick Down/Shoot)
     */
    private void simulateHoldingB(long durationMs) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.milliseconds() < durationMs) {
            // Cycle: Flick UP (250ms) -> Flick DOWN+SHOOT (800ms)

            // 1. Flick UP
            long flickUpEnd = System.currentTimeMillis() + 250;
            while(opModeIsActive() && System.currentTimeMillis() < flickUpEnd) {
                updateSystems(); // Keep aiming
                if(timer.milliseconds() >= durationMs) break;
            }

            // 2. Flick DOWN + SHOOT
            intake.setHighroll(-1);
            intake.setIntake(-1);
            intake.lowerRollers(-1);

            long shootEnd = System.currentTimeMillis() + 800;
            while(opModeIsActive() && System.currentTimeMillis() < shootEnd) {
                updateSystems(); // Keep aiming
                if(timer.milliseconds() >= durationMs) break;
            }
        }

        // Stop shooting motors after loop
        intake.setHighroll(0);
        intake.setIntake(0);
        intake.lowerRollers(0);
    }

    /**
     * Drives while optionally running Intake (Simulating A)
     * @param durationMs How long to drive
     * @param speedY Forward/Back speed
     * @param runIntake True = Simulate 'A', False = Just drive
     */
    private void driveAndAction(long durationMs, double speedY, boolean runIntake) {
        ElapsedTime timer = new ElapsedTime();

        if (runIntake) {
            intake.setIntake(Settings.IntakePowe);
            intake.setHighroll(Settings.HighRolPow);
            intake.lowerRollers(-1);
        } else {
            intake.setIntake(0);
            intake.setHighroll(0);
            intake.lowerRollers(0);
        }

        while (opModeIsActive() && timer.milliseconds() < durationMs) {
            drive.drive(speedY, 0, 0, 1.0);
            updateSystems(); // Keep flywheel spinning/tracking
        }

        // Stop Drive
        drive.drive(0,0,0,0);
        // Stop Intake
        intake.setIntake(0);
        intake.setHighroll(0);
        intake.lowerRollers(0);
    }

    /**
     * Standard Wait that keeps Flywheel & Turret active
     */
    private void waitAndTrack(long durationMs) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < durationMs) {
            updateSystems();
        }
    }

    /**
     * Updates Flywheel (Formula) and Turret (Auto Aim)
     */
    private void updateSystems() {
        double distance = limelight.getDistance();

        // 1. FLYWHEEL FORMULA
        if (distance > 0) {
            double formulaRPM = (Settings.fly_A * Math.pow(distance, 2)) +
                    (Settings.fly_B * distance) +
                    Settings.fly_C;
            Settings.fly_targetRPM = formulaRPM;
        }
        flywheel.controls();

        // 2. TURRET TRACKING
        LLResult llResult = limelight.getResult();
        if (llResult != null && llResult.isValid()) {
            double currentTx = llResult.getTx();
            double tagYaw = limelight.getYaw();

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
            turret.autoTrack(0);
        }
    }
}