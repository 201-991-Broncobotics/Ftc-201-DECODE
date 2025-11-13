package Autons;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import InCaseDiffyFails.Tank;
import mechanisms.Flywheel;
import mechanisms.Intake;
import mechanisms.Turret;
import sensors.Limelight;

@Autonomous(name = "BlueAuto")
public class BlueAuto extends LinearOpMode {

    private ElapsedTime runtime;
    private Tank drive = new Tank();
    private Turret turret = new Turret();
    private Flywheel flywheel = new Flywheel();
    private Intake intake = new Intake();
    private Limelight limelight = new Limelight();

    @Override
    public void runOpMode() {

        Gamepad dummy = new Gamepad(); // no controller in autonomous

        drive.init(hardwareMap, dummy);
        turret.init(hardwareMap, dummy);
        flywheel.init(hardwareMap, dummy);
        intake.init(hardwareMap, dummy);
        limelight.init(hardwareMap, dummy, 0);

        runtime = new ElapsedTime();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // --- Step 1: Spin up flywheel ---
        flywheel.setFlywheel(0.6);
        sleep(1000);

        // --- Step 2: Auto-track target for up to 3 seconds ---
        while (opModeIsActive() && runtime.seconds() < 3) {
            LLResult llResult = limelight.getResult();
            if (llResult != null && llResult.isValid()) {
                double tx = llResult.getTx();
                double deadzone = 1.0;
                double power = 0.2;

                if (tx > deadzone) turret.autoTrack(power);
                else if (tx < -deadzone) turret.autoTrack(-power);
                else turret.autoTrack(0);

                telemetry.addData("AutoTrack Tx", tx);
            } else {
                turret.autoTrack(0);
                telemetry.addLine("No valid Limelight target");
            }
            telemetry.update();
        }
        intake.lowerRollers(1.0);   // Start bottom rollers forward
        intake.setHighroll(-0.6);   // Run top roller toward flywheel
        sleep(10000);                // Wait 3 seconds to shoot all rings

        intake.lowerRollers(0);     // Stop bottom rollers
        intake.setHighroll(0);      // Stop top roller
        flywheel.setFlywheel(0);

        // --- Step 3: Drive forward for 2 seconds ---
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.0) {
            drive.setRight(-0.2, -0.2);
            drive.setLeft(-0.2, -0.2);
        }

        // --- Step 4: Stop drive ---
        drive.setRight(0, 0);
        drive.setLeft(0, 0);
    }
}
