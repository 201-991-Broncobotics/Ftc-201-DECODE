package Teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import InCaseDiffyFails.Tank;
import mechanisms.Flywheel;
import mechanisms.Intake;
import sensors.Limelight;

@TeleOp(name = "Blue Op", group = "Concept")
public class BlueOp extends LinearOpMode {

    Gamepad operator, driver;
    Tank drive = new Tank();
    mechanisms.Turret turret = new mechanisms.Turret();
    Flywheel flywheel = new Flywheel();
    Intake intake = new Intake();
    private IMU imu;
    private Limelight limelight = new Limelight();
    boolean autoTrackToggle = false;
    boolean lastBumperPressed = false;



    ColorSensor cs0;

    @Override
    public void runOpMode() {

        operator = gamepad2;
        driver = gamepad1;
        limelight.init(hardwareMap, driver,1);
        telemetry.addLine("Limelight initialized");
        turret.init(hardwareMap, driver);
        flywheel.init(hardwareMap, driver);
        intake.init(hardwareMap, driver);
        drive.init(hardwareMap, operator);
        waitForStart();
        limelight.start();
        while (opModeIsActive()) {

            // ---------- Toggle Auto-Track ----------
            if (driver.left_bumper && !lastBumperPressed) {
                autoTrackToggle = !autoTrackToggle;
            }
            lastBumperPressed = driver.left_bumper;

            // ---------- Turret Control ----------
            boolean manualTurretActive = driver.right_trigger > 0.1 || driver.left_trigger > 0.1;

            if (manualTurretActive) {
              //  turret.controls(); // manual override
            } else if (autoTrackToggle) {
                LLResult llResult = limelight.getResult();
                if (llResult != null && llResult.isValid()) {
                    double tx = llResult.getTx(); // horizontal offset
                    double ta = llResult.getTa();
                    double deadzone = 1.0;
                    double power = 0.2;
                   // if (tx > deadzone) turret.autoTrack(power);
                  //  else if (tx < -deadzone) turret.autoTrack(-power);
                 //   else turret.autoTrack(0);

                    telemetry.addData("AutoTrack Tx", tx);
                } else {
                  //  turret.autoTrack(0);
                    telemetry.addLine("No valid Limelight target");
                }
            } else {
               // turret.autoTrack(0); // stop if no manual or auto-track
            }

           // turret.controls();
            flywheel.controls();
            drive.drive();
            intake.control();
            telemetry.addData("Flywheel Volcity", flywheel.flywheel.getPower());
            LLResult llResult = limelight.getResult();
            if (llResult != null && llResult.isValid()) {
                telemetry.addData("Limelight Tx", llResult.getTx());
                telemetry.addData("Limelight Ty", llResult.getTy());
                telemetry.addData("Limelight Ta", llResult.getTa());
            } else {
                telemetry.addLine("No valid Limelight target"); }
            telemetry.addData("Motor Speed: ", flywheel.flywheel.getPower());
            telemetry.addData("Controller Magnitude: ", driver.right_stick_y);
            telemetry.update();
        }
    }
}