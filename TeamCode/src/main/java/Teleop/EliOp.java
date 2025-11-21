
package Teleop;

import static mechanisms.Settings.HighRolPow;
import static mechanisms.Settings.SweMax;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;

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

    Gamepad operator, driver;
    mechanisms.Turret turret = new mechanisms.Turret();
    Flywheel flywheel = new Flywheel();
    Intake intake = new Intake();
    RGB rgb = new RGB();
    private IMU imu;
    private Limelight limelight = new Limelight();
    boolean autoTrackToggle = false;
    boolean lastR3Pressed = false;



    ColorSensor cs0;

    @Override
    public void runOpMode() {


        operator = gamepad2;
        driver = gamepad2;
        limelight.init(hardwareMap, driver,0);
        telemetry.addLine("Limelight initialized");
        turret.init(hardwareMap, driver);
        flywheel.init(hardwareMap, driver);
        intake.init(hardwareMap, driver);
        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                SweMax, // max motor power limit
                telemetry
        );
        drive.zeroModules();
        waitForStart();
        limelight.start();
        while (opModeIsActive()) {
            //Micah's thing
            intake.setIntake(0);
            intake.setHighroll(-0.1);
            intake.lowerRollers(0);

            // ---------- Toggle Auto-Track ----------
            if (driver.right_stick_button && !lastR3Pressed) {
                autoTrackToggle = !autoTrackToggle;
            }
            lastR3Pressed = driver.right_stick_button;

            // ---------- Turret Control ----------
            boolean manualTurretActive = driver.right_trigger > 0.1 || driver.left_trigger > 0.1;

            if (manualTurretActive) {
                // Turret
                turret.controls();
            } else {
                LLResult llResult = limelight.getResult();
                if (llResult != null && llResult.isValid()) {
                    double tx = llResult.getTx(); // horizontal offset

                    // COMMENT THIS OUT WHEN TUNING IS DONE
                    limelight.turretPID.setPIDF(
                            Settings.turret_P,
                            Settings.turret_I,
                            Settings.turret_D,
                            0.0
                    );

                    // Use PID to minimize tx
                    double power = limelight.turretPID.calculate(tx);
                    telemetry.addData("PID power:", power);
                    // Clamp power to [-1, 1]
                    power = power > 0 ? Math.min(1, power) : Math.max(-1, power);
                    turret.setTurrets(power,power);

                    // ELI'S ORIGINAL:
                    // double deadzone = 1.0;
                    // double power = 0.2;
                    // if (tx > deadzone) robot.turret.setPower(power);
                    // else if (tx < -deadzone) robot.turret.setPower(-power);
                    // else robot.turret.setPower(0);

                    telemetry.addData("AutoTrack Tx", tx);
                } else {
                    turret.setTurrets(0,0);
                    telemetry.addLine("No valid Limelight target");

                }
            }
            //  else {
            //      robot.turret.setPower(0); // stop if no manual or auto-track
            //  }


            turret.controls();
            flywheel.controls();


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
            telemetry.update();
            double forward = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            double throttle = 1.0;
            drive.drive(forward, strafe, turn, throttle);
            }
        }
    }

