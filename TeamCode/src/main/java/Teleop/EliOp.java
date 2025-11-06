package Teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import Diffy.DiffySwerveKinematics;
import mechanisms.Flywheel;
import mechanisms.Intake;
import mechanisms.Turret;
import sensors.Limelight;

@TeleOp(name = "Eli Op", group = "Concept")
public class EliOp extends LinearOpMode {

    Gamepad operator, driver;
    DiffySwerveKinematics drive;

    mechanisms.Turret turret = new mechanisms.Turret();
    Flywheel flywheel = new Flywheel();
    Intake intake = new Intake();
    private IMU imu;


    ColorSensor cs0;

    @Override
    public void runOpMode() {

        operator = gamepad2;
        driver = gamepad1;

        turret.init(hardwareMap, driver);
        flywheel.init(hardwareMap, driver);
        intake.init(hardwareMap, driver);
        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                1.0, // max motor power limit
                telemetry
        );
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            turret.controls();
            flywheel.controls();
            intake.control();
            telemetry.update();
            telemetry.addData("Flywheel Volcity", flywheel.targetVelocity);
            double forward = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            // Deadzones
            if (Math.abs(forward) < 0.05) forward = 0;
            if (Math.abs(strafe) < 0.05) strafe = 0;
            if (Math.abs(turn) < 0.05) turn = 0;

            double throttle = 1.0;
            drive.drive(forward, strafe, turn, throttle);
        }
    }
}