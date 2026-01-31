/*package Autons;

import static mechanisms.Settings.SweMax;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import Diffy.DiffySwerveKinematics;
import mechanisms.Flywheel;
import mechanisms.Intake;
import mechanisms.Turret;
import sensors.Limelight;

@Autonomous(name = "Auto: Drive Only", group = "Autonomous")
public class Auto_DriveOnly extends LinearOpMode {

    private DiffySwerveKinematics drive;
    // We init these just to lock them in place, but we won't use them
    private Turret turret = new Turret();
    private Flywheel flywheel = new Flywheel();
    private Intake intake = new Intake();
    private Limelight limelight = new Limelight();

    @Override
    public void runOpMode() {
        Gamepad dummy = new Gamepad();

        // 1. INIT (Lock mechanisms)
        turret.init(hardwareMap, dummy);
        flywheel.init(hardwareMap, dummy, limelight);
        intake.init(hardwareMap, dummy);
        limelight.init(hardwareMap, dummy, 0);

        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                SweMax, telemetry
        );
        drive.zeroModules();

        telemetry.addData("Status", "Ready to Drive Only");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // 2. DRIVE FORWARD
            telemetry.addData("Action", "Driving Forward");
            telemetry.update();

            ElapsedTime driveTimer = new ElapsedTime();

            // Drives for 2.0 seconds
            while (opModeIsActive() && driveTimer.seconds() < 2.0) {
                // -0.5 is forward power. Change to 0.5 if it goes backward.
                drive.drive(-0.5, 0, 0, 1.0);
            }

            // 3. STOP
            drive.drive(0, 0, 0, 0);
        }
    }
}*/