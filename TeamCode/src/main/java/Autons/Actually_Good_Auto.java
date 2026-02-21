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
import InCaseDiffyFails.Tank;
import mechanisms.Flywheel;
import mechanisms.Intake;
import mechanisms.Settings;
import mechanisms.Turret;
import sensors.Limelight;

@Autonomous(name = "Better Auto")
public class Actually_Good_Auto extends LinearOpMode {

    private ElapsedTime runtime;
    double lastKnownTx = 0;


    private Turret turret = new Turret();
    private Flywheel flywheel = new Flywheel();
    private Intake intake = new Intake();
    private Limelight limelight = new Limelight();
    private DiffySwerveKinematics drive;
    double distance = limelight.getDistance();
    double formulaRPM = 0;


    @Override
    public void runOpMode() {
        Gamepad dummy = new Gamepad(); // no controller in autonomous
        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                SweMax, // max motor power limit
                telemetry
        );
        turret.init(hardwareMap, dummy);
        intake.init(hardwareMap, dummy);
        limelight.init(hardwareMap, dummy, 0);

        waitForStart();
        while (opModeIsActive()) {
            ;
            if (distance > 0) {
                formulaRPM = (Settings.fly_A * Math.pow(distance, 2)) +
                        (Settings.fly_B * distance) +
                        Settings.fly_C;
                flywheel.fly1.setVelocity(formulaRPM);
                flywheel.fly0.setVelocity(formulaRPM);
            }
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

                intake.setIntake(Settings.IntakePowe);
                intake.setHighroll(Settings.HighRolPow);
                intake.lowerRollers(-1);
                sleep(5000); // 5 seconds
                intake.setPushservos(0);
                sleep(2000);
                intake.setPushservos(90);
                intake.setIntake(0);
                intake.setHighroll(0);
                intake.lowerRollers(0);
                sleep(2000);

                drive.drive(-1, 0, 0, 1);




        }
    }
}