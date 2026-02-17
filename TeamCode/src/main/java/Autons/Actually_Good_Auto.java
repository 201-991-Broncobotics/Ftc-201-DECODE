package Autons;

import static mechanisms.Settings.SweMax;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import Diffy.DiffySwerveKinematics;
import InCaseDiffyFails.Tank;
import mechanisms.Flywheel;
import mechanisms.Intake;
import mechanisms.Turret;
import sensors.Limelight;

@Autonomous(name = "Better Auto")
public class Actually_Good_Auto extends LinearOpMode {

    private ElapsedTime runtime;

    private Turret turret = new Turret();
    private Flywheel flywheel = new Flywheel();
    private Intake intake = new Intake();
    private Limelight limelight = new Limelight();
    private  DiffySwerveKinematics drive;



    @Override
    public void runOpMode(){
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
       /* intake.setBallflick(90);            // push up

        waitForStart();
        while(opModeIsActive()){;
        drive.drive(-1,0,0,1);
    }

        */
}
}