package Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import Diffy.SwerveModule;
import Diffy.DiffySwerveKinematics;
@TeleOp(name="RestSwerve", group="Utility")
public class RestSwerveOp extends LinearOpMode{
    DiffySwerveKinematics drive;
    @Override
    public void runOpMode() {
        drive = new DiffySwerveKinematics(
                hardwareMap.get(DcMotorEx.class, "lSwe0"),
                hardwareMap.get(DcMotorEx.class, "lSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe1"),
                hardwareMap.get(DcMotorEx.class, "rSwe0"),
                1.0, // max motor power limit
                telemetry
        );
        drive.zeroModules();
        telemetry.addLine("Swerve Wheels and Wrist have been reset successfully");
        telemetry.update();
        // functions.Sleep(500);
        // requestOpModeStop();
        waitForStart();
        requestOpModeStop();
    }
}
