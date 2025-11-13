package mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Flywheel {
    boolean flywheeltoggle, lastBPressed, lastbumppressed, autotracktoggle = false;
    ;
    public DcMotor flywheel;
    public double setPower = -0.6;

    Gamepad Controller;


    public void init(HardwareMap hdwMap, Gamepad controller) {

        flywheel = hdwMap.get(DcMotorEx.class, "flyM");
        Controller = controller;
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setFlywheel(double power) {
        flywheel.setPower(power);
    }

    public void controls(double power) {
        if (Controller.b && !lastBPressed) {
            flywheeltoggle = !flywheeltoggle;
        }
        lastBPressed = Controller.b;
        if (flywheeltoggle) {
            flywheel.setPower(power); // Set target speed
        } else {
            flywheel.setPower(0); // Stop flywheel
        }
    }
}