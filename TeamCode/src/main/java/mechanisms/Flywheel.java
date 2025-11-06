package mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Flywheel {
    boolean flywheeltoggle, lastBPressed, lastbumppressed, autotracktoggle = false;
    ;
    private DcMotorEx flywheel;
    public double targetVelocity = -4200;

    Gamepad Controller;


    public void init(HardwareMap hdwMap, Gamepad controller) {

        flywheel = hdwMap.get(DcMotorEx.class, "flyM");
        Controller = controller;
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setFlywheel(double power) {
        flywheel.setPower(power);
    }

    public void controls() {
        if (Controller.b && !lastBPressed) {
            flywheeltoggle = !flywheeltoggle;
        }
        lastBPressed = Controller.b;
        if (flywheeltoggle) {
            flywheel.setVelocity(targetVelocity); // Set target speed
        } else {
            flywheel.setVelocity(0); // Stop flywheel
        }
    }
}