package mechanisms;

import static mechanisms.Settings.FlyPower;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import mechanisms.Settings;



public class Flywheel {
    boolean flywheeltoggle, lastBPressed, lastbumppressed, autotracktoggle = false;
    ;
    public DcMotor flywheel;

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

    public void controls() {
        if (Controller.left_bumper && !lastBPressed) {
            flywheeltoggle = !flywheeltoggle;
        }
        lastBPressed = Controller.left_bumper;
        if (flywheeltoggle) {
            flywheel.setPower(FlyPower); // Set target speed
        } else {
            flywheel.setPower(0); // Stop flywheel
        }
        if (Controller.dpadUpWasPressed()){
            FlyPower = FlyPower + 0.05;
        }
        if (Controller.dpadDownWasPressed()){
            FlyPower = FlyPower - 0.05;
        }
    }
}
