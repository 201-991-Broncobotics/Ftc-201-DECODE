package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Flywheel {
    boolean flywheeltoggle = false;
    boolean lastBPressed = false;
    private DcMotor flywheel;
    Gamepad Controller;


    public void init(HardwareMap hdwMap, Gamepad controller) {

        flywheel = hdwMap.get(DcMotor.class, "flyM");
        Controller = controller;

    }

    public void setFlywheel(double power) {
        flywheel.setPower(power);
    }

    public void controls() {
        if(Controller.b && !lastBPressed) {
            flywheeltoggle = !flywheeltoggle;
        }
        if (flywheeltoggle) {
            flywheel.setPower(-.70); // Flywheel on
        } else {
            flywheel.setPower(0);  // flywheel off
        }
    }
}

