package mechanisms;

import static mechanisms.Settings.FlyPower;
import static mechanisms.Settings.closevel;
import static mechanisms.Settings.farvel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import Diffy.PIDController;


public class Flywheel {
    boolean flywheeltoggle, lastBPressed, lastbumppressed, autotracktoggle,speedLimitingEnabled = false;
    double integralSum = 0;
    public DcMotorEx flywheel;
    Gamepad Controller;
    public double minDifference, minPosition, maxPosition, minPower, maxPower, initialPower, maxSpeed, tolerance, maxIntegral, maxAcceleration, maxDeceleration; // all of these variables can be changed elsewhere in the code
    public double kP, kI, kD, kP2;

    // public PIDController FlywheelPID;



    public void init(HardwareMap hdwMap, Gamepad controller) {

        flywheel = hdwMap.get(DcMotorEx.class, "flyM");
        Controller = controller;
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // FlywheelPID = new PIDController(0, 0, 0, () -> flywheel.getCurrentPosition());


    }
        public void setFlywheel(double vel) {
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(Settings.flykP, Settings.flykI, Settings.flykD, 0));
        flywheel.setVelocity(vel);
    }

    public void controls() {
        if (Controller.left_bumper && !lastBPressed) {
            flywheeltoggle = !flywheeltoggle;
        }
        lastBPressed = Controller.left_bumper;
        if (flywheeltoggle) {
            flywheel.setVelocity(Settings.flyVelocity / 60 * 20); // Set target speed
        } else {
            flywheel.setPower(0); // Stop flywheel
        }
        if (Controller.dpadUpWasPressed()) {
            FlyPower = FlyPower + (0.05 * FlyPower);
        }
        if (Controller.dpadDownWasPressed()) {
            FlyPower = FlyPower - (0.05 * FlyPower);

        }
        if (Controller.dpad_left) {
            setFlywheel(farvel);
        }
        if (Controller.dpad_right) {
            setFlywheel(closevel);

        }
    }
}
