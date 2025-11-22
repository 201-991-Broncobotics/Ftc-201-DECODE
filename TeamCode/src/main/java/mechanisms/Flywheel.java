package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    public boolean flywheeltoggle = false;
    boolean lastBPressed = false;

    public DcMotorEx flywheel; // Changed to Ex for velocity reading
    Gamepad Controller;

    // Variable to hold the active target power (manual or auto)
    private double currentTargetPower = 0;

    public void init(HardwareMap hdwMap, Gamepad controller) {
        flywheel = hdwMap.get(DcMotorEx.class, "flyM");
        Controller = controller;

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Modified controls to accept an Auto-Override power.
     * @param autoPower The calculated power from Limelight (or -1 if disabled)
     */
    public void controls(double autoPower) {
        // Toggle On/Off with Left Bumper
        if (Controller.left_bumper && !lastBPressed) {
            flywheeltoggle = !flywheeltoggle;
        }
        lastBPressed = Controller.left_bumper;

        if (flywheeltoggle) {
            if (autoPower > 0) {
                // Use Auto Speed if provided
                currentTargetPower = autoPower;
            } else {
                // Use Manual Speed from Settings
                currentTargetPower = Settings.FlyPower;
            }

            flywheel.setPower(currentTargetPower);
        } else {
            currentTargetPower = 0;
            flywheel.setPower(0);
        }

        // Manual Tuning D-Pad (Updates the global static setting)
        if (Controller.dpad_up) Settings.FlyPower += 0.005;
        if (Controller.dpad_down) Settings.FlyPower -= 0.005;
    }

    @SuppressWarnings("unused")
    public double getTargetPower() {
        return currentTargetPower;
    }

    @SuppressWarnings("unused")
    public double getCurrentVelocity() {
        return flywheel.getVelocity();
    }
}