package InCaseDiffyFails;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Tank {
    private DcMotor lTan0, lTan1, rTan0, rTan1;
    private Gamepad Controller;

    public void init(HardwareMap hdwMap, Gamepad controller) {
        lTan0 = hdwMap.get(DcMotor.class, "lTan0");
        lTan1 = hdwMap.get(DcMotor.class, "lTan1");
        rTan0 = hdwMap.get(DcMotor.class, "rTan0");
        rTan1 = hdwMap.get(DcMotor.class, "rTan1");

        // Reverse left side if needed
        lTan0.setDirection(DcMotor.Direction.REVERSE);
        lTan1.setDirection(DcMotor.Direction.REVERSE);

        lTan0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lTan1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rTan0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rTan1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lTan0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lTan1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rTan0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rTan1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Controller = controller;
    }

    public void setLeft(double power1, double power2) {
        lTan0.setPower(power1);
        lTan1.setPower(power2);
    }

    public void setRight(double power3, double power4) {
        rTan0.setPower(power3);
        rTan1.setPower(power4);
    }

    public void drive() {
        setLeft(Controller.left_stick_y, Controller.left_stick_y);
        setRight(Controller.right_stick_y, Controller.right_stick_y);
    }
}
