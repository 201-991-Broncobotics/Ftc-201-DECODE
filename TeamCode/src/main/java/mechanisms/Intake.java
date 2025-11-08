package mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    boolean intaketoggle, lastAPressed, lastXPressed, rollertoggle, lastYPressed, ballflickup, lastbumperpress, lrollertoggle, dpadpress, dpadtoggle = false;
    boolean autotracktoggle, lastbumppressed = false;

    // 🔹 New variables for release servo
    private boolean releaseToggle = false;
    private boolean lastR3Pressed = false;

    Gamepad Controller;
    private sensors.Limelight limelight;

    public void setLimelight(sensors.Limelight ll) {
        limelight = ll;
    }

    private DcMotor intake, highroll;
    private Servo ballflick;
    private CRServo lowerroller1, lowerroller2;

    // 🔹 New servo
    private Servo releaseServo; // relS

    public void init(HardwareMap hwdM, Gamepad controller) {
        intake = hwdM.get(DcMotor.class, "intM");
        highroll = hwdM.get(DcMotor.class, "hRolM");
        ballflick = hwdM.get(Servo.class, "pusS");
        lowerroller1 = hwdM.get(CRServo.class, "lRolS0");
        lowerroller2 = hwdM.get(CRServo.class, "lRolS1");

        // 🔹 Initialize new servo
        releaseServo = hwdM.get(Servo.class, "sorS");

        Controller = controller;
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setHighroll(double power) {
        highroll.setPower(power);
    }

    public void lowerRollers(double power) {
        lowerroller1.setPower(power);
        lowerroller2.setPower(power);
    }

    public void control() {
        if (Controller.a && !lastAPressed) {
            intaketoggle = !intaketoggle;
        }
        lastAPressed = Controller.a;
        if (intaketoggle) {
            intake.setPower(-.7); // intake on
        } else {
            intake.setPower(0);  // intake off
        }
        if (Controller.x && !lastXPressed) {
            rollertoggle = !rollertoggle;
        }
        lastXPressed = Controller.x;
        if (rollertoggle) {
            highroll.setPower(-.60);
        } else {
            highroll.setPower(0);
        }
        if (Controller.y && !lastYPressed) {
            if (ballflick.getPosition() == 0) {
                ballflick.setPosition(90);
            } else {
                ballflick.setPosition(0);
            }
            lastYPressed = true;
        }
        if (!Controller.y) {
            lastYPressed = false;
        }
        if (Controller.right_bumper && !lastbumperpress) {
            lrollertoggle = !lrollertoggle;
        }
        lastbumperpress = Controller.right_bumper;
        if (lrollertoggle) {
            lowerroller1.setPower(-1);
            lowerroller2.setPower(1);
        } else {
            lowerroller1.setPower(0);
            lowerroller2.setPower(0);
        }
        if (Controller.dpad_up && !dpadpress) {
            dpadtoggle = !dpadtoggle;
        }
        dpadpress = Controller.dpad_up;
        if (dpadtoggle) {
            rollertoggle = !rollertoggle;
            intaketoggle = !intaketoggle;
            lrollertoggle = !lrollertoggle;
        }

        // 🔹 New Release Servo Control (R3 button)
        if (Controller.right_stick_button && !lastR3Pressed) {
            releaseToggle = !releaseToggle;
            if (releaseToggle) {
                releaseServo.setPosition(270); // move up 90° (adjust if needed)
            } else {
                releaseServo.setPosition(0); // back down
            }
        }
        lastR3Pressed = Controller.right_stick_button;
    }
}
