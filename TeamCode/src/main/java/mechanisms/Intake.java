package mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
// Controls: B = All intake rolles; R1 = Ball flick A= Intake
public class Intake {
    boolean intaketoggle, lastAPressed, lastBPressed, rollertoggle, ballflickup = false;
    boolean autotracktoggle, lastbumppressed = false;

    // 🔹 New variables for release servo
    private boolean releaseToggle = false;
    private boolean lastYPressed = false;
    Gamepad Controller;
    private DcMotor intake, highroll;
    private Servo ballflick;
    private CRServo lowerroller1, lowerroller2;
    private Servo releaseServo; // relS
    public void init(HardwareMap hwdM, Gamepad controller) {
        intake = hwdM.get(DcMotor.class, "intM");
        highroll = hwdM.get(DcMotor.class, "hRolM");
        ballflick = hwdM.get(Servo.class, "pusS");
        lowerroller1 = hwdM.get(CRServo.class, "lRolS0");
        lowerroller2 = hwdM.get(CRServo.class, "lRolS1");
        releaseServo = hwdM.get(Servo.class, "sorS");
        Controller = controller;
    }

    public void setHighroll(double power) {
        highroll.setPower(power);
    }

    public void lowerRollers(double power) {
        lowerroller1.setPower(power);
        lowerroller2.setPower(power);
    }

    public void control() {
        if (Controller.a && !lastAPressed) {  // Button A
            intaketoggle = !intaketoggle;
        }
        lastAPressed = Controller.a;
        if (intaketoggle) {
            intake.setPower(-.7); // intake on
        } else {
            intake.setPower(0);  // intake off
        }
        if (Controller.right_bumper && !ballflickup) {  // Right Bumper ( Duh)
            if (ballflick.getPosition() == 0) {
                ballflick.setPosition(90);
            } else {
                ballflick.setPosition(0);
            }
            ballflickup = true;
        }
        if (!Controller.b) {    // B buttoN
            lastBPressed = false;
        }
        if (Controller.b && !lastBPressed) {
            rollertoggle = !rollertoggle;
        }
        if (rollertoggle) {
            lowerroller1.setPower(-1);
            lowerroller2.setPower(1);
            highroll.setPower(-.80);
        } else {
            lowerroller1.setPower(0);
            lowerroller2.setPower(0);
            highroll.setPower(0);
        }
        lastBPressed = Controller.b;
        if (Controller.y && !lastYPressed) {
            releaseToggle = !releaseToggle;
            if (releaseToggle) {
                releaseServo.setPosition(270); // move up 90° (adjust if needed)
            } else {
                releaseServo.setPosition(0); // back down
            }
        }
        lastYPressed = Controller.y;
    }
}
