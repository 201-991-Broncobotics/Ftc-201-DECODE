package mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake {
    boolean intaketoggle = false;
    boolean lastAPressed = false;
    boolean lastXPressed = false;
    boolean rollertoggle = false;
    boolean lastYPressed = false;
    boolean ballflickup = false;


    Gamepad Controller;
    private DcMotor intake;
    private CRServo highroll0; //hRolS0
    private CRServo highroll1;
    private Servo ballflick;

    public void init(HardwareMap hwdM, Gamepad controller) {
        intake = hwdM.get(DcMotor.class, "intM");
        highroll0 = hwdM.get(CRServo.class, "hRolS0");
        highroll1 = hwdM.get(CRServo.class, "hRolS1");
        ballflick = hwdM.get(Servo.class, "pusS");

        Controller = controller;
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void control() {
        if (Controller.a && !lastAPressed) {
            intaketoggle = !intaketoggle;
        }
        if (intaketoggle) {
            intake.setPower(-.40); // intake on
        } else {
            intake.setPower(0);  // intake off
        }
        if (Controller.x && !lastXPressed) {
            rollertoggle = !rollertoggle;
        }
        if (rollertoggle) {
            highroll0.setPower(1);
            highroll1.setPower(-1);
        } else {
            highroll0.setPower(0);
            highroll1.setPower(0);
        }
        if (Controller.y && !lastYPressed) {
            ballflick.setPosition(90);

            lastYPressed = true;
        }
        if (Controller.y && lastYPressed) {
            ballflick.setPosition(0);
            lastYPressed = false;
        }
    }
}

