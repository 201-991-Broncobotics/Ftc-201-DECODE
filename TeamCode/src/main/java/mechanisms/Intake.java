package mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake {
    boolean intaketoggle,lastAPressed,lastXPressed,rollertoggle,lastYPressed,ballflickup,lastbumperpress,lrollertoggle, dpadpress, dpadtoggle= false;


    Gamepad Controller;
    private DcMotor intake,highroll; //hRolS0

    private Servo ballflick;
    private CRServo lowerroller1,lowerroller2;

    public void init(HardwareMap hwdM, Gamepad controller) {
        intake = hwdM.get(DcMotor.class, "intM");
        highroll = hwdM.get(DcMotor.class, "hRolM");
        ballflick = hwdM.get(Servo.class, "pusS");
        lowerroller1=hwdM.get(CRServo.class, "lRolS0");
        lowerroller2=hwdM.get(CRServo.class, "lRolS1");
        Controller = controller;
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }
    public void lowerRollers(double power) {
        lowerroller1.setPower(power);
        lowerroller2.setPower(power);
    }    public void control() {
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
            ballflick.setPosition(90);

            lastYPressed = true;
        }
        lastYPressed = Controller.y;
        if (Controller.y && lastYPressed) {
            ballflick.setPosition(0);
            lastYPressed = false;
        }
        if(Controller.right_bumper && !lastbumperpress){
            lrollertoggle = !lrollertoggle;
        }
        lastbumperpress = Controller.right_bumper;
        if (lrollertoggle){
            lowerroller1.setPower(-1);
            lowerroller2.setPower(1);
        }
        else{
            lowerroller1.setPower(0);
            lowerroller2.setPower(0);
        }
        if(Controller.dpad_up && !dpadpress){
            dpadtoggle = !dpadtoggle;
        }
        dpadpress = Controller.dpad_up;
        if(dpadtoggle){
            rollertoggle = !rollertoggle;
            intaketoggle = !intaketoggle;
            lrollertoggle = !lrollertoggle;
        }
    }
}

