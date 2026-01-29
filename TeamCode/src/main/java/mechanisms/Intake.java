package mechanisms;

import static java.lang.Thread.sleep;
import static mechanisms.Settings.IntakePowe;
import static mechanisms.Settings.HighRolPow;
import static mechanisms.Settings.redownpos;
import static mechanisms.Settings.reuppos;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Controls: B = All intake rolles; R1 = Ball flick A= Intake
public class Intake {
    boolean intaketoggle, lastAPressed, lastBPressed, rollertoggle, ballflickup, lastXPressed = false;
    boolean autotracktoggle, lastbumppressed = false;

    // 🔹 New variables for release servo
    private boolean releaseToggle = false;
    private boolean lastYPressed = false;
    Gamepad Controller;
    private DcMotor intake, highroll;
    private Servo ballflick;
    private CRServo lowerroller1, lowerroller2;
    private ElapsedTime shootTimer = new ElapsedTime();
    private int shootState = 0;
    private boolean shooting = false;
    private Servo SortServo, realeaseservo,paddle; // relS

    public void init(HardwareMap hwdM, Gamepad controller) {
        intake = hwdM.get(DcMotor.class, "intM");
        highroll = hwdM.get(DcMotor.class, "hRolM");
        ballflick = hwdM.get(Servo.class, "pusS");
        lowerroller1 = hwdM.get(CRServo.class, "lRolS0");
        lowerroller2 = hwdM.get(CRServo.class, "lRolS1");
        SortServo = hwdM.get(Servo.class, "sorS");
        realeaseservo = hwdM.get(Servo.class, "relS");
        Controller = controller;
    }

    public void setHighroll(double power) {
        highroll.setPower(power);
    }

    public void setIntake(double power) {
        intake.setPower(power);
    }

    public void setBallflick(double pos) {
        ballflick.setPosition(pos);
    }


    public void lowerRollers(double power) {
        lowerroller1.setPower(power);
        lowerroller2.setPower(power);
    }

    public void release(double pos) {
        realeaseservo.setPosition(pos);
    }

    public void control() {

        if (Controller.a) {
            intake.setPower(IntakePowe);
            setHighroll(HighRolPow);
            lowerRollers(-1);
        }
        if (Controller.aWasReleased()) {
            intake.setPower(1);
            setHighroll(1);
            lowerRollers(-.3);
        }
        if (Controller.b && !shooting) {
            shooting = true;
            setIntake(-1);
            shootState = 0;
            shootTimer.reset();
            realeaseservo.setPosition(reuppos); // move up 90° (adjust if needed)
            SortServo.setPosition(1);

        }

        if (!Controller.b) {
            shooting = false;
            shootState = 0;
            setBallflick(90);   // Push down
            setIntake(0);
            realeaseservo.setPosition(redownpos); // back down

        }

// State machine to simulate the while-loop behavior safely
        if (shooting) {
            switch (shootState) {
                case 0:
                    // Step 1: start cycle
                    setBallflick(0);       // push up
                    setHighroll(0);
                    shootTimer.reset();
                    shootState = 1;
                    break;

                case 1:
                    // Step 2: wait 100ms
                    if (shootTimer.milliseconds() >= 250) {
                        setBallflick(90);   // push down
                        shootTimer.reset();
                        shootState = 2;
                    }
                    break;

                case 2:
                    // Step 3: wait 250ms before next cycle
                    if (shootTimer.milliseconds() >= 750) {
                        shootState = 0; // loop back to step 1
                    }
                    break;
            }
        }
        if (Controller.x) {
            setIntake(-IntakePowe);
            lowerRollers(1);
            setHighroll(-HighRolPow);
        }
        /*if (Controller.b && !lastYPressed) {
            releaseToggle = !releaseToggle;
            if (releaseToggle) {
                realeaseservo.setPosition(reuppos); // move up 90° (adjust if needed)
                SortServo.setPosition(180);
            } else {
                realeaseservo.setPosition(redownpos); // back down
            }
        }
        lastYPressed = Controller.b;

         */
    }

     // set true when you want to start auto shooting

    public void runAutoShooter(boolean autoshoot) {
        if(autoshoot){
            switch (shootState) {
                case 0:
                    // Step 1: start cycle
                    setBallflick(0);            // push up
                    setHighroll(HighRolPow);
                    setIntake(-1);
                    shootTimer.reset();
                    shootState = 1;
                    break;

                case 1:
                    // Step 2: wait 200ms
                    if (shootTimer.milliseconds() >= 200) {
                        setBallflick(90);       // push down
                        shootTimer.reset();
                        shootState = 2;
                    }
                    break;

                case 2:
                    // Step 3: wait 450ms, then loop
                    if (shootTimer.milliseconds() >= 450) {
                        shootState = 0;         // repeat cycle
                    }
                    break;
            }
        }
    }
}

