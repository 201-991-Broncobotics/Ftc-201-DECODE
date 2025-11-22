package mechanisms;

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

public class Intake {
    // Hardware
    private DcMotor intake, highroll;
    private Servo ballflick;
    private CRServo lowerroller1, lowerroller2;
    private Servo SortServo, realeaseservo;

    // State Variables
    private ElapsedTime shootTimer = new ElapsedTime();
    private int shootState = 0;
    private boolean shooting = false;
    private boolean releaseToggle = false;
    private boolean lastYPressed = false;

    Gamepad Controller;

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

    public void control() {
        // --- INTAKE CONTROLS ---
        if (Controller.a) {
            // Holding A: Run Intake
            intake.setPower(IntakePowe);
            setHighroll(HighRolPow);
            lowerRollers(-1);
        }
        else if (Controller.x) {
            // Holding X: Reverse Intake
            intake.setPower(-IntakePowe);
            lowerRollers(1);
            setHighroll(-HighRolPow);
        }
        else if (!shooting) {
            // If not holding A, not holding X, and not shooting cycle: STOP
            intake.setPower(0);     // <--- FIXED: Was 1 in your code
            setHighroll(0);         // <--- FIXED: Was .5 in your code
            lowerRollers(0);        // <--- FIXED: Was -.3 in your code
            realeaseservo.setPosition(redownpos);
        }

        // --- SHOOTING SEQUENCE (Button B) ---
        if (Controller.b && !shooting) {
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }

        // Cancel shot if B is released?
        // (Based on your logic, you seemed to want it to only run while B is held,
        // but usually shooting is a "One Press" action. I kept your logic here.)
        if (!Controller.b) {
            shooting = false;
            shootState = 0;
            setBallflick(90);   // Reset flicker
            // We rely on the "else" block in the Intake Controls above to stop the motors
        }

        if (shooting) {
            switch (shootState) {
                case 0:
                    setBallflick(-20);       // Flick Up
                    setHighroll(HighRolPow);
                    setIntake(0);
                    shootTimer.reset();
                    shootState = 1;
                    break;

                case 1:
                    if (shootTimer.milliseconds() >= 200) {
                        setBallflick(80);   // Flick Down
                        shootTimer.reset();
                        shootState = 2;
                    }
                    break;

                case 2:
                    if (shootTimer.milliseconds() >= 450) {
                        shootState = 0; // Repeat
                    }
                    break;
            }
            lowerRollers(-1);
            realeaseservo.setPosition(reuppos);
        }
    }
}