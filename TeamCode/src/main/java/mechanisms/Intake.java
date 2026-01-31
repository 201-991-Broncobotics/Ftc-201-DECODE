package mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private boolean shooting = false;
    private int shootState = 0;
    private ElapsedTime shootTimer = new ElapsedTime();

    Gamepad Controller;
    private DcMotor intake, highroll;
    private Servo ballflick, SortServo, realeaseservo;
    private CRServo lowerroller1, lowerroller2;

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

    public void setHighroll(double power) { highroll.setPower(power); }
    public void setIntake(double power) { intake.setPower(power); }
    public void setBallflick(double pos) { ballflick.setPosition(pos); }
    public void lowerRollers(double power) {
        lowerroller1.setPower(power);
        lowerroller2.setPower(power);
    }

    public void control() {
        // --- SHOOTING LOGIC (LATCHED) ---
        if (Controller.b && !shooting) {
            shooting = true;
            shootState = 0;
            shootTimer.reset();
        }

        if (shooting) {
            // Always hold these during the sequence
            realeaseservo.setPosition(Settings.redownpos); // OPEN GATE
            SortServo.setPosition(1);

            switch (shootState) {
                case 0: // NEW: Wait for Gate Servo to Open
                    setBallflick(Settings.flick_DOWN); // Keep flicker out of way

                    if (shootTimer.milliseconds() >= Settings.shoot_delay_ms) {
                        shootTimer.reset();
                        shootState = 1;
                    }
                    break;

                case 1: // Flick Up (Preparation)
                    setBallflick(Settings.flick_UP);

                    if (shootTimer.milliseconds() >= 220) {
                        shootTimer.reset();
                        shootState = 2;
                    }
                    break;

                case 2: // Flick Down & Shoot
                    setBallflick(Settings.flick_DOWN);
                    setHighroll(-1);
                    setIntake(-1);
                    lowerRollers(-1);

                    if (shootTimer.milliseconds() >= 800) {
                        shooting = false; // Sequence Done
                        shootState = 0;
                    }
                    break;
            }
        }
        else {
            // --- IDLE STATE ---
            setBallflick(Settings.flick_DOWN);
            realeaseservo.setPosition(Settings.reuppos); // CLOSE GATE

            // --- MANUAL INTAKE ---
            if (Controller.a) {
                setIntake(Settings.IntakePowe);
                setHighroll(Settings.HighRolPow);
                lowerRollers(-1);
            }
            else if (Controller.x) {
                setIntake(-Settings.IntakePowe);
                setHighroll(-Settings.HighRolPow);
                lowerRollers(1);
            }
            else {
                setIntake(0);
                setHighroll(0);
                lowerRollers(0);
            }
        }
    }
}