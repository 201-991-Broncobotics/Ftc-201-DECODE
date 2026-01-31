package mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    // internal variables
    private boolean shooting = false;
    private int shootState = 0;
    private ElapsedTime shootTimer = new ElapsedTime();

    // Hardware
    Gamepad Controller;
    private DcMotor intake, highroll;
    private Servo ballflick;
    private CRServo lowerroller1, lowerroller2;
    private Servo SortServo, realeaseservo;

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

    // Helper methods to keep code clean
    public void setHighroll(double power) { highroll.setPower(power); }
    public void setIntake(double power) { intake.setPower(power); }
    public void setBallflick(double pos) { ballflick.setPosition(pos); }
    public void lowerRollers(double power) {
        lowerroller1.setPower(power);
        lowerroller2.setPower(power);
    }

    public void control() {
        // --- 1. B BUTTON: Auto-Shooting Sequence (While Held) ---
        if (Controller.b) {
            shooting = true;
            realeaseservo.setPosition(Settings.redownpos);
            SortServo.setPosition(1);

            switch (shootState) {
                case 0:
                    // Step 1: Flick Up
                    setBallflick(0.43);
                    shootTimer.reset();
                    shootState = 1;
                    break;

                case 1:
                    // Step 2: Wait 220ms then Flick Down
                    setHighroll(0);
                    if (shootTimer.milliseconds() >= 220) {
                        setBallflick(0.95); // Push down
                        shootTimer.reset();
                        shootState = 2;
                    }
                    break;

                case 2:
                    // Step 3: Run Motors to shoot
                    setHighroll(-1);
                    setIntake(-1);
                    lowerRollers(-1);

                    // Wait 800ms before looping back to start
                    if (shootTimer.milliseconds() >= 800) {
                        shootState = 0;
                    }
                    break;
            }
        }
        else {
            // --- B RELEASED: Reset Shooter ---
            shooting = false;
            shootState = 0;
            setBallflick(0.95); // Reset flicker down
            realeaseservo.setPosition(Settings.reuppos); // Reset release
        }

        // --- 2. A & X BUTTONS: Manual Intake (Only if not shooting) ---
        // If we are shooting (B is held), we ignore A and X so they don't fight.
        if (!shooting) {
            if (Controller.a) {
                // HOLD A: Run Intake
                setIntake(Settings.IntakePowe);
                setHighroll(Settings.HighRolPow);
                lowerRollers(-1);
            }
            else if (Controller.x) {
                // HOLD X: Reverse/Outtake
                setIntake(-Settings.IntakePowe);
                setHighroll(-Settings.HighRolPow);
                lowerRollers(1);
            }
            else {
                // RELEASED: STOP EVERYTHING
                // This fixes the "keeps spinning" issue
                setIntake(0);
                setHighroll(0);
                lowerRollers(0);
            }
        }
    }
}