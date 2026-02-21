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

    // NEW: Variables to track ball detection
    private int ballsDetected = 0;
    private boolean isRpmDipped = false;

    Gamepad Controller;
    private DcMotor intake, highroll;
    private Servo ballflick, SortServo, realeaseservo, pushservo1, pushservo2;
    private CRServo lowerroller1, lowerroller2;

    public void init(HardwareMap hwdM, Gamepad controller) {
        intake = hwdM.get(DcMotor.class, "intM");
        highroll = hwdM.get(DcMotor.class, "hRolM");
        pushservo1 = hwdM.get(Servo.class, "pusS0");
        pushservo2 = hwdM.get(Servo.class, "pusS1");
        lowerroller1 = hwdM.get(CRServo.class, "lRolS0");
        lowerroller2 = hwdM.get(CRServo.class, "lRolS1");
        SortServo = hwdM.get(Servo.class, "sorS");
        realeaseservo = hwdM.get(Servo.class, "relS");
        Controller = controller;
    }

    public void setHighroll(double power) { highroll.setPower(power); }
    public void setIntake(double power) { intake.setPower(power); }
    public void setPushservos(double pos) {
        pushservo1.setPosition(pos);
        pushservo2.setPosition(pos);
    }
    public void lowerRollers(double power) {
        lowerroller1.setPower(power);
        lowerroller2.setPower(-power);
    }

    // NEW: Pass currentRPM into the control method
    public void control(double currentRPM) {

        // --- CONTINUOUS HELD SHOOTING LOGIC ---
        if (Controller.b) {
            // Initialize sequence when B is first pressed
            if (!shooting) {
                shooting = true;
                shootState = 0;
                ballsDetected = 0;
                isRpmDipped = false;
                shootTimer.reset();
            }

            // Always hold these states while B is held to feed the balls
            realeaseservo.setPosition(Settings.redownpos); // OPEN GATE
            SortServo.setPosition(1);
            setHighroll(1);
            setIntake(-1);
            lowerRollers(-1);

            switch (shootState) {
                case 0: // Detect 2 Balls via RPM Drop
                    // If RPM drops below our threshold, register a dip
                    if (currentRPM < (Settings.fly_targetRPM - Settings.rpm_drop_amount)) {
                        if (!isRpmDipped) {
                            isRpmDipped = true;
                            ballsDetected++;
                        }
                    }
                    // Wait for RPM to recover before being able to detect the next ball
                    else if (currentRPM > (Settings.fly_targetRPM - Settings.rpm_recovery_amount)) {
                        isRpmDipped = false;
                    }

                    // Once 2 balls are detected, move to delay state
                    if (ballsDetected >= 2) {
                        shootTimer.reset();
                        shootState = 1;
                    }
                    break;

                case 1: // Wait specified variable delay
                    if (shootTimer.milliseconds() >= Settings.post_shot_delay_ms) {
                        shootState = 2;
                    }
                    break;

                case 2: // Move flick servo to 0 deg
                    setPushservos(0);
                    break;
            }
        } else {
            // --- IDLE / MANUAL STATE (Executes when B is released) ---
            shooting = false;
            realeaseservo.setPosition(Settings.reuppos); // CLOSE GATE

            if (Controller.a) {
                setIntake(Settings.IntakePowe);
                setHighroll(Settings.HighRolPow);
                lowerRollers(-1);
                setPushservos(90);
            } else if (Controller.x) {
                setIntake(-Settings.IntakePowe);
                setHighroll(-Settings.HighRolPow);
                lowerRollers(1);
                setPushservos(90);
            } else {
                setIntake(0);
                setHighroll(0);
                setPushservos(90);
                lowerRollers(0);
            }

            if (Controller.y) {
                setPushservos(Settings.flick_UP);
            }
        }
    }
}