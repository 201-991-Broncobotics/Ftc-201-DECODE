// Imports for FTC OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "MichaGptOp")
public class MicahGptOp extends OpMode {

    // Motors
    DcMotor flyM, intM;
    DcMotor flyE, intE;
    CRServo turS1, turS2;
    CRServo lRolS, hRolS0, hRolS1;

    // Servos
    Servo sorS, relS, pusS, padS;
    Servo hanS0, hanS1;

    // Color Sensors
    ColorSensor colE0, colE1, colE2, colE3;

    // For RPM
    int prevFlyEnc = 0, prevIntEnc = 0;
    long lastRPMCheck = 0;
    double flyRPM = 0, intRPM = 0;
    double flySetSpeed = 0, intSetSpeed = 0;

    static final int ENCODER_TICKS_PER_REV = 28; // Update for your motor

    // --- PIDF GAINS ---
    // These constants are for the manual F+P controller.
    // YOU MUST TUNE THESE VALUES for your robot!
    // Start with small values (like these) and increase Kp until it oscillates, then back off.
    // Kf is roughly 1.0 / max_theoretical_RPM, but also needs tuning.
    static final double FLYWHEEL_KP = 0.0005; // Proportional gain
    static final double FLYWHEEL_KF = 0.00016; // Feedforward gain (approx 1.0 / 6000 RPM)

    static final double INTAKE_KP = 0.0001; // Proportional gain
    static final double INTAKE_KF = 0.00006; // Feedforward gain (approx 1.0 / 16000 RPM)


    // Toggle states:
    boolean releaseToggle = false, sortToggle = false,
            hang0Toggle = false, hang1Toggle = false,
            lowRollerToggle = false, highRoller0Toggle = false, highRoller1Toggle = false,
            pushToggle = false;

    @Override
    public void init() {
        flyM = hardwareMap.get(DcMotor.class, "flyM");
        intM = hardwareMap.get(DcMotor.class, "intM");
        flyE = flyM;
        intE = intM;

        turS1 = hardwareMap.get(CRServo.class, "turS1");
        turS2 = hardwareMap.get(CRServo.class, "turS2");
        lRolS = hardwareMap.get(CRServo.class, "lRolS");
        hRolS0 = hardwareMap.get(CRServo.class, "hRolS0");
        hRolS1 = hardwareMap.get(CRServo.class, "hRolS1");

        sorS = hardwareMap.get(Servo.class, "sorS");
        relS = hardwareMap.get(Servo.class, "relS");
        pusS = hardwareMap.get(Servo.class, "pusS");
        padS = hardwareMap.get(Servo.class, "padS");
        hanS0 = hardwareMap.get(Servo.class, "hanS0");
        hanS1 = hardwareMap.get(Servo.class, "hanS1");

        colE0 = hardwareMap.get(ColorSensor.class, "colE0");
        colE1 = hardwareMap.get(ColorSensor.class, "colE1");
        colE2 = hardwareMap.get(ColorSensor.class, "colE2");
        colE3 = hardwareMap.get(ColorSensor.class, "colE3");
        // ... color sensors, etc.
    }

    // Edge detector for each toggle button
    boolean prevA = false, prevB = false, prevX = false, prevY = false;
    boolean prevLeftBumper = false, prevRightBumper = false, prevDpadUp=false, prevDpadDown=false, prevDpadLeft=false, prevDpadRight=false;

    @Override
    public void loop() {
        // 1. Update the actual RPM values first
        updateRPM();

        // 2. Set Flywheel speed
        flySetSpeed += -gamepad1.right_stick_y * 2;
        flySetSpeed = Math.max(0, Math.min(6000, flySetSpeed));
        // This now uses the F+P controller
        setFlywheelRPM(flySetSpeed);

        // 3. Set Intake speed
        intSetSpeed += -gamepad1.left_stick_y * 2;
        intSetSpeed = Math.max(0, Math.min(16000, intSetSpeed));
        // This now uses the F+P controller
        setIntakeRPM(intSetSpeed);


        // 4. Turret rotation (servos)
        // This logic looks correct. If it's not spinning, check:
        // 1. Hardware connections
        // 2. Configuration names ("turS1", "turS2")
        // 3. That your CRServos are not in "Servo" mode on the hub
        if (gamepad1.dpad_left) {
            turS1.setPower(-1.0);
            turS2.setPower(-1.0);
        } else if (gamepad1.dpad_right) {
            turS1.setPower(1.0);
            turS2.setPower(1.0);
        } else {
            turS1.setPower(0);
            turS2.setPower(0);
        }

        // 5. Toggle servo actions

        // Release servo toggle (A)
        if (gamepad1.a && !prevA) releaseToggle = !releaseToggle;
        prevA = gamepad1.a;
        relS.setPosition(releaseToggle ? 1.0 : 0.0); // Set positions as desired

        // Sort servo toggle (B)
        if (gamepad1.b && !prevB) sortToggle = !sortToggle;
        prevB = gamepad1.b;
        sorS.setPosition(sortToggle ? 1.0 : 0.0);

        // Hang servos toggles (Y for hangServo0, X for hangServo1)
        if (gamepad1.y && !prevY) hang0Toggle = !hang0Toggle;
        prevY = gamepad1.y;
        hanS0.setPosition(hang0Toggle ? 1.0 : 0.0);

        if (gamepad1.x && !prevX) hang1Toggle = !hang1Toggle;
        prevX = gamepad1.x;
        hanS1.setPosition(hang1Toggle ? 1.0 : 0.0);

        // Low roller toggle (left bumper)
        if (gamepad1.left_bumper && !prevLeftBumper) lowRollerToggle = !lowRollerToggle;
        prevLeftBumper = gamepad1.left_bumper;
        lRolS.setPower(lowRollerToggle ? 1.0 : 0.0);

        // High roller 0 toggle (right bumper)
        // Toggle both high rollers with one button (right bumper)
        // Second servo runs in reverse direction
        if (gamepad1.right_bumper && !prevRightBumper) {
            highRoller0Toggle = !highRoller0Toggle;
        }
        prevRightBumper = gamepad1.right_bumper;
        hRolS0.setPower(highRoller0Toggle ? 1.0 : 0.0);   // normal direction
        hRolS1.setPower(highRoller0Toggle ? -1.0 : 0.0);  // reversed direction


        // Push servo toggle (D-pad down)
        if (gamepad1.dpad_down && !prevDpadDown) pushToggle = !pushToggle;
        prevDpadDown = gamepad1.dpad_down;
        pusS.setPosition(pushToggle ? 1.0 : 0.0);

        // Paddle servo: button to angle up, button to angle down (D-pad left/right)
        // --- !!! CONTROL CONFLICT !!! ---
        // Your turret code also uses D-pad Left and D-pad Right.
        // This conflict might be causing issues.
        // I've commented out the paddle controls to let the turret work.
        // You should assign the paddle to different buttons (e.g., D-pad Up).
        /*
        if (gamepad1.dpad_left && !prevDpadLeft) padS.setPosition(1.0); // up, tune to real angle
        if (gamepad1.dpad_right && !prevDpadRight) padS.setPosition(0.0); // down, tune to real angle
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        */
        // Don't forget to update all edge-detection variables, even if unused, to prevent stale presses
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevDpadUp = gamepad1.dpad_up;


        // Telemetry output (RPM and sensors)
        telemetry.addData("Flywheel Target RPM", flySetSpeed);
        telemetry.addData("Flywheel Actual RPM", flyRPM);
        telemetry.addData("Intake Target RPM", intSetSpeed);
        telemetry.addData("Intake Actual RPM", intRPM);
        telemetry.addData("Color0", colE0.red());
        telemetry.addData("Color1", colE1.red());
        telemetry.addData("Release", releaseToggle);
        telemetry.addData("Sort", sortToggle);
        telemetry.addData("Hang0", hang0Toggle);
        telemetry.addData("Hang1", hang1Toggle);
        telemetry.addData("LowRoller", lowRollerToggle);
        telemetry.addData("HighRoller0", highRoller0Toggle);
        telemetry.addData("HighRoller1", highRoller1Toggle);
        telemetry.addData("Push", pushToggle);
    }

    void updateRPM() {
        long now = System.currentTimeMillis();
        double deltaMs = now - lastRPMCheck;

        // Only update every 100ms
        if (deltaMs > 100) {
            int currFlyEnc = flyE.getCurrentPosition();
            int currIntEnc = intE.getCurrentPosition();

            double deltaTicksFly = currFlyEnc - prevFlyEnc;
            double deltaTicksInt = currIntEnc - prevIntEnc;

            // --- RPM Calculation Fix ---
            // The original calculation used 600.0, which was off by a factor of 100.
            // RPM = (ticks / ms) * (60000 ms_per_min / ticks_per_rev)
            // RPM = (deltaTicks / deltaMs) * (60000.0 / ENCODER_TICKS_PER_REV)
            // RPM = (deltaTicks * 60000.0) / (deltaMs * ENCODER_TICKS_PER_REV)
            flyRPM = (deltaTicksFly * 60000.0) / (deltaMs * ENCODER_TICKS_PER_REV);
            intRPM = (deltaTicksInt * 60000.0) / (deltaMs * ENCODER_TICKS_PER_REV);

            prevFlyEnc = currFlyEnc;
            prevIntEnc = currIntEnc;
            lastRPMCheck = now;
        }
    }

    // --- New F+P Controller for Flywheel ---
    void setFlywheelRPM(double targetRPM) {
        if (targetRPM <= 0) {
            flyM.setPower(0);
            return;
        }

        double currentRPM = flyRPM; // Get the latest measured RPM
        double error = targetRPM - currentRPM;

        // F-term (feedforward) + P-term (proportional)
        double feedforward = targetRPM * FLYWHEEL_KF;
        double proportional = error * FLYWHEEL_KP;

        double power = feedforward + proportional;

        // Clamp power to [0, 1.0] since flywheel only spins one way
        power = Math.max(0, Math.min(1.0, power));

        flyM.setPower(power);
    }

    // --- New F+P Controller for Intake ---
    void setIntakeRPM(double targetRPM) {
        if (targetRPM <= 0) {
            intM.setPower(0);
            return;
        }

        double currentRPM = intRPM; // Get the latest measured RPM
        double error = targetRPM - currentRPM;

        // F-term (feedforward) + P-term (proportional)
        double feedforward = targetRPM * INTAKE_KF;
        double proportional = error * INTAKE_KP;

        double power = feedforward + proportional;

        // Clamp power to [0, 1.0]
        power = Math.max(0, Math.min(1.0, power));

        intM.setPower(power);
    }
}
