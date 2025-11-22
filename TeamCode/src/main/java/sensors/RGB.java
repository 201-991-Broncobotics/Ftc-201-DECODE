package sensors;

import static mechanisms.Settings.SERVO_RANGE;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import mechanisms.Settings;

public class RGB {
    private Servo rgbServo;

    public void init(HardwareMap hwdM, Gamepad controller) {
        rgbServo = hwdM.get(Servo.class, "rgb");
    }

    /**
     * Sets the servo position based on degrees.
     * Assumes the user's "degree" values map to a standard range (defined in Settings).
     */
    public void setRgbDegrees(double degrees) {
        double position = degrees / SERVO_RANGE;
        // Clamp to safe range 0.0 - 1.0
        position = Math.max(0.0, Math.min(1.0, position));
        rgbServo.setPosition(position);
    }

    // Helper methods for specific states
    public void showRedDefault() { setRgbDegrees(Settings.RGB_RedDefault); }
    public void showBlueDefault() { setRgbDegrees(Settings.RGB_BlueDefault); }

    public void showRedWait() { setRgbDegrees(Settings.RGB_RedWait); }
    public void showBlueWait() { setRgbDegrees(Settings.RGB_BlueWait); }

    public void showRedReady() { setRgbDegrees(Settings.RGB_RedReady); }
    public void showBlueReady() { setRgbDegrees(Settings.RGB_BlueReady); }

    public void showRedShoot() { setRgbDegrees(Settings.RGB_RedShoot); }
    public void showBlueShoot() { setRgbDegrees(Settings.RGB_BlueShoot); }

    public void showRainbow() { setRgbDegrees(Settings.RGB_Rainbow); }
}