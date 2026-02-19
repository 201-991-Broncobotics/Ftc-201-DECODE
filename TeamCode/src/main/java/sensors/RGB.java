package sensors;

import static mechanisms.Settings.rgb_default;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RGB {
    private Servo Rgb;
    public void init(HardwareMap hwdM, Gamepad controller) {
        Rgb = hwdM.get(Servo.class, "Rgb");

    }
    public void setRgb(double rgb) {
        Rgb.setPosition(rgb);
    }
}
