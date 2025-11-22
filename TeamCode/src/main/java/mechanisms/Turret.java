package mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private CRServo turret1, turret2;
    Gamepad Controller;

    public void init(HardwareMap hdwMap, Gamepad controller) {
        // Make sure these names match your Config on the Driver Hub exactly!
        turret1 = hdwMap.get(CRServo.class, "turS1");
        turret2 = hdwMap.get(CRServo.class, "turS2");
        Controller = controller;
    }

    /**
     * Sets turret power with Voltage Compensation.
     * @param power - The desired power (-1.0 to 1.0)
     * @param currentVoltage - The current battery voltage (read from OpMode)
     */
    public void setTurretsWithVoltageComp(double power, double currentVoltage) {
        // Nominal voltage is 12.0V.
        // If battery is 14V, we multiply power by (12/14) = 0.85 (Slows it down)
        // If battery is 11V, we multiply power by (12/11) = 1.09 (Speeds it up)
        double compensatedPower = power * (12.0 / currentVoltage);

        // Clamp to ensure we don't exceed limits
        compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

        turret1.setPower(compensatedPower);
        turret2.setPower(compensatedPower);
    }

    // Fallback for manual testing without voltage logic
    public void setTurrets(double power1, double power2) {
        turret1.setPower(power1);
        turret2.setPower(power2);
    }
}