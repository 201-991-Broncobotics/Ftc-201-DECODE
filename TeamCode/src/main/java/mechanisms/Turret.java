package mechanisms;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import sensors.Limelight;

public class Turret {
    private CRServo turret1, turret2; // Declare a continuous rotation serv
    Gamepad Controller;
    boolean autotracktoggle, lastbumppressed = false;
    private Telemetry telemetry;
    private Limelight limelight;
// o

    public void init(HardwareMap hdwMap, Gamepad controller) {
        turret1 = hdwMap.get(CRServo.class, "turS1");
        turret2 = hdwMap.get(CRServo.class, "turS2");
        Controller = controller;


    }

    public void setTurrets(double power1, double power2) {
        turret1.setPower(power1);
        turret2.setPower(power2);
    }

    public void controls() {
        if (Controller.right_trigger > 0.1) {
            setTurrets(0.15, 0.15); // Spin forward based on trigger pressure
        } else if (Controller.left_trigger > 0.1) {
            setTurrets(-0.15, -0.15); // Spin forward based on trigger pressure

               } else {
              setTurrets(0, 0);         }

        }
        // public void autoTrack(double power) {
        // turret1.setPower(power);
        //  turret2.setPower(power);
    }


// }