package sensors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import mechanisms.Settings;
import sensors.PIDFController;

public class Limelight {
    Gamepad Controller;
    private Limelight3A limelight;
    private LLResult results;

    public final PIDFController turretPID = new PIDFController(0, 0, 0, 0);

    public void init(HardwareMap hwdM, Gamepad controller, int pipeline) {
        limelight = hwdM.get(Limelight3A.class, "liml");
        limelight.pipelineSwitch(pipeline);
        try { results = limelight.getLatestResult(); } catch (Exception e) { }
        Controller = controller;
    }

    public void start() { if(limelight != null) limelight.start(); }
    public LLResult getResult() { return (limelight != null) ? limelight.getLatestResult() : null; }

    public double getTx() {
        if(limelight == null) return 0;
        results = limelight.getLatestResult();
        return (results != null && results.isValid()) ? results.getTx() : 0;
    }

    public double getYaw() {
        if (limelight == null) return 0;
        results = limelight.getLatestResult();
        if (results == null || !results.isValid()) return 0;
        List<LLResultTypes.FiducialResult> fids = results.getFiducialResults();
        if (fids.isEmpty()) return 0;
        return fids.get(0).getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES);
    }

    public double getDistance() {
        if(limelight == null) return 0;
        results = limelight.getLatestResult();

        if (results != null && results.isValid()) {
            double verticalOffset = results.getTy();

            double totalAngleDeg = Settings.CAMERA_ANGLE_DEG + verticalOffset;
            double totalAngleRad = Math.toRadians(totalAngleDeg);

            double heightDiff = Settings.TARGET_HEIGHT_IN - Settings.CAMERA_HEIGHT_IN;

            if (Math.abs(Math.tan(totalAngleRad)) < 0.001) return 0;

            double rawDistance = heightDiff / Math.tan(totalAngleRad);

            // FIX: Account for camera offset so turning doesn't change distance
            return rawDistance - Settings.CAMERA_OFFSET_FROM_CENTER;
        }
        return 0;
    }
}