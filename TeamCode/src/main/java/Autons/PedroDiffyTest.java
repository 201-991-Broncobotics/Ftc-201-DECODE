package Autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import pedropathing.Constants;

@Autonomous(name = "Pedro Diffy Test", group = "Test")
public class PedroDiffyTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Follower follower = Constants.createFollower(hardwareMap);

        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        waitForStart();
        if (isStopRequested()) return;

        PathChain forwardPath = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(0, 0, 0),
                                new Pose(0, 24, 0)
                        )
                )
                .build();

        follower.followPath(forwardPath);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }

        follower.breakFollowing();
    }
}
