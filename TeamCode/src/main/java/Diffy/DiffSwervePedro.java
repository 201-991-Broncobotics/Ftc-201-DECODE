package Diffy;

import com.pedropathing.Drivetrain;
import com.pedropathing.VectorCalculator; // or whatever package it actually is
import com.pedropathing.math.Vector;

import Diffy.DiffySwerveKinematics;

public class DiffSwervePedro extends Drivetrain {

    private final DiffySwerveKinematics kinematics;

    public DiffSwervePedro(DiffySwerveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    @Override
    public double[] calculateDrive(
            Vector correctiveVector,
            Vector headingVector,
            Vector centripetalVector,
            double currentHeading) {

        double forward = correctiveVector.getYComponent();
        double strafe  = correctiveVector.getXComponent();
        double turn    = headingVector.getMagnitude(); // or magnitude depending on Pedro version

        return new double[]{forward, strafe, turn, 1.0};
    }

    @Override
    public void runDrive(double[] driveValues) {
        kinematics.drive(
                driveValues[0],
                driveValues[1],
                driveValues[2],
                driveValues[3]
        );
    }

    @Override
    public void startTeleopDrive() {

    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {

    }

    @Override
    public double xVelocity() {
        return 0;
    }

    @Override
    public double yVelocity() {
        return 0;
    }

    @Override
    public void setXVelocity(double xMovement) {

    }

    @Override
    public void setYVelocity(double yMovement) {

    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public String debugString() {
        return "";
    }

    @Override
    public void updateConstants() {
        // If you have drivetrain-specific constants, update them here.
        // For now, if you don't need to change anything, you can leave it empty:
    }

    @Override
    public void breakFollowing() {

    }

}
