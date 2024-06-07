package org.firstinspires.ftc.teamcode.library.drivetrain;

import org.firstinspires.ftc.teamcode.library.autoDrive.Localizer;

public interface Drivetrain {
    public void driveMax(double magnitude, double theta, double driveTurn, double movementPower);
    public void drive(double magnitude, double theta, double driveTurn, double movementPower);

    public default void driveMax(double magnitude, double theta, double driveTurn, double movementPower, double voltage) {

    }

    public default void drive(double magnitude, double theta, double driveTurn, double movementPower, double voltage) {

    }
//    public default void brake(Localizer localizer, double movementPower, double heading) {
//
//    }
    public default void brake(Localizer localizer, double driveTurn, double movementPower) {

    }
    public default void brake(Localizer localizer, double movementPower) {

    }


    public String getTelemetry();
}
