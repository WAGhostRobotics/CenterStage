package org.firstinspires.ftc.teamcode.library.autoDrive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class HansenPKLocalizer {
    private Encoder rightEncoder, frontEncoder, leftEncoder;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 12; // MEASURE in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 7; // MEASURE in; offset of the lateral wheel

    private double x, y ,lastHeading, lastX, lastY, rawX, rawY, heading, r0, r1, relX, relY;


    private static double X_MULTIPLIER = 1, Y_MULTIPLIER = 1; //multiplier in the X and Y direciton
    private ElapsedTime time;
    private double elapsedTime;


    public HansenPKLocalizer(LinearOpMode opMode, HardwareMap hardwareMap, boolean twoWheel) {
        time = new ElapsedTime();

        reset();
    }

    public HansenPKLocalizer(LinearOpMode opMode, HardwareMap hardwareMap) {

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rr"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lf"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));

        //TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE);

        rightEncoder.reset();
        frontEncoder.reset();
        leftEncoder.reset();

        reset();
    }

    public void reset() {
        x = y = lastHeading = lastX = lastY = 0;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void update() {
        elapsedTime = time.seconds();
        time.reset();
        calculateRawValues();

        double headingChange = heading - lastHeading;
        double diffX = rawX - lastX;
        double diffY = rawY - lastY;
        if(headingChange == 0) {
            relX = diffX;
            relY = diffY;
        } else {
            r0 = diffX / headingChange;
            r1 = diffY / headingChange;

            relX = r0 * Math.sin(headingChange) - r1 * (1 - Math.cos(headingChange));
            relY = r1 * Math.sin(headingChange) + r0 * (1 - Math.cos(headingChange));
        }

        x += relX * Math.cos(heading) - relY * Math.sin(heading);
        y += relY * Math.cos(heading) + relX * Math.sin(heading);

        lastX = rawX;
        lastY = rawY;

        lastHeading = heading;
    }

    public void calculateRawValues() {
        double rawX = (getRightEP() + getLeftEP()) / 2.0;
        double heading = (getRightEP() - getLeftEP())/LATERAL_DISTANCE;
        double rawY = getFwdEP() - FORWARD_OFFSET * heading;

        setRawValues(rawX, rawY, heading);
    }

    public double getLastHeading() {
        return lastHeading;
    }

    public void setRawValues(double x, double y, double head) {
        rawX = x;
        rawY = y;
        heading = head;
    }

    public double getHeading(Angle angle) {
        if (angle == Angle.RADIANS) {
            return getHeading();
        } else {
            return Math.toDegrees(getHeading());
        }
    }

    public double getHeading() {
        return heading;
    }

    public double getRawX() {
        return rawX;
    }

    public double getRawY() {
        return rawY;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private double getRightEP() {
        return encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }
    private double getLeftEP() {
        return encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }
    private double getFwdEP() {
        return encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER;
    }
    public enum Angle{
        RADIANS, DEGREES,
    }
    public String getTelemetry() {
        return "First heading: " + heading + "\n" + "X: " + x + "\n" + "Y: " + y;
    }
}
