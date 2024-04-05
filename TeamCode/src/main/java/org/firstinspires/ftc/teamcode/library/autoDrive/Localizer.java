package org.firstinspires.ftc.teamcode.library.autoDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.component.Imu;
import org.firstinspires.ftc.teamcode.component.OldImu;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Localizer {

    private Encoder rightEncoder;
    private Encoder frontEncoder;
    private Encoder leftEncoder;

    private OldImu imu;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.689; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 3.5; // in; distance between the left and right wheels 4.125
    public static double FORWARD_OFFSET = 3.5; // in; offset of the lateral wheel
//public static double FORWARD_OFFSET = -1.1811; // in; offset of the lateral wheel

    private double x;
    private double y;
    private double xPrev;
    private double yPrev;


    private double lastHeading;
    private double lastX;
    private double lastY;

    private double rawX;
    private double rawY;
    private double heading; // radians

    private double r0;
    private double r1;

    private double relX;
    private double relY;

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction
    private ElapsedTime time;
    private double elapsedTime;
    private double velocity;
    private double avgVel;
    private double lastVelocity;
    private double acceleration;

    // Making arrays of velocities and acc to get a non-noisy average of last 10 values
    public double[] velocities;
    public double[] accelerations;

    public final int length = 10;
    private int index = 0;

    public Localizer(LinearOpMode opMode, HardwareMap hardwareMap, boolean twoWheel){
        velocities = new double[length];
        accelerations = new double[length];
        time = new ElapsedTime();
        imu = new OldImu(hardwareMap);
        imu.initImuThread(opMode);

        reset();
    }

    public Localizer(LinearOpMode opMode, HardwareMap hardwareMap){

        imu = new OldImu(hardwareMap);
        imu.initImuThread(opMode);
        velocities = new double[length];
        accelerations = new double[length];

        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rr"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lf"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rf"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        leftEncoder.setDirection(Encoder.Direction.REVERSE);

        rightEncoder.reset();
        frontEncoder.reset();
        leftEncoder.reset();

        reset();
    }

    public void reset(){
        x = 0;
        y = 0;

        lastHeading = 0;
        lastX = 0;
        lastY = 0;

    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public void update(){
        elapsedTime = time.seconds();
        time.reset();
        calculateRawValues();

        if(heading -lastHeading == 0){
            relX = (rawX -lastX);
            relY = (rawY -lastY);
        }else{
            r0 = (rawX -lastX) / (heading - lastHeading);
            r1 = (rawY -lastY) / (heading - lastHeading);

            relX = r0 * Math.sin(heading -lastHeading) - r1 * (1 - Math.cos(heading -lastHeading));
            relY = r1 * Math.sin(heading -lastHeading) + r0 * (1 - Math.cos(heading -lastHeading));
        }

        x += relX * Math.cos(heading) - relY * Math.sin(heading);
        y += relY * Math.cos(heading) + relX * Math.sin(heading);

        velocity = Math.sqrt(
                Math.pow((x-xPrev)/elapsedTime, 2) + Math.pow((y-yPrev)/elapsedTime, 2)
        );

        // Average velocity - running average of 10 values
        if (index<length) {
            velocities[index] = getRawVelocity();
            index++;
        }
        else {
            index = 0;
        }
        double sum = 0;
        for (int i =0; i<length;i++) {
            sum+=velocities[i];
        }
        lastVelocity = avgVel;
        avgVel = sum/length;

        // Getting average acceleration - running sum of 10
        double rawAcc = (avgVel-lastVelocity)/elapsedTime;
        if (index<length) {
            accelerations[index] = rawAcc;
            index++;
        }
        else {
            index = 0;
        }
        // Re-using the previous sum variable
        sum = 0;
        for (int i =0; i<length;i++) {
            sum+=accelerations[i];
        }
        acceleration = sum/length;

        xPrev = x;
        yPrev = y;
        lastX = rawX;
        lastY = rawY;

        lastHeading = heading;
    }

    public void calculateRawValues(){
        double rawX = (getRightEncoderPosition() + getLeftEncoderPosition())/2.0;
        double heading = (getRightEncoderPosition() - getLeftEncoderPosition())/(LATERAL_DISTANCE);
        double rawY = getFwdEncoderPosition() - (FORWARD_OFFSET * heading);

        setRawValues(rawX, rawY, heading);
    }

    public double getLastHeading(){
        return lastHeading;
    }

    public void setRawValues(double x, double y, double head){
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


    public double getHeading(){
        return heading;
    }

    public double getRawX(){
        return rawX;
    }

    public double getRawY(){
        return rawY;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private double getRightEncoderPosition(){
        return encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    private double getLeftEncoderPosition(){
        return encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    private double getFwdEncoderPosition(){
        return encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER;
    }

    public void initImu(){
        imu.initIMU();
    }

    public enum Angle{
        RADIANS,
        DEGREES,
    }

    public double getHeadingImu() {
        return imu.getCurrentHeading();
    }

    public double getAngularVelocityImu() {
        return imu.getAngularVelocity();
    }

    public String getTelemetry() {
        return "First heading: " + imu.getCurrentHeading();
    }

    public double getRawVelocity() {
        return velocity;
    }

    public double getAvgVelocity() {
        return avgVel;
    }

    public double getAcc() {
        return acceleration;
    }
}