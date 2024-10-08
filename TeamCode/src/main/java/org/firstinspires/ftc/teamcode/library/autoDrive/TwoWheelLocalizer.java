package org.firstinspires.ftc.teamcode.library.autoDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.component.Imu;
import org.firstinspires.ftc.teamcode.util.Encoder;


public class TwoWheelLocalizer extends Localizer {

    private Encoder parallelEncoder;
    private Encoder perpendicularEncoder;

    public static double PERPENDICULAR_X = 3;
    public static double PARALLEL_Y = 0;
    //-8, -8.5
    //-3.088
    //-8.696

    public TwoWheelLocalizer(LinearOpMode opMode, HardwareMap hardwareMap){
        super(opMode, hardwareMap, true);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lb"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "lf"));

        parallelEncoder.setDirection(Encoder.Direction.REVERSE);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        parallelEncoder.reset();
        perpendicularEncoder.reset();

    }

    @Override
    public void calculateRawValues(){
        double headingChange = getHeading() - normalizeRadians(getLastHeading());
        if(headingChange > Math.PI){
            headingChange -= 2 * Math.PI;
        }else if(headingChange<-Math.PI){
            headingChange += 2 * Math.PI;
        }

        double heading = getLastHeading() + headingChange;
        double rawX = getParallelEncoderPosition() + (PARALLEL_Y * heading);
        double rawY = getPerpendicularEncoderPosition() - (PERPENDICULAR_X * heading);

        setRawValues(rawX, rawY, heading);
    }

    private double getParallelEncoderPosition(){
        return encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER;
    }

    private double getPerpendicularEncoderPosition(){
        return encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER;
    }

    @Override
    public double getHeading(){
        return Math.toRadians(getHeadingImu());
    }
}