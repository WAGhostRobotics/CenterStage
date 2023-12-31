package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.ArrayList;

public class AnalogEncoder {

    private boolean reverse = false;

    private AnalogInput encoder;
    private double zero = 0;
    //    private double range = 3.385;
    private double range = 3.273;
    private double last=Double.NaN;


    public ArrayList<Double> log = new ArrayList<Double>();
    private double maxLogSize = 15;


    public AnalogEncoder(AnalogInput encoder){
        this.encoder = encoder;
    }

    public AnalogEncoder(AnalogInput encoder, double zero, boolean reverse) {
        this.encoder = encoder;
        this.zero = zero;
        this.reverse = reverse;
    }




    public void zero(){
        zero = getCurrentPosition();
    }
    public double getZero(){
        return zero;
    }
    public void setZero(double zero){
        this.zero = zero;
    }
    public void reverse(boolean reverse){
        this.reverse = reverse;
    }
    public boolean getDirection() {
        return reverse;
    }

    public double getCurrentPosition() {

        double pos;

        double offset = -Math.toRadians(zero);

        pos = Angle.norm(((reverse?(1- getVoltage()/range):(getVoltage()/range))) * 2 * Math.PI + offset);

        return pos;

    }


    public double getCurrentPositionMedianFilter(){
        double pos;


        double offset = -Math.toRadians(zero);


        pos = normalizeRadians(((reverse?(1- getVoltage()/range):(getVoltage()/range))) * 2 * Math.PI + offset);


        if(log.size()<maxLogSize){
            log.add(pos);
        }else{
            log.remove(0);
            log.add(pos);
        }


        return getMedian(log);

    }

    public double getMedian(ArrayList<Double> log){
        ArrayList<Double> sorted = quicksort(log);

        if(sorted.size()%2 != 0){
            return sorted.get(sorted.size()/2);
        }else{
            return (sorted.get(sorted.size()/2) + sorted.get((sorted.size()-1)/2))/2;
        }
    }


    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }

    public ArrayList<Double> quicksort(ArrayList<Double> input){

        if(input.size() <= 1){
            return input;
        }

        int middle = (int) Math.ceil((double)input.size() / 2);
        double pivot = input.get(middle);

        ArrayList<Double> less = new ArrayList<Double>();
        ArrayList<Double> greater = new ArrayList<Double>();

        for (int i = 0; i < input.size(); i++) {
            if(input.get(i) <= pivot){
                if(i == middle){
                    continue;
                }
                less.add(input.get(i));
            }
            else{
                greater.add(input.get(i));
            }
        }

        return concatenate(quicksort(less), pivot, quicksort(greater));
    }

    private ArrayList<Double> concatenate(ArrayList<Double> less, double pivot, ArrayList<Double> greater){

        ArrayList<Double> list = new ArrayList<Double>();

        for (int i = 0; i < less.size(); i++) {
            list.add(less.get(i));
        }

        list.add(pivot);

        for (int i = 0; i < greater.size(); i++) {
            list.add(greater.get(i));
        }

        return list;
    }
}
