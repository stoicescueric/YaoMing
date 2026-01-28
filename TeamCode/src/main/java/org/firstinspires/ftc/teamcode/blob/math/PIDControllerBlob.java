package org.firstinspires.ftc.teamcode.blob.math;

import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayDeque;
import java.util.Deque;

public class PIDControllerBlob {


    public double kp,ki,kd,ff1=0,ff2=0;
    ElapsedTime timer=new ElapsedTime();

    public PIDControllerBlob(double kp , double ki , double kd){
        this.kp=kp;
        this.ki=ki;
        this.kd=kd;
        timer.startTime();
        timer.reset();
    }
    public PIDControllerBlob(double kp , double ki , double kd , double ff1 , double ff2){
        this.kp=kp;
        this.ki=ki;
        this.kd=kd;
        this.ff1=ff1;
        this.ff2=ff2;
        timer.startTime();
        timer.reset();
    }

    private static class IntegralPart{
        public ElapsedTime timer =new ElapsedTime();
        public double value=0;
        public IntegralPart(double value)
        {
            this.value=value;
            timer.startTime();
            timer.reset();
        }
    }

    public double integralBound=0.1;
    private final Deque<IntegralPart> integralParts=new ArrayDeque<>();

    private double lastError=0;
    private double integralSum=0;
    public double calculate(double reference , double state){

        if(Double.isNaN(reference) || Double.isNaN(state) || Double.isNaN(timer.seconds()))return 0;

        double error=reference-state;
        integralSum+=timer.seconds()*error;

        integralParts.addLast(new IntegralPart(error));
        while(integralParts.getFirst().timer.seconds()>integralBound)
        {
            IntegralPart temp=integralParts.getFirst();
            integralParts.removeFirst();
            integralSum-=temp.value*(temp.timer.seconds()-integralParts.getFirst().timer.seconds());
        }

        double derivative=(error-lastError)/timer.seconds();
        double output=kp*error+kd*derivative;
         output+=integralSum*ki;
        output+=ff1+ff2*state;
        timer.reset();
        lastError=error;
        return output;
    }

    public double calculateMecanum(double reference , double state){
        double error=reference-state;


        double derivative=(error-lastError)/timer.seconds();
        double output=kp*error+kd*derivative;
        timer.reset();
        lastError=error;
        return output;
    }

}