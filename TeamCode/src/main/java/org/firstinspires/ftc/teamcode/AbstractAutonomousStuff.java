package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



public abstract class AbstractAutonomousStuff //NO CLUE HOW TO DO THIS
{
    private BNO055IMU imu;
    private Orientation lastAngle = new Orientation();
    private double currentAngle = 0.0;

    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;



    public void Move(String direction, int milliseconds, double wheelPower, DcMotor RightFront, DcMotor RightBack, DcMotor LeftFront, DcMotor LeftBack){
        if (direction == "forward" || direction == "backward"){
            if (direction == "backward"){
                wheelPower = -wheelPower;
            }

            LeftFront.setPower(wheelPower);
            LeftBack.setPower(wheelPower);
            RightFront.setPower(wheelPower);
            RightBack.setPower(wheelPower);
            sleep(milliseconds);
            LeftFront.setPower(0);
            LeftBack.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);

        }
        else if (direction == "left" || direction == "right"){
            if (direction == "left"){
                wheelPower = -wheelPower;
            }

            LeftFront.setPower(wheelPower);
            LeftBack.setPower(-wheelPower);
            RightFront.setPower(-wheelPower);
            RightBack.setPower(wheelPower);
            sleep(milliseconds);
            LeftFront.setPower(0);
            LeftBack.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);
        }
    }


    public void sleep(int millis){
        try {
            Thread.sleep(millis);
        } catch (Exception e){}
    }
}
