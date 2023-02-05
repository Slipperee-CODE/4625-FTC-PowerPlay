/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.lang.*;


@TeleOp(name="CleanestGreenestMeanestTeleop", group="Iterative Opmode")


//@Disabled


public class CleanerTeleop extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor spoolMotor = null;
    private Servo coneGrabber = null;


    private float SpeedReduction = 50;


    double y;
    double x;
    double rx;


    double linearSlideY;
    double theMaxPowerOfTheLinearSlide = 1;
    double theMinPowerOfTheLinearSlide = -theMaxPowerOfTheLinearSlide;


    int clawServoState = 0;

    public static double servoClawReleasedPos = 0.75;
    public static double servoClawPulledInPos = -0.75;

    boolean activeSlides = false;

    int encoderTicksForFullExtension = 3763;


    @Override
    public void init()
    {
        LeftFront = hardwareMap.dcMotor.get("LeftFront"); // 2
        LeftBack = hardwareMap.dcMotor.get("LeftBack"); // 3
        RightFront = hardwareMap.dcMotor.get("RightFront"); // 0
        RightBack = hardwareMap.dcMotor.get("RightBack"); // 1

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spoolMotor = hardwareMap.dcMotor.get("spoolMotor"); //

        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        coneGrabber = hardwareMap.servo.get("coneGrabber");


        SpeedReduction = SpeedReduction/100;


        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() { }


    @Override
    public void start() { runtime.reset(); }


    @Override
    public void loop()
    {
        y = -gamepad1.left_stick_y; // Remember, this is reversed!
        x = gamepad1.left_stick_x;
        rx = -gamepad1.right_stick_x;

        linearSlideY = gamepad2.left_stick_y;


        grabberServoCode();
        linearSlideCode();
        movementCode();
    }


    public void movementCode()
    {
        if (gamepad1.dpad_up){
            y = Math.min(SpeedReduction,1);
        }
        else if (gamepad1.dpad_down){
            y = Math.max(-SpeedReduction,-1);
        }
        else if (gamepad1.dpad_left){
            x = Math.max(-SpeedReduction*1.2,-1);
        }
        else if (gamepad1.dpad_right){
            x = Math.min(SpeedReduction*1.2,1);
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x - rx) / denominator;
        double backLeftPower = (y - x - rx) / denominator;
        double frontRightPower = (y - x + rx) / denominator;
        double backRightPower = (y + x + rx) / denominator;


        if (gamepad1.right_bumper){

            LeftFront.setPower(frontLeftPower);
            LeftBack.setPower(backLeftPower);
            RightFront.setPower(frontRightPower);
            RightBack.setPower(backRightPower);

        }
        else {

            LeftFront.setPower(frontLeftPower*SpeedReduction);
            LeftBack.setPower(backLeftPower*SpeedReduction);
            RightFront.setPower(frontRightPower*SpeedReduction);
            RightBack.setPower(backRightPower*SpeedReduction);

        }


        if (gamepad2.a){
            int motorPosition = LeftFront.getCurrentPosition();
            telemetry.addData("MotorPos Left Front",motorPosition);
            motorPosition = RightFront.getCurrentPosition();
            telemetry.addData("MotorPos Right Front",motorPosition);
            motorPosition = RightBack.getCurrentPosition();
            telemetry.addData("MotorPos Right Back",motorPosition);
            motorPosition = LeftBack.getCurrentPosition();
            telemetry.addData("MotorPos Left Back",motorPosition);
            motorPosition = spoolMotor.getCurrentPosition();
            telemetry.addData("MotorPos Spool Motor",motorPosition);


            telemetry.update();
        }


    }


    public void grabberServoCode()
    {
        if (gamepad2.b)
        {
            if (clawServoState == 1)
            {
                coneGrabber.setPosition(servoClawPulledInPos);
                telemetry.addLine("Servo 1 Thing");
                telemetry.update();
            }
            else if (clawServoState == 0)
            {
                coneGrabber.setPosition(servoClawReleasedPos);
                telemetry.addLine("Servo 2 Thing");
                telemetry.update();
            }

            if (clawServoState < 1)
            {
                clawServoState++;
            }
            else
            {
                clawServoState = 0;
            }

            sleep(500);
        }
    }


    public void linearSlideCode()
    {
        if (linearSlideY > 0){
            activeSlides = false;
            spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spoolMotor.setPower(Math.min(linearSlideY, theMaxPowerOfTheLinearSlide));
        }
        else if (linearSlideY < 0){
            activeSlides = false;
            spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spoolMotor.setPower(Math.max(linearSlideY, theMinPowerOfTheLinearSlide));
        }
        else if (gamepad2.dpad_up){
            activeSlides = false;
            spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spoolMotor.setPower(-theMaxPowerOfTheLinearSlide/2);
        }
        else if (gamepad2.dpad_down){
            activeSlides = false;
            spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spoolMotor.setPower(theMaxPowerOfTheLinearSlide/2);
        }
        else if (!gamepad1.dpad_up && !gamepad2.dpad_down && linearSlideY == 0 && !activeSlides)
        {
            spoolMotor.setPower(0);
        }

        if (gamepad2.x){
            activeSlides = true;
            spoolMotor.setTargetPosition(0);
            spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spoolMotor.setPower(1);
        }

        if (gamepad2.y){
            activeSlides = true;
            spoolMotor.setTargetPosition(-encoderTicksForFullExtension);
            spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spoolMotor.setPower(-1);
        }
    }


    public void sleep(int millis){
        try {
            Thread.sleep(millis);
        } catch (Exception e){}
    }


    @Override
    public void stop()
    {

    }
}
