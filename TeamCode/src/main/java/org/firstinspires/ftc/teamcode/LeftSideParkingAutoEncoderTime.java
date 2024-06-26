/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Left Side Parking Auto WITH ENCODERS", group="Linear Autonomous")
public class LeftSideParkingAutoEncoderTime extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    //FIX THESE WHEN YOU CAN!!!!!!!!!!!!!!!!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    // UNITS ARE METERS
    double tagsize = 0.0635;

    //^^^^^^^^^^^^^ FIX THESE WHEN YOU CAN!!!!!!!!!!!!!!!!

    int[] ID_TAGS_OF_INTEREST = {0, 1, 2};

    AprilTagDetection tagOfInterest = null;

    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor spoolMotor = null;
    private Servo coneGrabber = null;

    private float SpeedReduction = 50;

    public static double servoClawReleasedPos = 0.7;
    public static double servoClawPulledInPos = -0.7;

    private BNO055IMU imu;



    private Orientation lastAngle = new Orientation();
    private double currentAngle = 0.0;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");
        coneGrabber = hardwareMap.servo.get("coneGrabber");
        spoolMotor = hardwareMap.dcMotor.get("spoolMotor");

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO0155IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu"); //We still need to name the imu on the robot this
        imu.initialize(parameters);

        SpeedReduction = SpeedReduction/100;

        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        spoolMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    for (int tagId : ID_TAGS_OF_INTEREST)
                    {
                        if(tag.id == tagId)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }

                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */


            MainAutoCode();
            Turn(90 + 44);

            sleep(100);

            Move("left", 450, 0.3);
            //TestingCode();
        }
        else
        {
            if (tagOfInterest.id == ID_TAGS_OF_INTEREST[0]) //First Image
            {
                MainAutoCode();
                Turn(90 + 42);
                Move("left", 425, 0.3);
                sleep(300);
                Move("forward",.94,0.6, true);
            }
            else if(tagOfInterest.id == ID_TAGS_OF_INTEREST[1]) //Second Image
            {
                MainAutoCode();
                Turn(90 + 44);

                sleep(100);

                Move("left", 425, 0.3);
            }
            else if (tagOfInterest.id == ID_TAGS_OF_INTEREST[2]) //Third Image
            {
                MainAutoCode();
                Turn(-40);

                sleep(100);

                Move("right", 630, 0.15);
                sleep(200);
                Move("forward",1.0,0.6, true);
            }
        }
    }

    void TestingCode(){
        Move("backward", 1, 0.5, true);

        sleep(500);
    }


    void MainAutoCode()
    {
        int waitBetweenMovement = 200;

        //Slides Up
        MoveSlides(0.2,0.8);

        Move("forward",.15, 0.3, true);

        sleep(waitBetweenMovement + 750);

        //Slides Down
        MoveSlides(0.02,0.8);

        sleep(waitBetweenMovement);

        //Grab Cone
        coneGrabber.setPosition(servoClawReleasedPos);
        telemetry.addData("yoo",coneGrabber.getPosition());
        telemetry.update();

        sleep(waitBetweenMovement);

        //Slides Up a Bit
        MoveSlides(0.25,0.8);

        Move("forward",2.05, 0.7, true); //THIS LINE REQUIRES THE GRADIENT

        sleep(waitBetweenMovement);

        Move("backward",.05,0.5, false);

        sleep(waitBetweenMovement);

        Turn(-44);

        sleep(waitBetweenMovement);

        //Slides Up
        MoveSlides(.9,0.8);

        Move("forward",0.4, 0.5, false);

        Move("backward",0.03,0.5, false);

        sleep(waitBetweenMovement + 300);

        //Slides Down
        MoveSlides(.77,0.8);

        sleep(waitBetweenMovement + 300);

        //Release Cone
        coneGrabber.setPosition(servoClawPulledInPos);

        sleep(waitBetweenMovement + 150);

        //Slides Up
        MoveSlides(.9,0.8);

        Move("backward",0.345, 0.5, false);

        MoveSlides(.7,0.8);
    }

    void Move(String direction, double distanceInTiles, double maxPower, boolean forwardGradient){ //ADD IN BOOL FOR GRADIENT FORWARDS, USE A SIN WAVE
        double wheelDiameter = 3.77953; //in
        double wheelCircumference = wheelDiameter * Math.PI;
        double tileLength = 24; //in
        double ticksPerRev = 537.7;
        double startingGradientPower = 0.1;

        double fullWheelRotationForHowMuchOfATile = wheelCircumference/tileLength;

        double rotationsNeeded = distanceInTiles/fullWheelRotationForHowMuchOfATile;

        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int encoderTicksNeeded = (int)(ticksPerRev*rotationsNeeded);

        if (direction == "backward")
        {
            encoderTicksNeeded = -encoderTicksNeeded;
            maxPower = -maxPower;
        }

        RightFront.setTargetPosition(encoderTicksNeeded);
        RightBack.setTargetPosition(encoderTicksNeeded);
        LeftFront.setTargetPosition(encoderTicksNeeded);
        LeftBack.setTargetPosition(encoderTicksNeeded);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (forwardGradient) {
            if (direction == "backward"){
                PowerAllTheMotors(-startingGradientPower,-startingGradientPower,-startingGradientPower,-startingGradientPower);
            }else{
                PowerAllTheMotors(startingGradientPower,startingGradientPower,startingGradientPower,startingGradientPower);
            }

        }
        else {
            PowerAllTheMotors(maxPower,maxPower,maxPower,maxPower);
        }


        while (LeftFront.isBusy() || RightFront.isBusy() || LeftBack.isBusy() || RightBack.isBusy()){
            if (forwardGradient) {
                double avgEncoderTick = (LeftBack.getCurrentPosition() + LeftBack.getCurrentPosition() + LeftBack.getCurrentPosition() + LeftBack.getCurrentPosition())/4;
                double distanceTraveledPercent = Math.min(avgEncoderTick/encoderTicksNeeded,1);
                double powerForAllMotors = Math.min(Math.sqrt(Math.sin(distanceTraveledPercent * Math.PI)) + startingGradientPower, maxPower);

                if (direction == "backward")
                {
                    powerForAllMotors = -powerForAllMotors;
                }

                PowerAllTheMotors(powerForAllMotors,powerForAllMotors,powerForAllMotors,powerForAllMotors);

                telemetry.addData("Power For All Motors On the Straight", powerForAllMotors);
                telemetry.update();
            }
        }

        sleep(200);
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


    void MoveSlides(double percentOfMaxHeight, double power)
    {
        double encoderTicksForFullExtension = 3763;

        double encoderTicksNeededFromStart = percentOfMaxHeight * encoderTicksForFullExtension;

        double encoderTicksNeededToMove = encoderTicksNeededFromStart - spoolMotor.getCurrentPosition();

        double targetEncoderTicks = spoolMotor.getCurrentPosition() + encoderTicksNeededToMove;

        spoolMotor.setTargetPosition((int) targetEncoderTicks);

        spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power = (encoderTicksNeededToMove>0) ? (power):(-power);

        spoolMotor.setPower(power);

        while (spoolMotor.isBusy()){
            //Do nothing
        }

        sleep(200);
    }


    public void sleep(int millis){
        try {
            Thread.sleep(millis);
        } catch (Exception e){}
    }


    void PowerAllTheMotors(double RightFrontPower, double RightBackPower, double LeftFrontPower, double LeftBackPower)
    {
        RightFront.setPower(RightFrontPower);
        RightBack.setPower(RightBackPower);
        LeftFront.setPower(LeftFrontPower);
        LeftBack.setPower(LeftBackPower);
    }

    void ResetAngle()
    {
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    double GetAngle()
    {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;

        }
        else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currentAngle += deltaAngle;
        lastAngle = orientation;
        telemetry.addData("gyro rotation", orientation.firstAngle);
        return currentAngle;
    }

    void Move(String direction, int milliseconds, double wheelPower){
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

    void Turn (double degrees)
    {
        ResetAngle();

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 1){
            double motorPower = (error < 0 ? -0.3 : 0.3);
            PowerAllTheMotors(motorPower, motorPower, -motorPower, -motorPower);
            error = degrees - GetAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        PowerAllTheMotors(0,0,0,0);

    }
}