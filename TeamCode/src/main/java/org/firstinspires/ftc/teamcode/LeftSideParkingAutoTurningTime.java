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

@Autonomous(name="Left Side Parking Auto WITH TURNING", group="Linear Autonomous")
public class LeftSideParkingAutoTurningTime extends LinearOpMode
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

        spoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
            Move("left",350,0.6);//For now just moving forward lol
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            if (tagOfInterest.id == ID_TAGS_OF_INTEREST[0]) //First Image
            {
                MainAutoCode();
                Move("forward", 1000,0.6);
            }
            else if(tagOfInterest.id == ID_TAGS_OF_INTEREST[1]) //Second Image
            {
                MainAutoCode();
                Move("forward", 350,0.6);
            }
            else if (tagOfInterest.id == ID_TAGS_OF_INTEREST[2]) //Third Image
            {
                MainAutoCode();
                Move("backward", 800,0.6);
            }
        }
    }

    void MainAutoCode()
    {
        MoveSlides("up", 1000, 0.8);

        Move("forward", 100, 0.5);

        MoveSlides("down", 1000, 0.8);

        sleep(500);

        coneGrabber.setPosition(servoClawReleasedPos);

        sleep(500);

        MoveSlides("up", 2000, 0.8);

        Move("left", 120, 0.5);
        Move("forward", 2300, 0.5);
        Move("backward", 400, 0.5);

        sleep(500);

        Turn(-45);

        MoveSlides("up", 2600, 0.8);

        Move("forward",125,0.5);

        sleep(200);

        MoveSlides("down", 400, 0.8);

        sleep(500);

        coneGrabber.setPosition(servoClawPulledInPos);

        sleep(500);

        MoveSlides("up", 400, 0.8);

        sleep(100);

        Move("backward",125,0.5);

        sleep(100);

        MoveSlides("down", 2600, 0.8);

        Turn(135);

        MoveSlides("up",2000,0.8);




        //PAST THIS POINT THE CODE IS FULLY UNTESTED AND COULD RUN THE ROBOT STRAIGHT INTO SEVERAL WALLS
        sleep(500);

        Move("forward",1000,0.5);

        sleep(300);

        MoveSlides("down",1000,0.8);

        sleep(500);

        coneGrabber.setPosition(servoClawReleasedPos);

        sleep(500);

        MoveSlides("up",1000,0.8);

        sleep(300);

        Move("backward",1000,0.5);

        sleep(300);

        Turn(-135);

        sleep(300);

        Move("forward",125,0.5);

        sleep(200);

        MoveSlides("down", 400, 0.8);

        sleep(500);

        coneGrabber.setPosition(servoClawPulledInPos);

        sleep(500);

        MoveSlides("up", 400, 0.8);

        sleep(100);

        Move("backward",125,0.5);

        sleep(100);

        MoveSlides("down", 1000, 0.8);

        Turn(135);
    }

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

    void MoveSlides(String direction, int milliseconds, double wheelPower)
    {
        if (direction == "up"){
            wheelPower = -wheelPower;
        }

        spoolMotor.setPower(wheelPower);
        sleep(milliseconds);
        telemetry.addData("This","this");
        telemetry.update();
        spoolMotor.setPower(0);
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

    void Turn (double degrees)
    {
        ResetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2){
            double motorPower = (error < 0 ? -0.2 : 0.2);
            PowerAllTheMotors(motorPower, motorPower, -motorPower, -motorPower);
            error = degrees - GetAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        PowerAllTheMotors(0,0,0,0);

    }
}