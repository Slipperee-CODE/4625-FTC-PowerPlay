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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Left Side Parking Auto", group="Linear Autonomous")
public class LeftSideParkingAuto extends LinearOpMode
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

        SpeedReduction = SpeedReduction/100;

        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
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


            Move("forward",250,1); //For now just moving forward lol
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            if (tagOfInterest.id == ID_TAGS_OF_INTEREST[0]) //First Image
            {


                spoolMotor.setPower(-0.6);
                sleep(500);
                spoolMotor.setPower(0);

                Move("forward", 100, 0.5);

                spoolMotor.setPower(0.6);
                sleep(500);
                spoolMotor.setPower(0);

                sleep(500);

                coneGrabber.setPosition(servoClawReleasedPos);

                sleep(500);

                spoolMotor.setPower(-0.6);
                sleep(1000);
                spoolMotor.setPower(0);

                Move("left", 120, 0.5);
                Move("forward", 2300, 0.5);
                Move("backward", 400, 0.5);

                sleep(500);

                Move("right", 500, 0.5);

                spoolMotor.setPower(-0.6);
                sleep(1750);
                spoolMotor.setPower(0);

                Move("forward",125,0.5);

                sleep(200);

                spoolMotor.setPower(0.6);
                sleep(250);
                spoolMotor.setPower(0);

                sleep(500);

                coneGrabber.setPosition(servoClawPulledInPos);

                sleep(500);

                spoolMotor.setPower(-0.6);
                sleep(250);
                spoolMotor.setPower(0);

                sleep(100);

                Move("backward",125,0.5);

                sleep(100);

                spoolMotor.setPower(0.6);
                sleep(1750);
                spoolMotor.setPower(0);

                //Move("left",1100,0.6);
                //Move("forward",400,0.7);
            }
            else if(tagOfInterest.id == ID_TAGS_OF_INTEREST[1]) //Second Image
            {

                spoolMotor.setPower(-0.6);
                sleep(500);
                spoolMotor.setPower(0);

                Move("forward", 100, 0.5);

                spoolMotor.setPower(0.6);
                sleep(500);
                spoolMotor.setPower(0);

                sleep(500);

                coneGrabber.setPosition(servoClawReleasedPos);

                sleep(500);

                spoolMotor.setPower(-0.6);
                sleep(1000);
                spoolMotor.setPower(0);

                Move("left", 120, 0.5);
                Move("forward", 2300, 0.5);
                Move("backward", 400, 0.5);

                sleep(500);

                Move("right", 500, 0.5);

                spoolMotor.setPower(-0.6);
                sleep(1750);
                spoolMotor.setPower(0);

                Move("forward",125,0.5);

                sleep(200);

                spoolMotor.setPower(0.6);
                sleep(250);
                spoolMotor.setPower(0);

                sleep(500);

                coneGrabber.setPosition(servoClawPulledInPos);

                sleep(500);

                spoolMotor.setPower(-0.6);
                sleep(250);
                spoolMotor.setPower(0);

                sleep(100);

                Move("backward",125,0.5);

                sleep(100);

                spoolMotor.setPower(0.6);
                sleep(1750);
                spoolMotor.setPower(0);



                //Move("left", 125, 0.5);
                //Move("forward",400,0.7);
            }
            else if (tagOfInterest.id == ID_TAGS_OF_INTEREST[2]) //Third Image
            {

                spoolMotor.setPower(-0.6);
                sleep(500);
                spoolMotor.setPower(0);

                Move("forward", 100, 0.5);

                spoolMotor.setPower(0.6);
                sleep(500);
                spoolMotor.setPower(0);

                sleep(500);

                coneGrabber.setPosition(servoClawReleasedPos);

                sleep(500);

                spoolMotor.setPower(-0.6);
                sleep(1000);
                spoolMotor.setPower(0);

                Move("left", 120, 0.5);
                Move("forward", 2300, 0.5);
                Move("backward", 400, 0.5);

                sleep(500);

                Move("right", 500, 0.5);

                spoolMotor.setPower(-0.6);
                sleep(1750);
                spoolMotor.setPower(0);

                Move("forward",125,0.5);

                sleep(200);

                spoolMotor.setPower(0.6);
                sleep(250);
                spoolMotor.setPower(0);

                sleep(500);

                coneGrabber.setPosition(servoClawPulledInPos);

                sleep(500);

                spoolMotor.setPower(-0.6);
                sleep(250);
                spoolMotor.setPower(0);

                sleep(100);

                Move("backward",125,0.5);

                sleep(100);

                spoolMotor.setPower(0.6);
                sleep(1750);
                spoolMotor.setPower(0);


                //Move("right",700,0.6);
                //Move("forward",400,0.7);
            }
        }
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
}