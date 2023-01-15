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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="WebcamLinearOpMode", group="Linear Opmode")
@Disabled
public class WebcamLinearOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;

    private float SpeedReduction = 50;

    OpenCvWebcam webcam1 = null;

    int colorIAmSeeing = 0;

    double actualAvg1;
    double actualAvg2;
    double actualAvg3;


    @Override
    public void runOpMode() {
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        SpeedReduction = SpeedReduction / 100;


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");

        boolean initLoop = true;

        while (!opModeIsActive()) {
            telemetry.addData("G", actualAvg1);
            telemetry.addData("Y", actualAvg2);
            telemetry.addData("P", actualAvg3);


            if (actualAvg1 > 6 || actualAvg2 > 6 || actualAvg3 > 6) {
                if (actualAvg1 > actualAvg2 && actualAvg1 > actualAvg3) {
                    colorIAmSeeing = 1;
                } else if (actualAvg2 > actualAvg1 && actualAvg2 > actualAvg3) {
                    colorIAmSeeing = 2;
                } else if (actualAvg3 > actualAvg1 && actualAvg3 > actualAvg2) {
                    colorIAmSeeing = 3;
                } else {
                    colorIAmSeeing = 0;
                }
            }

            if (colorIAmSeeing == 1) {
                telemetry.addData("Green", colorIAmSeeing);
            } else if (colorIAmSeeing == 2) {
                telemetry.addData("Yellow", colorIAmSeeing);
            } else if (colorIAmSeeing == 3) {
                telemetry.addData("Purple", colorIAmSeeing);
            } else {
                telemetry.addData("No Color :( ", colorIAmSeeing);
            }

            /*
            if (opModeIsActive()){
                telemetry.addLine("We leaving this loop fr fr");
                initLoop = false;
            }
            */


            telemetry.update();
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        webcam1.closeCameraDevice();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (colorIAmSeeing == 1) {
                //strafe left and forward
                telemetry.addLine("Green has been found");

                Move("left",1000,0.6);
                Move("forward",300,0.6);

                requestOpModeStop();

            } else if (colorIAmSeeing == 2) {
                //Strafe right and forward
                telemetry.addLine("Yellow has been found");

                Move("right",1000,0.6);
                Move("forward",300,0.6);

                requestOpModeStop();

            } else if (colorIAmSeeing == 3) {
                //Go forward
                telemetry.addLine("Purple has been found");


                Move("forward",300,0.6);

                requestOpModeStop();

            }

            telemetry.update();
            idle();
        }
    }

    public void Move(String direction, int milliseconds, double wheelPower){
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

    public class examplePipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // Make a working copy of the input matrix in HSV
            Mat mat = new Mat();
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


            // NOTE: In OpenCV's implementation,
            // Hue values are half the real value
            Scalar lowHSV1 = new Scalar(60, 100, 100);
            Scalar highHSV1 = new Scalar(71, 255, 255);
            Mat thresh = new Mat();


            Core.inRange(mat, lowHSV1, highHSV1, thresh);

            Scalar avg1 = Core.mean(thresh);

            actualAvg1 = avg1.val[0];


            Scalar lowHSV2 = new Scalar(20, 100, 100);
            Scalar highHSV2 = new Scalar(40, 255, 255);
            thresh = new Mat();


            Core.inRange(mat, lowHSV2, highHSV2, thresh);

            Scalar avg2 = Core.mean(thresh);

            actualAvg2 = avg2.val[0];


            Scalar lowHSV3 = new Scalar(144, 100, 100);
            Scalar highHSV3 = new Scalar(155, 255, 255);
            thresh = new Mat();


            Core.inRange(mat, lowHSV3, highHSV3, thresh);

            Scalar avg3 = Core.mean(thresh);

            actualAvg3 = avg3.val[0];


            return input;
        }
    }
}
