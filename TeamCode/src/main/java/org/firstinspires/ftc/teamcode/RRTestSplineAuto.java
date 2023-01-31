package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RRTestSplineAuto extends LinearOpMode
{
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        /*
        Trajectory goForward = drivetrain.trajectoryBuilder(new Pose2d(0,0, 0))
                .forward(100)
                .back(10)
                .build();

        drivetrain.followTrajectory(goForward);


        Trajectory lineToPosition = drivetrain.trajectoryBuilder(new Pose2d (10, 0,10))
                .lineTo(new Vector2d(0,0))
                .build();

        drivetrain.followTrajectory(lineToPosition);


        Trajectory strafeLeft = drivetrain.trajectoryBuilder(new Pose2d(0,0 ,0))
                .strafeLeft(50)
                .build();

        drivetrain.followTrajectory(strafeLeft);


        Trajectory strafeToPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0,0 ))
                .strafeTo(new Vector2d(50, 0))
                .build();

        drivetrain.followTrajectory(strafeToPosition);


        Trajectory splineToPosition = drivetrain.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .splineTo(new Vector2d(50,50), Math.toRadians(90))
                .build();

        drivetrain.followTrajectory(splineToPosition);
        */

        Trajectory testFullMovementAutoPt1 = drivetrain.trajectoryBuilder(new Pose2d(38, -61.5, Math.toRadians(90)))
                .strafeLeft(4.5)
                .forward(50)
                .build();

        Trajectory testFullMovementAutoPt2 = drivetrain.trajectoryBuilder(testFullMovementAutoPt1.end())
                .forward(5)
                .back(6)
                .build();

        Trajectory testFullMovementAutoPt3 = drivetrain.trajectoryBuilder(testFullMovementAutoPt2.end())
                .forward(27)
                .back(27)
                .build();

        Trajectory testFullMovementAutoPt4 = drivetrain.trajectoryBuilder(testFullMovementAutoPt3.end())
                .forward(6)
                .back(6)
                .build();

        Trajectory testFullMovementAutoPt5 = drivetrain.trajectoryBuilder(testFullMovementAutoPt4.end())
                //.strafeLeft(22.5)
                .strafeRight(24)
                .build();



        drivetrain.followTrajectory(testFullMovementAutoPt1);
        drivetrain.turn(Math.toRadians(40));
        drivetrain.followTrajectory(testFullMovementAutoPt2);
        drivetrain.turn(Math.toRadians(-130));
        drivetrain.followTrajectory(testFullMovementAutoPt3);
        drivetrain.turn(Math.toRadians(130));
        drivetrain.followTrajectory(testFullMovementAutoPt4);
        drivetrain.turn(Math.toRadians(-40));
        drivetrain.followTrajectory(testFullMovementAutoPt5);
    }
}
