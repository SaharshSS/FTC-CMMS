package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutonomousRoutineManager")
public class AutonomousRoutineManager extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize RoadRunner drive class (Assuming you're using SampleMecanumDrive)
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Set initial position on the field
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        // Define a trajectory to specific field points
        Trajectory testTrajectory = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(24, 24), Math.toRadians(90))  // Move to (24,24) facing 90°
                .splineTo(new Vector2d(-12, 36), Math.toRadians(180)) // Move to (-12,36) facing 180°
                .build();

        waitForStart(); // Wait for play button press

        if (isStopRequested()) return;

        // Follow the trajectory
        drive.followTrajectory(testTrajectory);

        // Log the final pose estimate after running the test
        Pose2d finalPose = drive.getPoseEstimate();
        telemetry.addData("Final X", finalPose.getX());
        telemetry.addData("Final Y", finalPose.getY());
        telemetry.addData("Final Heading", Math.toDegrees(finalPose.getHeading()));
        telemetry.update();
    }
}
