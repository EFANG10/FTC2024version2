package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.drive.DriveConstants;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TimingConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="autoright")
//@Disabled
public class roadrunnertest extends LinearOpMode {

    // Motor assignments
    DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    DcMotor leftextend, lefttilt, rightextend, righttilt;

    // Roadrunner Drive
    MecanumDrive drive;

    public void init(HardwareMap hardwareMap) {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right");
        leftextend = hardwareMap.get(DcMotor.class, "leftextend");
        lefttilt = hardwareMap.get(DcMotor.class, "lefttilt");
        rightextend = hardwareMap.get(DcMotor.class, "rightextend");
        righttilt = hardwareMap.get(DcMotor.class, "righttilt");

        // Initialize Roadrunner drive system (assuming a mecanum drive)
        drive = new MecanumDrive(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        // Set motor directions if necessary (adjust according to your robot configuration)
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void driveInCircle() {
        // Create a trajectory for a small circle
        // Define radius of the circle (e.g., 12 inches) and angular velocity
        double radius = 12.0; // Example radius for the circle
        double angularVelocity = Math.toRadians(90); // 90 degrees per second

        // Define the trajectory, where we start at Pose(0, 0, 0) (centered and facing the right)
        Trajectory circleTrajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(radius, 0, Math.PI / 2)) // Creates an arc path
                .build();

        // Follow the trajectory
        drive.followTrajectory(circleTrajectory);
    }

    public void stop() {
        drive.setMotorPowers(0, 0, 0, 0);
    }
}
