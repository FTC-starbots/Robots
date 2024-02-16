package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoRedRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //___________________________trajetórias A_________________________
        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, -16.5), Math.toRadians(-90))
                .splineTo(new Vector2d(32, -16.5), Math.toRadians(90))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end(), true)
                .splineTo(new Vector2d(32, -25.5), Math.toRadians(90))
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2a.end(), true)
                .splineTo(new Vector2d(4, -41), Math.toRadians(180))
                .build();
        //__________________________trajetórias B__________________________
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(34, -5), Math.toRadians(0))
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end(), true)
                .splineTo(new Vector2d(32, -37), Math.toRadians(90))
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2b.end(), true)
                .splineTo(new Vector2d(4, -47.5), Math.toRadians(180))
                .build();
        //________________________trajetórias C__________________________
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(32, 1.5), Math.toRadians(90))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c.end(), true)
                .splineTo(new Vector2d(32, -30), Math.toRadians(90))
                .build();
        Trajectory traj3c = drive.trajectoryBuilder(traj2c.end(), true)
                .splineTo(new Vector2d(6, -42), Math.toRadians(180))
                .build();
        //_________________________________________________________________
        waitForStart();

        if (isStopRequested()) return;


        int traj_selection = 2;

        if(traj_selection == 1) {
            drive.followTrajectory(traj1a);
            drive.followTrajectory(traj2a);
            drive.followTrajectory(traj3a);
        } else if (traj_selection == 2) {
            drive.followTrajectory(traj1b);
            drive.followTrajectory(traj2b);
            drive.followTrajectory(traj3b);
        }else if (traj_selection == 3) {
            drive.followTrajectory(traj1c);
            drive.followTrajectory(traj2c);
            drive.followTrajectory(traj3c);

        }
        sleep(2000);

    }
}
