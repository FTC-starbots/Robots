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
public class AutoBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //___________________________trajetórias A_________________________
        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(32, -25), Math.toRadians(110))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end(), true)
                .splineTo(new Vector2d(35, -30), Math.toRadians(-90))
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2a.end(), false)
                .splineTo(new Vector2d(60, -20), Math.toRadians(90))
                .build();
        Trajectory traj4a = drive.trajectoryBuilder(traj3a.end(), false)
                .splineTo(new Vector2d(40, 60), Math.toRadians(180))
                .build();
        Trajectory traj5a = drive.trajectoryBuilder(traj4a.end(), false)
                .splineTo(new Vector2d(30, 78), Math.toRadians(90))
                .build();
        Trajectory traj6a = drive.trajectoryBuilder(traj5a.end(), true)
                .splineTo(new Vector2d(25, 74), Math.toRadians(180))
                .build();
        //__________________________trajetórias B__________________________
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(34, -6), Math.toRadians(0))
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end(), true)
                .splineTo(new Vector2d(26, -30), Math.toRadians(-90))
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2b.end(), false)
                .splineTo(new Vector2d(60, -10), Math.toRadians(90))
                .build();
        Trajectory traj4b = drive.trajectoryBuilder(traj3b.end(), false)
                .splineTo(new Vector2d(40, 60), Math.toRadians(180))
                .build();
        Trajectory traj5b = drive.trajectoryBuilder(traj4b.end(), false)
                .splineTo(new Vector2d(25, 78), Math.toRadians(90))
                .build();
        Trajectory traj6b = drive.trajectoryBuilder(traj5b.end(), true)
                .splineTo(new Vector2d(20, 74), Math.toRadians(180))
                .build();
        //________________________trajetórias C__________________________
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(32, 0), Math.toRadians(90))
                .build();
        Trajectory traj1c2 = drive.trajectoryBuilder(traj1c.end(), false)
                .splineTo(new Vector2d(32, 2), Math.toRadians(90))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c2.end(), true)
                .splineTo(new Vector2d(26, -20), Math.toRadians(-90))
                .build();
        Trajectory traj3c = drive.trajectoryBuilder(traj2c.end(), false)
                .splineTo(new Vector2d(60, 0), Math.toRadians(90))
                .build();
        Trajectory traj4c = drive.trajectoryBuilder(traj3c.end(), false)
                .splineTo(new Vector2d(40, 60), Math.toRadians(180))
                .build();
        Trajectory traj5c = drive.trajectoryBuilder(traj4c.end(), false)
                .splineTo(new Vector2d(24, 82), Math.toRadians(90))
                .build();
        Trajectory traj6c = drive.trajectoryBuilder(traj5c.end(), true)
                .splineTo(new Vector2d(22, 78), Math.toRadians(180))
                .build();
        //_________________________________________________________________
        waitForStart();

        if (isStopRequested()) return;


        int traj_selection = 3;

        if(traj_selection == 1) {
            drive.followTrajectory(traj1a);
            drive.followTrajectory(traj2a);
            drive.followTrajectory(traj3a);
            drive.followTrajectory(traj4a);
            drive.followTrajectory(traj5a);
            drive.followTrajectory(traj6a);
        } else if (traj_selection == 2) {
            drive.followTrajectory(traj1b);
            drive.followTrajectory(traj2b);
            drive.followTrajectory(traj3b);
            drive.followTrajectory(traj4b);
            drive.followTrajectory(traj5b);
            drive.followTrajectory(traj6b);

        }else if (traj_selection == 3) {
            drive.followTrajectory(traj1c);
            drive.followTrajectory(traj1c2);
            drive.followTrajectory(traj2c);
            drive.followTrajectory(traj3c);
            drive.followTrajectory(traj4c);
            drive.followTrajectory(traj5c);
            drive.followTrajectory(traj6c);

        }
        sleep(2000);

    }
}
