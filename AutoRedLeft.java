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
public class AutoRedLeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //___________________________trajetórias A_________________________
        Trajectory traj1a = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(32, 19.5), Math.toRadians(-90))
                .build();
        Trajectory traj2a = drive.trajectoryBuilder(traj1a.end(), true)
                .splineTo(new Vector2d(26, 27), Math.toRadians(-90))
                .build();
        Trajectory traj3a = drive.trajectoryBuilder(traj2a.end(), false)
                .splineTo(new Vector2d(26, 32.2), Math.toRadians(90))
                .build();
        Trajectory traj4a = drive.trajectoryBuilder(traj3a.end(), true)
                .splineTo(new Vector2d(26, 30.8), Math.toRadians(-90))
                .splineTo(new Vector2d(53, 30.8), Math.toRadians(90))
                .build();
        Trajectory traj5a = drive.trajectoryBuilder(traj4a.end(), false)
                .splineTo(new Vector2d(53, -60), Math.toRadians(-90))
                .splineTo(new Vector2d(35, -75), Math.toRadians(-90))
                .build();
        Trajectory traj6a = drive.trajectoryBuilder(traj5a.end(), true)
                .splineTo(new Vector2d(3, -82), Math.toRadians(180))
                .build();
        //__________________________trajetórias B__________________________
        Trajectory traj1b = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(34, 3), Math.toRadians(0))
                .build();
        Trajectory traj2b = drive.trajectoryBuilder(traj1b.end(), true)
                .splineTo(new Vector2d(29, 3), Math.toRadians(180))
                .splineTo(new Vector2d(29, 24), Math.toRadians(-90))
                .build();
        Trajectory traj3b = drive.trajectoryBuilder(traj2b.end(), false)
                .splineTo(new Vector2d(29, 31.25), Math.toRadians(90))
                .build();
        Trajectory traj4b = drive.trajectoryBuilder(traj3b.end(), true)
                .splineTo(new Vector2d(29, 30), Math.toRadians(-90))
                .splineTo(new Vector2d(53, 30), Math.toRadians(90))
                .build();
        Trajectory traj5b = drive.trajectoryBuilder(traj4b.end(), false)
                .splineTo(new Vector2d(53, -60), Math.toRadians(-90))
                .splineTo(new Vector2d(30, -75), Math.toRadians(-90))
                .build();
        Trajectory traj6b = drive.trajectoryBuilder(traj5b.end(), true)
                .splineTo(new Vector2d(3, -82), Math.toRadians(180))
                .build();
        //________________________trajetórias C__________________________
        Trajectory traj1c = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(32, -3), Math.toRadians(-90))
                .build();
        Trajectory traj2c = drive.trajectoryBuilder(traj1c.end(), true)
                .splineTo(new Vector2d(23.25, 5), Math.toRadians(-90))
                .build();
        Trajectory traj3c = drive.trajectoryBuilder(traj2c.end(), false)
                .splineTo(new Vector2d(23.25, 30), Math.toRadians(90))
                .build();
        Trajectory traj4c = drive.trajectoryBuilder(traj3c.end(), true)
                .splineTo(new Vector2d(23.25, 29), Math.toRadians(-90))
                .splineTo(new Vector2d(53, 29), Math.toRadians(90))
                .build();
        Trajectory traj5c = drive.trajectoryBuilder(traj4c.end(), false)
                .splineTo(new Vector2d(53, -60), Math.toRadians(-90))
                .splineTo(new Vector2d(35, -75), Math.toRadians(-90))
                .build();
        Trajectory traj6c = drive.trajectoryBuilder(traj5c.end(), true)
                .splineTo(new Vector2d(3, -82), Math.toRadians(180))
                .build();
        //_________________________________________________________________
        waitForStart();

        if (isStopRequested()) return;

        //selecionar com posição do cone

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
            drive.followTrajectory(traj2c);
            drive.followTrajectory(traj3c);
            drive.followTrajectory(traj4c);
            drive.followTrajectory(traj5c);
            drive.followTrajectory(traj6c);

        }
        sleep(2000);

    }
}
