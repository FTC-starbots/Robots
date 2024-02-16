package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoRedLeftReading extends LinearOpMode {

    boolean USE_WEBCAM;
    float x;
    float y;
    int traj_selection;
    List<Recognition> myTfodRecognitions;
    Recognition myTfodRecognition;
    TfodProcessor myTfodProcessor;
    VisionPortal myVisionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        USE_WEBCAM = true;
        initTfod();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

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
        telemetryTfod();
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        if (JavaUtil.listLength(myTfodRecognitions) > 0) {
            for (Recognition myTfodRecognition_item : myTfodRecognitions){
                if (myTfodRecognition_item.getLabel() == "RedMarker"){
                    x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                    y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                    if (x > 400) {
                        traj_selection = 3;
                    } else if (x < 400) {
                        traj_selection = 2;
                    }
                }
            }
        }
        else {
            traj_selection = 1;
        }

        sleep(1500);

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

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("MyCustomModel.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("BlueMarker", "RedMarker"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);

        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    private void telemetryTfod() {

        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            telemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
        }
    }
}
