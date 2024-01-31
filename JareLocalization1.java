package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override



    public void runOpMode() throws InterruptedException {

        DcMotorEx Pulso;
        Pulso = hardwareMap.get(DcMotorEx .class, "Pulso");
        Pulso.setDirection(DcMotor.Direction.FORWARD);
        Pulso.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Pulso.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        DcMotorEx Slide_D;
        Slide_D = hardwareMap.get(DcMotorEx .class, "Slide_D");
        Slide_D.setDirection(DcMotor.Direction.FORWARD);
        Slide_D.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide_D.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotorEx Slide_E;
        Slide_E = hardwareMap.get(DcMotorEx .class, "Slide_E");
        Slide_E.setDirection(DcMotor.Direction.FORWARD);
        Slide_E.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide_E.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double angulo_desejado = 0;
        double erro_angular = 0;
        int piddelay = 0;

        PIDCoefficients pidconfig_1 = new PIDCoefficients(3, 0.2 , 0.2);

        PIDFController angular_controler = new PIDFController(pidconfig_1);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d joystickInput = new Vector2d (gamepad1.left_stick_x, -gamepad1.left_stick_y )
                    .rotated(-poseEstimate.getHeading());
            Vector2d joystickraw = new Vector2d (gamepad1.left_stick_x, -gamepad1.left_stick_y);


            double robot_speed = 0.8;
            double angulo_atual = (poseEstimate.getHeading()*180/Math.PI);

            angular_controler.setTargetPosition(0.0);
            double correcao_angular = -angular_controler.update(erro_angular);

            if (gamepad1.left_trigger>0.1 || gamepad1.right_trigger>0.1){
                correcao_angular = (gamepad1.left_trigger-gamepad1.right_trigger);
                piddelay = 0;
            }

            if (piddelay < 20){
                angulo_desejado = angulo_atual;
                piddelay++;
            }
            if(gamepad1.right_bumper) angulo_desejado = -90+(joystickraw.angle()*180/Math.PI);

            double potencia_maxima_ang = 0.5;

            if(gamepad1.dpad_up) angulo_desejado = 0;
            else if(gamepad1.dpad_down) angulo_desejado = 180;
            else if(gamepad1.dpad_right) angulo_desejado = 270;
            else if(gamepad1.dpad_left) angulo_desejado = 90;



            if(correcao_angular>potencia_maxima_ang) correcao_angular=potencia_maxima_ang;
            else if(correcao_angular<-potencia_maxima_ang) correcao_angular=-potencia_maxima_ang;


            double et_power = ((joystickInput.getY()*0.8-joystickInput.getX())*robot_speed - correcao_angular);
            double ef_power = ((joystickInput.getY()*0.8+joystickInput.getX())*robot_speed - correcao_angular);
            double df_power = ((joystickInput.getY()*0.8-joystickInput.getX())*robot_speed + correcao_angular);
            double dt_power = ((joystickInput.getY()*0.8+joystickInput.getX())*robot_speed + correcao_angular);

            drive.setMotorPowers(et_power, ef_power, df_power, dt_power);



            erro_angular = (angulo_desejado - angulo_atual);
            if (erro_angular>180) erro_angular = erro_angular - 360;
            else if (erro_angular<-180) erro_angular = erro_angular + 360;
            erro_angular = erro_angular/45;





            drive.update();

            Slide_E.setPower(-gamepad1.right_stick_x);
            Slide_D.setPower(gamepad1.right_stick_x);


            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("pulso", Pulso.getPower());
            telemetry.addData("joy", gamepad1.right_stick_x);
            telemetry.addData("erro", erro_angular);
            telemetry.update();
        }
    }
}
