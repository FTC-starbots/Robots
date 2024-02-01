package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        LED led_1 = hardwareMap.get(LED.class, "led_1");
        LED led_2 = hardwareMap.get(LED.class, "led_2");
        LED led_3 = hardwareMap.get(LED.class, "led_3");
        LED led_4 = hardwareMap.get(LED.class, "led_4");
        LED led_5 = hardwareMap.get(LED.class, "led_5");
        LED led_6 = hardwareMap.get(LED.class, "led_6");
        LED led_7 = hardwareMap.get(LED.class, "led_7");
        LED led_8 = hardwareMap.get(LED.class, "led_8");

        int LED_Dir = 0;
        int LED_Esq = 0;






        DistanceSensor SensorDir;
        SensorDir = hardwareMap.get(DistanceSensor.class, "SensorDir");

        DistanceSensor SensorEsq;
        SensorEsq = hardwareMap.get(DistanceSensor.class, "SensorEsq");

        Servo GarraDir;
        GarraDir = hardwareMap.get(Servo.class, "GarraDir");

        Servo GarraEsq;
        GarraEsq = hardwareMap.get(Servo.class, "GarraEsq");

        Servo Escotilha;
        Escotilha = hardwareMap.get(Servo.class, "Escotilha");

        Servo Trilho;
        Trilho = hardwareMap.get(Servo.class, "Trilho");

        Servo Gatilho;
        Gatilho = hardwareMap.get(Servo.class, "Gatilho");

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

        double robot_speed = 0.9;
        double angulo_desejado = 0;
        double erro_angular = 0;

        int player = 1;
        double coordX = 0;
        double coordY = 0;

        int piddelay = 0;
        int sensordelay = 0;
        int atiradordelay = 0;
        boolean atirador = false;
        double SD = 500;
        double SE = 500;

        PIDCoefficients pidconfig_1 = new PIDCoefficients(1, 0.1 , 0.1);

        PIDFController angular_controler = new PIDFController(pidconfig_1);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        Escotilha.setPosition(1);
        Trilho.setPosition(0.1);

        GarraDir.setPosition(0.9);
        GarraEsq.setPosition(0.1);

        while (!isStopRequested()) {

            Pose2d poseEstimate = drive.getPoseEstimate();


            if (gamepad1.y) player = 1;
            if (gamepad2.y) player = 2;


            if (player == 1) {
                coordX = gamepad1.left_stick_x;
                coordY = -gamepad1.left_stick_y;
            }
            else{
                coordX = gamepad2.left_stick_x;
                coordY = -gamepad2.left_stick_y;
            }

            Vector2d joystickInput = new Vector2d (coordX, coordY)
                    .rotated(-poseEstimate.getHeading());
            Vector2d joystickraw = new Vector2d (coordX, coordY);



            double angulo_atual = (poseEstimate.getHeading()*180/Math.PI);

            angular_controler.setTargetPosition(0.0);
            double correcao_angular = -angular_controler.update(erro_angular);

            if (gamepad1.left_trigger>0.1 || gamepad1.right_trigger>0.1){
                correcao_angular = ((gamepad1.left_trigger-0.1)-(gamepad1.right_trigger-0.1));
                piddelay = 0;
            }
            else if ((gamepad2.left_trigger>0.1 || gamepad2.right_trigger>0.1) && player==2){
                correcao_angular = ((gamepad2.left_trigger-0.1)-(gamepad2.right_trigger-0.1));
                piddelay = 0;
            }
            else if (piddelay < 20) correcao_angular = 0;



            if (piddelay < 20){
                angulo_desejado = angulo_atual;
                piddelay++;
            }
            if(gamepad1.right_bumper) angulo_desejado = -90+(joystickraw.angle()*180/Math.PI);

            double potencia_maxima_ang = 0.7;

            if(gamepad1.dpad_up) angulo_desejado = 0;
            else if(gamepad1.dpad_down) angulo_desejado = 180;
            else if(gamepad1.dpad_right || gamepad2.dpad_right) angulo_desejado = 270;
            else if(gamepad1.dpad_left || gamepad2.dpad_left) angulo_desejado = 90;



            if(correcao_angular>potencia_maxima_ang) correcao_angular=potencia_maxima_ang;
            else if(correcao_angular<-potencia_maxima_ang) correcao_angular=-potencia_maxima_ang;


            double et_power = ((joystickInput.getY()*0.9-joystickInput.getX())*robot_speed - correcao_angular);
            double ef_power = ((joystickInput.getY()*0.9+joystickInput.getX())*robot_speed - correcao_angular);
            double df_power = ((joystickInput.getY()*0.9-joystickInput.getX())*robot_speed + correcao_angular);
            double dt_power = ((joystickInput.getY()*0.9+joystickInput.getX())*robot_speed + correcao_angular);

            drive.setMotorPowers(et_power, ef_power, df_power, dt_power);


            erro_angular = (angulo_desejado - angulo_atual);
            if (erro_angular>180) erro_angular = erro_angular - 360;
            else if (erro_angular<-180) erro_angular = erro_angular + 360;
            erro_angular = erro_angular/45;


            drive.update();

            Slide_E.setPower(-gamepad1.right_stick_y);
            Slide_D.setPower(gamepad1.right_stick_y);

            if (gamepad2.x && sensordelay>2){
                SD = SensorDir.getDistance(DistanceUnit.MM);
                SE = SensorEsq.getDistance(DistanceUnit.MM);
                sensordelay = 0;
                robot_speed = 0.6;
            }
            else{
                if(player == 1) robot_speed = 0.9;
                else robot_speed = 0.6;
            }
            sensordelay++;


            if (gamepad2.right_bumper) {
                GarraDir.setPosition(0.9);
                LED_Dir = 1;
            }
            else if ((SD < 60 && gamepad2.x) || (gamepad1.a)) {
                GarraDir.setPosition(0.1);
                LED_Dir = 0;
            }

            if (gamepad2.left_bumper) {
                GarraEsq.setPosition(0.1);
                LED_Esq = 1;
            }
            else if ((SE < 60 && gamepad2.x) || (gamepad1.a)) {
                GarraEsq.setPosition(0.8);
                LED_Esq = 0;
            }


                Pulso.setPower(-gamepad2.right_stick_y);
            

            if (atirador) atiradordelay++;
            else atiradordelay = 0;

            if(gamepad1.back){ atirador = true; Escotilha.setPosition(0);}

            if(atiradordelay>20) Trilho.setPosition(0.5);
            if(atiradordelay>30) Gatilho.setPosition(1);
            if(atiradordelay>50) {Trilho.setPosition(0.1); Escotilha.setPosition(1); atirador = false;}









            if (LED_Dir == 1) { led_1.enable(true); led_2.enable(false); led_7.enable(true); led_8.enable(false);}
            if (LED_Dir == 0) { led_1.enable(false); led_2.enable(true); led_7.enable(false); led_8.enable(true);}
            if (LED_Esq == 1) { led_3.enable(true); led_4.enable(false); led_5.enable(true); led_6.enable(false);}
            if (LED_Esq == 0) { led_3.enable(false); led_4.enable(true); led_5.enable(false); led_6.enable(true);}

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("pulso", Pulso.getPower());
            telemetry.addData("joy", gamepad1.right_stick_x);
            telemetry.addData("erro", erro_angular);
            telemetry.addData("Sensor DIreita", SD);
            telemetry.addData("Sensor Esquerda", SE);
            telemetry.update();




        }
    }


}
