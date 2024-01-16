//Início das importações

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//Fim das importações

//Nomeação do arquivo

@TeleOp(name = "Teste (Java)")

//------------------------------------------------------------------------------

//Início do código principal

public class BeatleTeleOp extends LinearOpMode {

    //Iniciação dos motores

    private ElapsedTime timer;

    private DcMotor DF; // Motor Direita Frente
    private DcMotor DT; // Motor Direita Trás
    private DcMotor EF; // Motor Esquerda Frente
    private DcMotor ET; // Motor Esquerda Frente
    private Servo garra_D;
    private Servo garra_E;

    //----------------------------------------------------------------------------

    //Variáveis

    float x; //Recebe o valor lido no eixo X do controle analógico
    float y; //Recebe o valor lido no eixo Y do controle analógico
    float gatilho;
    double angulo_desejado;
    double angulo_atual;
    double erro_angular;
    //----------------------------------------------------------------------------
    @Override
    //Função principal

    public void runOpMode() {

        acessando_hardware();
        configurando_hardware();

        timer = new ElapsedTime();

        waitForStart();//Início do programa (Play no app da FTC)

        //Chamada de funções

        //--------------------------------------------------------------------------

        //Loop do programa

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                x = gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;

                gatilho = (gamepad1.right_trigger-gamepad1.left_trigger);

                if (gatilho != 0){
                    ((DcMotorEx) DF).setVelocity(-gatilho * 30000);
                    ((DcMotorEx) EF).setVelocity(gatilho * 30000);
                    ((DcMotorEx) DT).setVelocity(gatilho * 3000);
                    ((DcMotorEx) ET).setVelocity(-gatilho * 3000);
                }
                else {
                    ((DcMotorEx) DF).setVelocity(( (-y + x) * 10) * 3000);
                    ((DcMotorEx) EF).setVelocity(( (-y - x) * 10) * 3000);
                    ((DcMotorEx) DT).setVelocity(( y + x) * 3000);
                    ((DcMotorEx) ET).setVelocity(( y - x) * 3000);
                }

                if (gamepad1.a){
                    garra_D.setPosition(1);
                    garra_E.setPosition(1);
                }
                else {
                    garra_D.setPosition(0);
                    garra_E.setPosition(0);
                }

                if (!gamepad1.a){
                    if (gamepad1.b){
                        garra_D.setPosition(0.75);
                        garra_E.setPosition(0.75);
                    }
                    else {
                        garra_D.setPosition(0);
                        garra_E.setPosition(0);
                    }
                }


                telemetry.addData("DF", ((DcMotorEx) DF).getVelocity());
                telemetry.addData("DT", ((DcMotorEx) DT).getVelocity());
                telemetry.addData("EF", ((DcMotorEx) EF).getVelocity());
                telemetry.addData("ET", ((DcMotorEx) ET).getVelocity());

                telemetry.update();

                //----------------------------------------------------------------------
            }

        }
        //--------------------------------------------------------------------------
    }
    //----------------------------------------------------------------------------
    //Configura o modo e a direção inicial de funcionamento dos motores
    public void acessando_hardware(){

        ET = hardwareMap.get(DcMotor.class, "ET");
        DF = hardwareMap.get(DcMotor.class, "DF");
        EF = hardwareMap.get(DcMotor.class, "EF");
        DT = hardwareMap.get(DcMotor.class, "DT");
        garra_D = hardwareMap.get(Servo.class, "garra_D");
        garra_E = hardwareMap.get(Servo.class, "garra_E");

    }
    public void configurando_hardware() {

        DF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slide_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        EF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ET.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DF.setDirection(DcMotorSimple.Direction.REVERSE);
        DT.setDirection(DcMotorSimple.Direction.FORWARD);
        EF.setDirection(DcMotorSimple.Direction.FORWARD);
        ET.setDirection(DcMotorSimple.Direction.REVERSE);

        DF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ET.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        garra_E.setDirection(Servo.Direction.REVERSE);

    }
    //----------------------------------------------------------------------------
}
//Fim do código principal
