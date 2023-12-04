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

@TeleOp(name = "JareTeleOp (Java)")

//------------------------------------------------------------------------------

//Início do código principal

public class Jare extends LinearOpMode {
  
  //Iniciação dos motores
  
  private ElapsedTime timer;
  
  private DcMotor DF; // Motor Direita Frente
  private DcMotor DT; // Motor Direita Trás
  private DcMotor EF; // Motor Esquerda Frente
  private DcMotor ET; // Motor Esquerda Frente
  
  //----------------------------------------------------------------------------
  
  //Variáveis
  
  float x; //Recebe o valor lido no eixo X do controle analógico
  float y; //Recebe o valor lido no eixo Y do controle analógico
  
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
        
        ((DcMotorEx) DF).setPower( -y + x);
        ((DcMotorEx) EF).setPower( -y - x);
        ((DcMotorEx) DT).setPower( y - x);
        ((DcMotorEx) ET).setPower( y + x);
        
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
    
    DF.setDirection(DcMotorSimple.Direction.FORWARD);
    DT.setDirection(DcMotorSimple.Direction.FORWARD);
    EF.setDirection(DcMotorSimple.Direction.FORWARD);
    ET.setDirection(DcMotorSimple.Direction.REVERSE);
    
    DF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    EF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    DT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ET.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
  }
  //----------------------------------------------------------------------------
}
//Fim do código principal
