//Início das importações

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import java.util.List;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

//Fim das importações

//Nomeação do arquivo

@TeleOp(name = "Jare3TeleOp (Java)")

//------------------------------------------------------------------------------

//Início do código principal

public class Jare3 extends LinearOpMode {
  
  //Iniciação dos motores
  
  private ElapsedTime timer;
  
  private DcMotor DF; // Motor Direita Frente
  private DcMotor DT; // Motor Direita Trás
  private DcMotor EF; // Motor Esquerda Frente
  private DcMotor ET; // Motor Esquerda Frente
  private IMU imu; // Hardware da IMU
  private DcMotor Slide_E;
  private DcMotor Slide_D;
  //----------------------------------------------------------------------------
  
  //Variáveis
  
  int velocidade_robo;//Contém o nível de velocidadde do robô em potência (100% = 1000)
  double angulo_desejado; //Indica o ângulo que o robô deve alcançar (pelo Trigger ou pelo Dpad)
  double erro_angular; //Variável que define o ângulo que o robô deve alcançar
  double angulo_atual;
  double potencia_correcao;
  double xf; //Cálculo de correção das coordenadas do eixo X do controle
  double yf; //Cálculo de correção das coordenadas do eixo Y do controle
  double xcam;
  double ycam;
  double zcam;
  double bcam;
  double gatilho; //Calcula qual gatilho está acionado
  double x = 0; //Recebe o valor lido no eixo X do controle analógico
  double y = 0; //Recebe o valor lido no eixo Y do controle analógico
  boolean up_dpad;
  boolean right_dpad;
  boolean down_dpad;
  boolean left_dpad;
  boolean modo_preciso;
  boolean modo_turbo;
  boolean modo_align;
  boolean USE_WEBCAM;
  
  List<AprilTagDetection> myAprilTagDetections;
  
  YawPitchRollAngles orientation;
  AngularVelocity angularVelocity;
  AprilTagProcessor myAprilTagProcessor;
  AprilTagDetection myAprilTagDetection;
  AprilTagMetadata myAprilTagMetadata;
  AprilTagLibrary.Builder myAprilTagLibraryBuilder;
  AprilTagProcessor.Builder myAprilTagProcessorBuilder;
  AprilTagLibrary myAprilTagLibrary;
  VisionPortal.Builder myVisionPortalBuilder;
  VisionPortal myVisionPortal;
  
  //----------------------------------------------------------------------------
  @Override
  //Função principal
  
  public void runOpMode() {
    
    acessando_hardware();
    configurando_hardware();
    
    imu.resetYaw();
    
    timer = new ElapsedTime();
    
    modo_align = false;
    
    USE_WEBCAM = true;
    
    initAprilTag();
    
    telemetry.addData("RC preview on/off", "3 dots => Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.update();
    
    waitForStart();//Início do programa (Play no app da FTC)
    
    //Chamada de funções
    
    //--------------------------------------------------------------------------
    
    //Loop do programa
    
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        
        telemetry.addData("Robô rodando...", "😁👍");
        telemetryAprilTag();
        
        orientation = imu.getRobotYawPitchRollAngles(); //Associa os eixos rotacionais com o IMU
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES); //Associa a velocidade de rotação com o IMU
        
        angulo_atual = orientation.getYaw(AngleUnit.DEGREES);
        
        gatilho = (gamepad1.right_trigger-gamepad1.left_trigger);
        
        if (JavaUtil.listLength(myAprilTagDetections) > 0){
        xcam = myAprilTagDetection.ftcPose.x*2.54;
        ycam = myAprilTagDetection.ftcPose.y*2.54;
        zcam = myAprilTagDetection.ftcPose.z*2.54;
        bcam = myAprilTagDetection.ftcPose.bearing;
        }
        else{
          modo_align = false;
        }
        
        
        if (modo_align){
          x = -(xcam / 8);
          y = ((ycam - 20) / 58.5);
          
          angulo_desejado = angulo_atual+bcam;
          velocidade_robo = 250; //A velocidadde do robô vai para 150%
          gatilho *= 250; //------------------------------------<[⇧]
          
          if (gamepad1.y){
            modo_align = false;
          }
        }
        else if (!modo_align){
          x = gamepad1.left_stick_x;
          y = -gamepad1.left_stick_y;
          
          if(gatilho>0.05f || gatilho<-0.05f){
            angulo_desejado = angulo_atual;
            erro_angular = 0;
            timer.reset();
          }
          else gatilho = 0;
          
          if (gamepad1.a) { //Se o botão "A" é pressionado, ativa a Velocidade Precisa
            velocidade_robo = 500; //A velocidadde do robô vai para 50%
            gatilho *= 500; //------------------------------------<[⇧]
          }
          else if (gamepad1.b) { //Se o botão "B" é pressionado, ativa a Velocidadde Turbo
            velocidade_robo = 3000; //A velocidadde do robô vai para 300%
            gatilho *= 3000; //------------------------------------<[⇧]
          }
          else { //Se nenhum botão é pressionado, ativa a Velocidadde Padrão
            velocidade_robo = 1500; //A velocidadde do robô vai para 150%
            gatilho *= 1500; //------------------------------------<[⇧]
          }
          
          if (gamepad1.y && JavaUtil.listLength(myAprilTagDetections) > 0){
            modo_align = true;
          }
        }
        
        potencia_correcao = 0;
        
        //Recebe o ângulo em que o robô se encontra
        
        double erro_horario =  angulo_atual - angulo_desejado;
        double erro_antihorario = angulo_atual + (360-angulo_desejado);
        
        if (timer.seconds() < 1.5){
          erro_horario =  0;
          erro_antihorario = 0;
          
        }
        
        if (Math.abs(erro_horario)>180){
          erro_angular = erro_antihorario;
        } 
        else{
          erro_angular = erro_horario;
           /*Calcula a diferença entre o ângulo desejado
            e o ângulo atual, para saber quanto o robô 
            deve se mover para achar o ângulo desejado*/
        }
        
        //erro_I = erro_I + (erro_angular/100);
        
        potencia_correcao = (erro_angular*25);
        
        //Leitura gamepad 1
  
        //Leitura controle direcional
          
        if (gamepad1.dpad_up){ //Se o direcional pra cima é pressionado
          angulo_desejado = 0; //O robô aponta para a referência zero
        }
        else if (gamepad1.dpad_right){ //Se o direcional pra direita é pressionado
          angulo_desejado = -90; //O robô aponta para 90° da referência zero
        }
        else if (gamepad1.dpad_down){ //Se o direcional pra baixo é pressionado
          angulo_desejado = 170; //O robô aponta para 180° da referência zero
        }
        else if (gamepad1.dpad_left){ //Se o direcional pra esquerda é pressionado
          angulo_desejado = 90; //O robô aponta para -90° da referência zero
        }
        else if (timer.seconds() < 1) angulo_desejado = angulo_atual;
        
        
        if (gamepad1.start) {
          imu.resetYaw(); //Se o botão Start é pressionado, a referência do IMU é reiniciada para a atual
        }
        
        //--------------------------------------------------------------------
        
        xf = Math.cos((angulo_atual) / 180 * 3.14159265f) * x + Math.sin((angulo_atual) / 180 * 3.14159265f) * y;
        yf = Math.sin((angulo_atual) / 180 * 3.14159265f) * x - Math.cos((angulo_atual) / 180 * 3.14159265f) * y;
        
        //----------------------------------------------------------------------
        
        //Ativação dos motores base
        
        ((DcMotorEx) DF).setVelocity( (yf*velocidade_robo)+(xf*velocidade_robo) + gatilho + potencia_correcao);
        ((DcMotorEx) EF).setVelocity( (yf*velocidade_robo)-(xf*velocidade_robo) - gatilho - potencia_correcao);
        ((DcMotorEx) DT).setVelocity( -(yf*velocidade_robo)-(xf*velocidade_robo) + gatilho + potencia_correcao);
        ((DcMotorEx) ET).setVelocity( -(yf*velocidade_robo)+(xf*velocidade_robo) - gatilho - potencia_correcao);
        
        //----------------------------------------------------------------------
        
        //Ativação slide
        
        Slide_D.setPower(gamepad1.right_stick_y);
        Slide_E.setPower(gamepad1.right_stick_y);
        
        
        
        
        
        //Telemetria
        
        telemetry.addData("time: ", timer.seconds());
        telemetry.addData("Angulo desejado: ", angulo_desejado);
        telemetry.addData("Angulo atual: ", angulo_atual);
        telemetry.addData("Modo Align: ", modo_align);
        telemetry.addData("DF: ", ((DcMotorEx) DF).getVelocity());
        telemetry.addData("DT: ", ((DcMotorEx) DT).getVelocity());
        telemetry.addData("EF: ", ((DcMotorEx) EF).getVelocity());
        telemetry.addData("ET: ", ((DcMotorEx) ET).getVelocity());
        telemetry.addData("Slide: ", ((DcMotorEx) Slide_D).getCurrentPosition());
       
        telemetry.update();
        
        //----------------------------------------------------------------------
      }
    }
    //--------------------------------------------------------------------------
  }
  //----------------------------------------------------------------------------
  //Configura o modo e a direção inicial de funcionamento dos motores
  
  
  
  
  
  
  public void acessando_hardware(){
    
    angulo_desejado = 0;
    
    ET = hardwareMap.get(DcMotor.class, "ET");
    DF = hardwareMap.get(DcMotor.class, "DF");
    EF = hardwareMap.get(DcMotor.class, "EF");
    DT = hardwareMap.get(DcMotor.class, "DT");
    
    imu = hardwareMap.get(IMU.class, "IMU");
    
    Slide_E = hardwareMap.get(DcMotor.class, "Slide_E");
    Slide_D = hardwareMap.get(DcMotor.class, "Slide_D");

    
  } 
 
 
 
 
  public void configurando_hardware() {
    
    DF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    EF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    DT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
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
    
    Slide_E.setDirection(DcMotorSimple.Direction.REVERSE);
    Slide_D.setDirection(DcMotorSimple.Direction.REVERSE);
    
    Slide_E.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Slide_D.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    Slide_E.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Slide_D.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    
  }
  
  private void initAprilTag() {

    myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
    
    myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
    
    myAprilTagMetadata = new AprilTagMetadata(55, "Our Awesome Team Tag", 3.5, DistanceUnit.INCH);
    
    myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);
    
    myAprilTagLibrary = myAprilTagLibraryBuilder.build();
    
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    
    myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
    
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    
    myVisionPortalBuilder = new VisionPortal.Builder();
    
    if (USE_WEBCAM) {
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    myVisionPortal = myVisionPortalBuilder.build();
  }

  private void telemetryAprilTag() {
    
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    if (JavaUtil.listLength(myAprilTagDetections) > 0){
      for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
        myAprilTagDetection = myAprilTagDetection_item;
        telemetry.addLine("");
        if (myAprilTagDetection.metadata != null) {
          telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
          telemetry.addLine("XYZ: " + JavaUtil.formatNumber(xcam, 6, 1) + ", " + JavaUtil.formatNumber(ycam, 6, 1) + ", " + JavaUtil.formatNumber(zcam, 6, 1) + "  (cm)");
          telemetry.addLine("Bearing: " + JavaUtil.formatNumber(bcam, 6, 1));
        } else {
          telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
          telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
        }
      }
    }
    else{
      telemetry.addLine("");
      telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    }
  }
}
//Fim do código principal
