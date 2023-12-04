package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
/*
* Primeiro teste do autônomo "azul-superior"*/
@Config
@Autonomous(group = "drive")
public class TestAutonomous extends LinearOpMode {
    //Instâncias de edição dinâmica do Dashboard
    /*
    * Essas instâncias ajudam no momento de calibração dos passos do autônomo,
    * onde é preciso alterar os valores constantemente para alcançar a posição
    * ideal do robô. Com isso, não é preciso esperar o código buildar novamente
    * para alterar os valores. É possível fazer isso simultaneamente à reprodução
    * real do código.
    * ATENÇÃO!!!
    * Os códigos alterados nas instâncias dinâmicas não são permanentes.
    * Assim que forem definidos os melhores valores, é necessário editá-los no
    * código original*/
    public static int position = 2;
    public static double deg1;
    public static double deg2;
    public static double deg3;
    public static double dist1;
    public static double dist2;
    public static double dist3;
    public static double dist4;
    public static double dist5;
    public static double dist6;
    public static double dist7;
    public static double dist8;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        //↓ Define o hardware virtual do RoadRunner acessando o arquivo do "SampleMecanumDrive"
        drive = new SampleMecanumDrive(hardwareMap);

        //↓ Espera até que o botão Start seja pressionado no Drive Station
        waitForStart();

        if (position == 1){
            runTrajectory1();
        }
        else if (position == 2){
            runTrajectory2();
        }
        else if (position == 3){
            runTrajectory3();
        }
    }
    public void runTrajectory1(){

        deg1 = 91;//instância de giro 1
        deg2 = 180;//instância de giro 2
        dist1 = 1.25;//instância de distância 1
        dist2 = 1.6;//instância de distância 2
        dist3 = 0.9;//instância de distância 3
        dist4 = 0.35;//instância de distância 4
        dist5 = 1.7;//instância de distância 5
        dist6 = 1.3;//instância de distância 6

        Trajectory step1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(dist1)
                .build();
        Trajectory step2 = drive.trajectoryBuilder(step1.end())
                .forward(dist2)
                .build();
        Trajectory step3 = drive.trajectoryBuilder(step2.end())
                .back(dist3)
                .build();
        Trajectory step4 = drive.trajectoryBuilder(step3.end())
                .forward(dist4)
                .build();
        Trajectory step5 = drive.trajectoryBuilder(step4.end())
                .back(dist4)
                .build();
        Trajectory step6 = drive.trajectoryBuilder(step5.end())
                .strafeRight(dist5)
                .build();
        Trajectory step7 = drive.trajectoryBuilder(step6.end())
                .strafeRight(dist6)
                .build();

        if(isStopRequested()) return;//Método de segurança padrão do FirstRobotics

        drive.followTrajectory(step1);
        drive.followTrajectory(step2);
        drive.turn(Math.toRadians(deg1));
        drive.followTrajectory(step3);
        drive.turn(Math.toRadians(deg2));
        drive.followTrajectory(step4);
        drive.followTrajectory(step5);
        drive.followTrajectory(step6);
        drive.turn(Math.toRadians(deg1));
        drive.followTrajectory(step7);
    }
    public void runTrajectory2(){
        deg1 = -98;//instância de giro 1
        deg2 = 95;//instância de giro 2
        dist1 = 1.78;//instância de distância 1
        dist2 = 0.35;//instância de distância 2
        dist3 = 1.65;//instância de distância 3
        dist4 = 0.3;//instância de distância 4
        dist5 = 0.3;//instância de distância 5
        dist6 = 1.7;//instância de distância 6
        dist7 = 0.5;//instância de distância 7
        dist8 = 1.82;//instância de distância 8

        Trajectory step1 = drive.trajectoryBuilder(new Pose2d())
                .forward(dist1)//Avança 1.78 unidades
                .build();
        Trajectory step2 = drive.trajectoryBuilder(step1.end())
                .back(dist2)//Volta 0.35 unidades
                .build();
        Trajectory step3 = drive.trajectoryBuilder(step2.end())
                .forward(dist3)//Avança 1.65 unidades
                .build();
        Trajectory step4 = drive.trajectoryBuilder(step3.end())
                .forward(dist4)//Avança 0.3 unidades
                .build();
        Trajectory step5 = drive.trajectoryBuilder(step4.end())
                .back(dist5)//Volta 0.3 unidades
                .build();
        Trajectory step6 = drive.trajectoryBuilder(step5.end())
                .strafeRight(dist6)//Deslisa para a esquerda (referência do robô) 1.7 unidades
                .build();
        Trajectory step7 = drive.trajectoryBuilder(step6.end())
                .strafeRight(dist7)//Deslisa para a esquerda (referência do robô) 0.5 unidades
                .build();/*Em todas as trajetórias, essa função compila todos os métodos listados,
                           permitindo sua execução quando a função for chamada*/

        if(isStopRequested()) return;//Método de segurança padrão do FirstRobotics

        drive.followTrajectory(step1);
        //Coloca o robô na zona das Spike Marks para liberação
        //de um dos pixels preso inicialmente na garra
        drive.followTrajectory(step2);
        //Dá uma pequena ré no robô para não interferir na posição
        //do pixel com o próximo passo
        drive.turn(Math.toRadians(deg1));
        //Gira o robô em direção à Backdrop
        drive.followTrajectory(step3);
        //Avança o robô até a Backdrop
        drive.followTrajectory(step4);
        //Última aproximação para descarte preciso do pixel
        drive.followTrajectory(step5);
        //Afasta da Backdrop para o estacionamento
        drive.followTrajectory(step6);
        //Desloca-se até a área de estacionamento maior
        drive.turn(Math.toRadians(deg2));
        //Gira para a posição de referência inicial do TeleOp
        drive.followTrajectory(step7);
        //Entra na zona de estacionamento
    }
    public void runTrajectory3(){
        deg1 = -98;//instância de giro 1
        deg2 = 95;//instância de giro 2
        dist1 = 1.78;//instância de distância 1
        dist2 = 0.35;//instância de distância 2
        dist3 = 1.65;//instância de distância 3
        dist4 = 0.3;//instância de distância 4
        dist5 = 0.3;//instância de distância 5
        dist6 = 1.7;//instância de distância 6
        dist7 = 0.5;//instância de distância 7
        dist8 = 1.82;//instância de distância 8

        Trajectory step1 = drive.trajectoryBuilder(new Pose2d())
                .forward(dist1)//Avança 1.78 unidades
                .build();
        Trajectory step2 = drive.trajectoryBuilder(step1.end())
                .back(dist2)//Volta 0.35 unidades
                .build();
        Trajectory step3 = drive.trajectoryBuilder(step2.end())
                .forward(dist3)//Avança 1.65 unidades
                .build();
        Trajectory step4 = drive.trajectoryBuilder(step3.end())
                .forward(dist4)//Avança 0.3 unidades
                .build();
        Trajectory step5 = drive.trajectoryBuilder(step4.end())
                .back(dist5)//Volta 0.3 unidades
                .build();
        Trajectory step6 = drive.trajectoryBuilder(step5.end())
                .strafeRight(dist6)//Deslisa para a esquerda (referência do robô) 1.7 unidades
                .build();
        Trajectory step7 = drive.trajectoryBuilder(step6.end())
                .strafeRight(dist7)//Deslisa para a esquerda (referência do robô) 0.5 unidades
                .build();/*Em todas as trajetórias, essa função compila todos os métodos listados,
                           permitindo sua execução quando a função for chamada*/

        if(isStopRequested()) return;//Método de segurança padrão do FirstRobotics

        drive.followTrajectory(step1);
        //Coloca o robô na zona das Spike Marks para liberação
        //de um dos pixels preso inicialmente na garra
        drive.followTrajectory(step2);
        //Dá uma pequena ré no robô para não interferir na posição
        //do pixel com o próximo passo
        drive.turn(Math.toRadians(deg1));
        //Gira o robô em direção à Backdrop
        drive.followTrajectory(step3);
        //Avança o robô até a Backdrop
        drive.followTrajectory(step4);
        //Última aproximação para descarte preciso do pixel
        drive.followTrajectory(step5);
        //Afasta da Backdrop para o estacionamento
        drive.followTrajectory(step6);
        //Desloca-se até a área de estacionamento maior
        drive.turn(Math.toRadians(deg2));
        //Gira para a posição de referência inicial do TeleOp
        drive.followTrajectory(step7);
        //Entra na zona de estacionamento
    }
}