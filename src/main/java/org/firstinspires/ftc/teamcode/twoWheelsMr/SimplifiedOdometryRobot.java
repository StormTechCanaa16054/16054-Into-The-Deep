/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.twoWheelsMr;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class SimplifiedOdometryRobot {
    // Ajuste de acordo com o robô da equipe
    private final double ODOM_INCHES_PER_COUNT   = 0.002929;   //  GoBilda Odometry Pod (1/226.8)
    private final boolean INVERT_DRIVE_ODOMETRY  = false;       // Ao dirigir PARA FRENTE, o valor da odometria DEVE aumentar. Caso contrário, inverta o valor desta constante.
    private final boolean INVERT_STRAFE_ODOMETRY = true;       //  Ao strafar para a ESQUERDA, o valor da odometria DEVE aumentar. Caso contrário, inverta o valor desta constante.

    private static final double DRIVE_GAIN          = 0.03;    // Força do controle de posição axial
    private static final double DRIVE_ACCEL         = 2;     // Limite de aceleração. Alteração percentual de potência por segundo. 1,0 = 0-100% de potência em 1 segundo.
    private static final double DRIVE_TOLERANCE     = 1.5;     // O controlador está "inPosition" se o erro de posição for < +/- este valor
    private static final double DRIVE_DEADBAND      = 0.6;     // Erro menor que isso causa saída zero. Deve ser menor que DRIVE_TOLERANCE
    private static final double DRIVE_MAX_AUTO      = 1;     // "padrão" Limite máximo de potência axial durante modo autônomo

    private static final double STRAFE_GAIN         = 0.03;    //Força do controle de posição lateral
    private static final double STRAFE_ACCEL        = 1.5;     // Limite de aceleração. Alteração percentual de potência por segundo. 1,0 = 0-100% de potência em 1 segundo.
    private static final double STRAFE_TOLERANCE    = 1.5;     // O controlador está "inPosition" se o erro de posição for < +/- este valor
    private static final double STRAFE_DEADBAND     = 0.6;     // Erro menor que isso causa saída zero. Deve ser menor que DRIVE_TOLERANCE
    private static final double STRAFE_MAX_AUTO     = 1;     // Limite máximo de potência lateral "padrão" durante o modo autônomo

    private static final double YAW_GAIN            = 0.018;    // Controle de posição de força de guinada
    private static final double YAW_ACCEL           = 3.0;     // Limite de aceleração. Alteração percentual de potência por segundo. 1,0 = 0-100% de potência em 1 segundo.
    private static final double YAW_TOLERANCE       = 1;     // O controlador está "inPosition" se o erro de posição for < +/- este valor
    private static final double YAW_DEADBAND        = 0.5;    // Erro menor que isso causa saída zero. Deve ser menor que DRIVE_TOLERANCE
    private static final double YAW_MAX_AUTO        = 0.6;     // Limite máximo de potência de guinada "padrão" durante o modo autônomo

    // Public Members
    public double driveDistance     = 0; //distância axial escalonada (+ = para frente)
    public double strafeDistance    = 0; // distância lateral escalonada (+ = esquerda)
    public double heading           = 0; // Último posição do robô (IMU)

    // Estabeleça um controlador proporcional para cada eixo para calcular a potência necessária para atingir um ponto de ajuste.
    public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl strafeController    = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // ---  Private Members

    // Objetos de interface de hardware
    private DcMotor leftFrontDrive;     //  controlar a roda motriz dianteira esquerda
    private DcMotor rightFrontDrive;    //  controlar a roda motriz dianteira direita
    private DcMotor leftBackDrive;      //  controlar a roda motriz traseira esquerda
    private DcMotor rightBackDrive;     //  controlar a roda motriz traseira direita

    private DcMotor driveEncoder;       // o pod de Odometria Axial (dianteiro/traseiro) (pode se sobrepor ao motor ou não)
    private DcMotor strafeEncoder;      // o pod de Odometria Lateral (esquerda/direita) (pode se sobrepor ao motor ou não)
    ElapsedTime clock = new ElapsedTime();

    private LinearOpMode myOpMode;
    private IMU imu;
    private ElapsedTime holdTimer = new ElapsedTime();  // Usuário para qualquer movimento que exija um tempo de espera ou limite de tempo.

    private int rawDriveOdometer    = 0; //Contagem do hodômetro axial não modificada
    private int driveOdometerOffset = 0; // Usado para compensar o odômetro axial
    private int rawStrafeOdometer   = 0; // Contagem do odômetro lateral não modificada
    private int strafeOdometerOffset= 0; // Usado para compensar o odômetro lateral
    private double rawHeading       = 0; // Título não modificado (graus)
    private double headingOffset    = 0; // Usado para compensar o rumo

    private double turnRate           = 0; // Última taxa de rotação do robô da IMU
    private boolean showTelemetry     = false;

    // Robot Constructor
    public SimplifiedOdometryRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry)
    {
        // Inicialize as variáveis de hardware. Observe que as strings usadas para 'get' cada
        // motor/dispositivo deve corresponder aos nomes atribuídos durante a configuração do robô.

        // !!! Define a direção de tração para garantir que a potência positiva impulsione cada roda para frente.
        leftFrontDrive  = setupDriveMotor("MotorFE", DcMotor.Direction.FORWARD);
        rightFrontDrive = setupDriveMotor("MotorFD", DcMotor.Direction.FORWARD);
        leftBackDrive  = setupDriveMotor( "MotorTE", DcMotor.Direction.FORWARD);
        rightBackDrive = setupDriveMotor( "MotorTD",DcMotor.Direction.REVERSE);
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        //  Conecte-se aos canais do codificador usando o nome desse canal.
        driveEncoder = myOpMode.hardwareMap.get(DcMotor.class, "axial");
        strafeEncoder = myOpMode.hardwareMap.get(DcMotor.class, "lateral");

        // Configure todos os hubs para usar o modo AUTO Bulk Caching para leituras mais rápidas do codificador
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Informe ao software como o Control Hub é montado no robô para alinhar os eixos IMU XYZ corretamente
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // zere todas as leituras de odometria.
        resetOdometry();

        // Defina o estado de telemetria desejado
        this.showTelemetry = showTelemetry;
    }

    /**
     *   Configure um motor de acionamento com parâmetros passados. Certifique-se de que o codificador esteja redefinido.
     * @param deviceName  Nome de texto associado ao motor na Configuração do Robô
     * @param direction   Direção desejada para fazer a roda girar PARA FRENTE com entrada de potência positiva
     * @return o objeto DCMotor
     */
    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Redefinir codificadores para zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requer que os cabos do encoder do motor sejam conectados.
        return aMotor;
    }

    /**
     *Leia todos os dispositivos de entrada para determinar o movimento do robô
     * sempre retorne verdadeiro para que possa ser usado em condições de loop "while"
     * @return true
     */
    public boolean readSensors() {
        rawDriveOdometer = driveEncoder.getCurrentPosition() * (INVERT_DRIVE_ODOMETRY ? -1 : 1);
        rawStrafeOdometer = strafeEncoder.getCurrentPosition() * (INVERT_STRAFE_ODOMETRY ? -1 : 1);
        driveDistance = (rawDriveOdometer - driveOdometerOffset) * ODOM_INCHES_PER_COUNT;
        strafeDistance = (rawStrafeOdometer - strafeOdometerOffset) * ODOM_INCHES_PER_COUNT;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        heading     = rawHeading - headingOffset;
        turnRate    = angularVelocity.zRotationRate;

        if (showTelemetry) {
            myOpMode.telemetry.addData("Odom Ax:Lat", "%6d %6d", rawDriveOdometer - driveOdometerOffset, rawStrafeOdometer - strafeOdometerOffset);
            myOpMode.telemetry.addData("Dist Ax:Lat", "%5.2f %5.2f", driveDistance, strafeDistance);
            myOpMode.telemetry.addData("Head Deg:Rate", "%5.2f %5.2f", heading, turnRate);
        }
        return true;  //faça isso para que esta função possa ser incluída na condição por um loop while para manter os valores atualizados.
    }

    //  ########################  Funções de controle de nível médio. #############################3#

    /**
     * Dirija na direção axial (para frente/para trás), mantenha o rumo atual e não desvie para os lados
     * @param distanceCm  Distância para viajar. +ve = avançar, -ve = reverter.
     * @param power Potência máxima a aplicar. Este número deve ser sempre positivo.
     * @param holdTime Tempo mínimo (seg) necessário para manter a posição final. 0 = sem retenção.
     */
    public void drive(double distanceCm, double power, double holdTime, double timeOut) {
        resetOdometry();
        clock.reset();

        driveController.reset(distanceCm/2.54, power);   // alcançar a distância de condução desejada
        strafeController.reset(0);              // Manter desvio zero de metralhadora
        yawController.reset();                          // Manter a direção da última curva
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors() && clock.seconds() < timeOut){

            // implementar potências de eixo desejadas
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Hora de sair?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Sair do loop se estivermos em posição e já estivermos lá por tempo suficiente.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Strafar na direção lateral (esquerda/direita), manter o rumo atual e não desviar para frente/para trás
     * @param distanceCm  Distância para viajar. +ve = esquerda, -ve = direita.
     * @param power Potência máxima a aplicar. Este número deve ser sempre positivo.
     * @param holdTime Tempo mínimo (seg) necessário para manter a posição final. 0 = sem retenção.
     */
    public void strafe(double distanceCm, double power, double holdTime) {
        resetOdometry();

        driveController.reset(0.0);             //  Manter o desvio zero da unidade
        strafeController.reset(distanceCm/2.54, power);  // Alcance a distância Strafe desejada
        yawController.reset();                          // Manter o ângulo da última curva
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implementar potências de eixo desejadas
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Hora de sair?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Sair do loop se estivermos em posição e já estivermos lá por tempo suficiente.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Girar para um rumo/direção absoluto
     * @param headingDeg Indo para obter. +ve = sentido anti-horário, -ve = sentido horário.
     * @param power Potência máxima a aplicar. Este número deve ser sempre positivo.
     * @param holdTime Tempo mínimo (seg) necessário para manter a posição final. 0 = sem retenção.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {

        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive() && readSensors()) {

            // implementar potências de eixo desejadas
            moveRobot(0, 0, yawController.getOutput(heading));

            // Hora de sair?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Sair do loop se estivermos em posição e já estivermos lá por tempo suficiente.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    public void tudo(double driveCm, double strafeCm, double headingDeg, double power, double holdTime){
        resetOdometry();

        driveController.reset(driveCm/2.54, power);
        strafeController.reset(strafeCm/2.54, power);
        yawController.reset(headingDeg, 0.1);
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implementar potências de eixo desejadas
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition() && driveController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Sair do loop se estivermos em posição e já estivermos lá por tempo suficiente.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }


    //  ########################  Funções de controle de baixo nível. ###############################

    /**
     *Acione os motores das rodas para obter os movimentos dos eixos solicitados
     * @param drive     Potência do eixo Avançar/Rev
     * @param strafe    Potência do eixo esquerdo/direito
     * @param yaw       Potência do eixo de guinada
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        // normalizar os valores do motor
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        //enviar energia para os motores
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            myOpMode.telemetry.update(); //  Suponha que esta seja a última coisa feita no loop.
        }
    }

    /**
     * Pare todos os motores.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    /**
     * Defina contagens e distâncias de odometria para zero.
     */
    public void resetOdometry() {
        readSensors();
        driveOdometerOffset = rawDriveOdometer;
        driveDistance = 0.0;
        driveController.reset(0);

        strafeOdometerOffset = rawStrafeOdometer;
        strafeDistance = 0.0;
        strafeController.reset(0);
    }

    /**
     * Redefine o rumo do robô para zero graus e também bloqueie esse rumo no controlador de rumo.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    /**
     * Ativar ou desativar a telemetria da unidade
     */
    public void showTelemetry(boolean show){
        showTelemetry = show;
    }
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * Esta classe é usada para implementar um controlador proporcional que pode calcular a potência de saída desejada
 * para obter um eixo para o valor do ponto de ajuste desejado.
 * Ele também implementa um limite de aceleração e uma potência máxima.
 */
class    ProportionalControl {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determina a potência necessária para obter o valor do ponto de ajuste desejado com base no novo valor de entrada.
     * Usa ganho proporcional e limita a taxa de variação da saída, bem como a saída máxima.
     * @param input  Valor atual de entrada de controle ao vivo (dos sensores)
     * @return potência de saída desejada.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize para +/- 180 se estivermos controlando o rumo
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Evite qualquer acumulação de saída do motor muito lenta
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calcule a potência de saída usando o ganho e limite-a aos limites
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Agora limite a taxa de mudança de saída (aceleração)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }
    public double getSetpoint() {return setPoint;}

    /**
     * Salva um novo ponto de ajuste e redefine o histórico de potência de saída.
     * Esta chamada permite que um limite de energia temporário seja definido para substituir o padrão.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Salva um novo ponto de ajuste e redefine o histórico de potência de saída.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Deixe todo o resto igual, basta reiniciar o temporizador de aceleração e definir a saída para 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }
}
