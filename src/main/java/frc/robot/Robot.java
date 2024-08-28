//Paquete principal del robot y el proyecto JAVA
package frc.robot;

//Clases que se van a utilizar en el proyecto.
import edu.wpi.first.wpilibj.TimedRobot;  //Clase de framework de un robot timed, las funciones periodicas se ejecutan cada 20ms.
import edu.wpi.first.wpilibj.Timer;       //Clase de un temporizador para calcular tiempo.
import edu.wpi.first.wpilibj.PS4Controller;//Clase para interface de un control PS4
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //Clase para manejo de un robot con drive diferencial 
import edu.wpi.first.wpilibj.motorcontrol.VictorSP; //Clase para controladores de velocidad VictorSP
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Clase para lectura/escritura en Shuffleboard.

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer; //Clase para iniciar la transmision USB de webcam
import edu.wpi.first.cscore.UsbCamera; //Clase para crear objeto de camara USB y configurar sus parametros de video.
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.CounterBase; //Clase para conteo de ticks de encoders.
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder; //Clase para interface con sensores encoders.
import edu.wpi.first.wpilibj.AnalogInput;




public class Robot extends TimedRobot {  //Clase principal del  principal del robot aqui se ponen variables globales.

  //Se crean los objetos de motores iquierdos y derechos.
  private final VictorSP leftMotors = new VictorSP(0);
  private final VictorSP rightMotors = new VictorSP(1);

  //Se crea el objeto de un drive diferencial y se le agregan los motores izquierdos y derechos
  private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);
  //Se agrega el controlador del PS4
  private final PS4Controller driverJoystick = new PS4Controller(0);
  //Se crea un timer
  private final Timer timer = new Timer();
  //Se crean los encoders izquierdo y derecho y su mapeo de conexion.
  private final Encoder leftEncoder = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
  private final Encoder rightEncoder = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
  DigitalInput LimitSwitch = new DigitalInput(4);
  DigitalInput HallSwitch = new DigitalInput(5);
  DigitalInput ProxSwitch = new DigitalInput(6);

  AnalogInput distancia = new AnalogInput(0);

  ShuffleboardTab tab = Shuffleboard.getTab("ShuffleBoard");
  GenericEntry ShuffleBoard = tab.add("Distancia", 0).getEntry();
  GenericEntry motores = tab.addPersistent("Max Speed", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",1)).withSize(4, 2).withPosition(0, 5).getEntry();

  double setpoint;

  //Constructor de la clase robot.
  public Robot() {
    
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    robotDrive.setDeadband(0.05);//Evita que se mueva el robot con movimientos muy minimos del joystick

    rightMotors.setInverted(true);//invierte el giro del  motor derecho segun la posicion en el chasis

    //Inicializar encoders
    leftEncoder.setSamplesToAverage(5); //El numero promedio de muestras antes de hacer el calculo.
    leftEncoder.setDistancePerPulse(1.0 / 360.0 * Math.PI * 6);//Ajusta la distancia recorrida por cada pulso de conteo.
    leftEncoder.setMinRate(1.0); //
    leftEncoder.reset();

    rightEncoder.setSamplesToAverage(5);
    rightEncoder.setDistancePerPulse(1.0 / 360.0 * Math.PI * 6);
    rightEncoder.setMinRate(1.0);//Es el movimiento minimo para indicar que el motor esta en movimiento.
    rightEncoder.reset();//Vuelve la cuenta del encoder a cero.

    //Inicializar datos en Dashboard
    SmartDashboard.putNumber("Chasis Speed", 0.5);
    SmartDashboard.putNumber("AutoCM", 0);
    SmartDashboard.putNumber("Set Point test", 100);

   

      //Inicia la captura automática de la cámara USB
        UsbCamera camera = CameraServer.startAutomaticCapture("Camara Frontal", 0);
        // Establece la resolución y la velocidad de cuadros
        camera.setResolution(320, 240);
        camera.setFPS(15);
        // Configuración adicional si es necesaria
        camera.setBrightness(50); // Ajustar el brillo
        camera.setExposureManual(50); // Ajustar la exposición

        
        

  }
  
  @Override
  public void robotPeriodic() {


   
    SmartDashboard.putBoolean("Sensores/Limit Switch", !LimitSwitch.get());
    SmartDashboard.putBoolean("Sensores/Hall Switch", !HallSwitch.get());
    SmartDashboard.putBoolean("Sensores/Prox Switch", ProxSwitch.get());

    SmartDashboard.putNumber("Sensores/LeftEncoder Distance inches", Math.round(leftEncoder.getDistance() * 100) / 100d);
    SmartDashboard.putNumber("Sensores/LeftEncoder Distance CM", Math.round(2.54 * leftEncoder.getDistance() * 100) / 100d);
    SmartDashboard.putNumber("Sensores/LeftEncoder Rate", leftEncoder.getRate());
    SmartDashboard.putNumber("Sensores/LeftEncoder raw", leftEncoder.getRaw());
    SmartDashboard.putNumber("Sensores/RightEncoder Distance inches", Math.round(rightEncoder.getDistance() * 100) / 100d);
    SmartDashboard.putNumber("Sensores/RightEncoder Distance CM", Math.round(2.54 * rightEncoder.getDistance() * 100) / 100d);
    SmartDashboard.putNumber("Sensores/RightEncoder Rate", rightEncoder.getRate());
    SmartDashboard.putNumber("Sensores/RightEncoder raw", rightEncoder.getRaw());


    double sensorValue = distancia.getVoltage();
    final double scaleFactor = 1 / (5. / 1024.); // scale converting voltage to distance
    double distance = 5 * sensorValue * scaleFactor; // convert the voltage to distance
    SmartDashboard.putNumber("Sensores/Ultrasonic MB1200", distance); // write the value to the LabVIEW DriverStation

    ShuffleBoard.setDouble(distance);
    motores.getDouble(1);



  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() { //inicializa los encoders y timer en cero.
    leftEncoder.reset();
    rightEncoder.reset();
    timer.restart();
    
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    double leftDistance = Math.round(2.54 * leftEncoder.getDistance() * 100) / 100d;// Escala la distancia del encoder en CM
    double rightDistance = Math.round(2.54 * rightEncoder.getDistance() * 100) / 100d;// Escala la distancia del encoder en CM
    double chasisDistance = ((leftDistance + rightDistance) / 2); // obtiene un promedio de las 2 distancias.
   
    //Control de avance en autonomo con encoders.

    
    int autoSPcm = 100;
    if (chasisDistance < autoSPcm) {
      robotDrive.arcadeDrive(0.5, 0);
      
    } else {
    
      robotDrive.arcadeDrive(0,0);
    
    }
    
    
    // Control de autonomo por tiempo de  2 seconds
    /*if (timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      robotDrive.stopMotor(); // stop robot
    }
    */
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

        double chasisSpeed = SmartDashboard.getNumber("Chasis Speed", 0.5);

    double power = -driverJoystick.getLeftY();
    double turn = -driverJoystick.getRightX();

    robotDrive.arcadeDrive( power * chasisSpeed, turn * chasisSpeed);

    

    

    
    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    leftEncoder.reset();
    rightEncoder.reset();
    timer.restart();
    setpoint = 0;


  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    double testSP = SmartDashboard.getNumber("Set Point test", 100);

    if (driverJoystick.getCrossButton()) {

      setpoint = testSP;
    } else if (driverJoystick.getCircleButton()) {

      setpoint = 0;

    }

    double leftDistance = Math.round(2.54 * leftEncoder.getDistance() * 100) / 100d;// Escala la distancia del encoder en CM
    double rightDistance = Math.round(2.54 * rightEncoder.getDistance() * 100) / 100d;// Escala la distancia del encoder en CM
    double chasisDistance = ((leftDistance + rightDistance) / 2); // obtiene un promedio de las 2 distancias.

    double error = setpoint - rightDistance;
    double kP = 0.05;
    double outputSpeed = kP * error;
    double ScaledOutputSpeed = Math.max(-1.0, Math.min(1.0, outputSpeed));

    // Output to motors

    leftMotors.set(ScaledOutputSpeed);
    rightMotors.set(ScaledOutputSpeed*0.95);

    //robotDrive.arcadeDrive(outputSpeed, 0);

    // Muestra los valores en autonomo.

    SmartDashboard.putNumber("PIDChasis/Set Point", setpoint);
    SmartDashboard.putNumber("PIDChasis/Right Position", rightDistance);
    SmartDashboard.putNumber("PIDChasis/Left Position", leftDistance);
    SmartDashboard.putNumber("PIDChasis/outputSpeed %", ScaledOutputSpeed*100);



  }
  

  //Dashboard Outs

  


  







}
