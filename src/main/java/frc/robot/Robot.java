//Paquete principal del robot y el proyecto JAVA
package frc.robot;

//Clases que se van a utilizar en el proyecto.
import edu.wpi.first.wpilibj.TimedRobot;  //Clase de framework de un robot timed, las funciones periodicas se ejecutan cada 20ms.
import edu.wpi.first.wpilibj.Timer;       //Clase de un temporizador para calcular tiempo.
import edu.wpi.first.wpilibj.PS4Controller;//Clase para interface de un control PS4
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //Clase para manejo de un robot con drive diferencial 
import edu.wpi.first.wpilibj.motorcontrol.VictorSP; //Clase para controladores de velocidad VictorSP
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Clase para lectura/escritura en Shuffleboard.
import frc.robot.LimelightHelpers;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer; //Clase para iniciar la transmision USB de webcam
import edu.wpi.first.cscore.UsbCamera; //Clase para crear objeto de camara USB y configurar sus parametros de video.
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.CounterBase; //Clase para conteo de ticks de encoders.
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder; //Clase para interface con sensores encoders.
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;



public class Robot extends TimedRobot {

  Servo servo = new Servo(9);
  
   private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Clase principal del  principal del robot aqui se ponen variables globales.



  private final AHRS navx = new AHRS(SPI.Port.kMXP);
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

  AnalogInput distancia = new AnalogInput(1);
  

  ShuffleboardTab tab = Shuffleboard.getTab("ShuffleBoard");
  //GenericEntry ShuffleBoard = tab.add("Distancia", 0).getEntry();
  //GenericEntry motores = tab.addPersistent("Max Speed", 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min",0,"max",1)).withSize(4, 2).withPosition(0, 5).getEntry();

  double setpoint;
  double anguloZ = 0;


  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  PneumaticHub m_pH = new PneumaticHub(1);
  private final Solenoid m_solenoid = m_pH.makeSolenoid(0);
  private final Compressor m_compressor = m_pH.makeCompressor();

  //Constructor de la clase robot.
  public Robot() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    distancia.setOversampleBits(4);
    distancia.setAverageBits(4);

    robotDrive.setDeadband(0.05);//Evita que se mueva el robot con movimientos muy minimos del joystick

    rightMotors.setInverted(true);//invierte el giro del  motor derecho segun la posicion en el chasis.
    robotDrive.setMaxOutput(0.8); //Limita la velocidad maxima del Drive Train a 0.8 para pruebas.

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
    SmartDashboard.putNumber("Motores/servo", 0.5);

   

      //Inicia la captura automática de la cámara USB
        UsbCamera camera = CameraServer.startAutomaticCapture("Camara Frontal", 0);
        // Establece la resolución y la velocidad de cuadros
        camera.setResolution(320, 240);
        camera.setFPS(15);
        // Configuración adicional si es necesaria
        camera.setBrightness(50); // Ajustar el brillo
        camera.setExposureManual(50); // Ajustar la exposición
        
        anguloZ = 0;
        
        navx.reset();
  }
  
  @Override
  public void robotPeriodic() {

    if (driverJoystick.getOptionsButton()) {
      
      navx.reset();
      navx.resetDisplacement();

    }

   
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
    final double scaleFactor = 1; // scale converting voltage to distance CM
    double distance = 5 * sensorValue * scaleFactor; // convert the voltage to distance
    SmartDashboard.putNumber("Sensores/Ultrasonic MB1200", distance); // write the value to the LabVIEW DriverStation

    //ShuffleBoard.setDouble(distance);
    //motores.getDouble(1);



    // Leer los datos del NavX y mostrarlos en el SmartDashboard
    double yaw = navx.getYaw(); // Ángulo de rotación alrededor del eje Z
    double pitch = navx.getPitch(); // Ángulo de inclinación alrededor del eje Y
    double roll = navx.getRoll(); // Ángulo de inclinación alrededor del eje X
    double compassHeading = navx.getCompassHeading(); // Brújula: 0 a 360 grados
    double fusedHeading = navx.getFusedHeading(); // Combinación de brújula y giroscopio
    double altitude = navx.getAltitude(); // Altitud basada en presión barométrica
    double linearAccelX = navx.getWorldLinearAccelX(); // Aceleración lineal en el eje X
    double linearAccelY = navx.getWorldLinearAccelY(); // Aceleración lineal en el eje Y
    double linearAccelZ = navx.getWorldLinearAccelZ(); // Aceleración lineal en el eje Z
    double totalAccelX = navx.getRawAccelX(); // Aceleración total en el eje X
    double totalAccelY = navx.getRawAccelY(); // Aceleración total en el eje Y
    double totalAccelZ = navx.getRawAccelZ(); // Aceleración total en el eje Z
    double velocityX = navx.getVelocityX(); // Velocidad en el eje X
    double velocityY = navx.getVelocityY(); // Velocidad en el eje Y
    double velocityZ = navx.getVelocityZ(); // Velocidad en el eje Z
    double displacementX = navx.getDisplacementX(); // Desplazamiento en el eje X
    double displacementY = navx.getDisplacementY(); // Desplazamiento en el eje Y
    double displacementZ = navx.getDisplacementZ(); // Desplazamiento en el eje Z
    double temperature = navx.getTempC(); // Temperatura del sensor en grados Celsius
     anguloZ = (int)navx.getAngle();

    // Mostrar los datos en el SmartDashboard
    SmartDashboard.putNumber("Navx/Yaw", yaw);
    SmartDashboard.putNumber("Navx/Pitch", pitch);
    SmartDashboard.putNumber("Navx/Roll", roll);
    SmartDashboard.putNumber("Navx/Compass Heading", compassHeading);
    SmartDashboard.putNumber("Navx/Fused Heading", fusedHeading);
    SmartDashboard.putNumber("Navx/Altitude", altitude);
    SmartDashboard.putNumber("Navx/Linear Accel X", linearAccelX);
    SmartDashboard.putNumber("Navx/Linear Accel Y", linearAccelY);
    SmartDashboard.putNumber("Navx/Linear Accel Z", linearAccelZ);
    SmartDashboard.putNumber("Navx/Total Accel X", totalAccelX);
    SmartDashboard.putNumber("Navx/Total Accel Y", totalAccelY);
    SmartDashboard.putNumber("Navx/Total Accel Z", totalAccelZ);
    SmartDashboard.putNumber("Navx/Velocity X", velocityX);
    SmartDashboard.putNumber("Navx/Velocity Y", velocityY);
    SmartDashboard.putNumber("Navx/Velocity Z", velocityZ);
    SmartDashboard.putNumber("Navx/Displacement X", displacementX);
    SmartDashboard.putNumber("Navx/Displacement Y", displacementY);
    SmartDashboard.putNumber("Navx/Displacement Z", displacementZ);
    SmartDashboard.putNumber("Navx/Temperature", temperature);
    SmartDashboard.putNumber("Navx/Angulo", anguloZ);


    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("Vision/LimelightX", x);
    SmartDashboard.putNumber("Vision/LimelightY", y);
    SmartDashboard.putNumber("Vision/LimelightArea", area);

    

    servo.set(SmartDashboard.getNumber("Motores/servo", 0.5));

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() { //inicializa los encoders y timer en cero.
    leftEncoder.reset();
    rightEncoder.reset();
    timer.restart();

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }

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
  public void teleopInit() {

    navx.reset();
    navx.resetDisplacement();

  }

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


      m_solenoid.set(true);
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
