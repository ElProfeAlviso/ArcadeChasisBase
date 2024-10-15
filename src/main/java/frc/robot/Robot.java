
//Paquete principal del robot y el proyecto JAVA Main.
package frc.robot;

//Librerias de Robot,framework de un robot timed, las funciones periodicas se ejecutan cada 20ms.
import edu.wpi.first.wpilibj.TimedRobot; //Framework de un robot basado en iteracion de tiempo.
import edu.wpi.first.wpilibj.Timer; //Clase de un temporizador para generar tiempo.
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //Clase para manejo de un robot con drive diferencial tipo Arcade

//Librerias de Motores
import edu.wpi.first.wpilibj.motorcontrol.VictorSP; //Clase para controladores de velocidad VictorSP
import edu.wpi.first.wpilibj.Servo;//Clase para control de un servomotor PWM

//LIbrerias Spark Max para Neos

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

//Librerias de Pneumatica
import edu.wpi.first.wpilibj.PneumaticHub; //Clase para controlar el modulo neumatico de REV
import edu.wpi.first.wpilibj.Compressor; //Clase para activar el compresor
import edu.wpi.first.wpilibj.Solenoid; //Clase para crear objetos de solenoide simple.

//Librerias de Sensores y IO
import edu.wpi.first.wpilibj.PS4Controller; //Clase para recibir control de un Joystick PS4
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput; //Clase para leer entradas en los puertos analogicos
import edu.wpi.first.wpilibj.DigitalInput;//Clase para leer señales en los puertos de entradas digitales
import edu.wpi.first.wpilibj.CounterBase; //Clase para contar los ticks en una entrada digital.
import edu.wpi.first.wpilibj.Encoder;//Clase para leer encoders de tipo cuadratura 4 señales en 2 canales A y B
import com.kauailabs.navx.frc.AHRS;//clase para un sensor de navegacion de tipo Attitude and Heading Reference System LA NAVX
import edu.wpi.first.wpilibj.SPI;//Clase para comunicar con el puerto SPI del Roborio.

//Librerias de Vision y Limelight
import edu.wpi.first.cameraserver.CameraServer;//Clase para crear el stream de una camara USB conectada al roborio.
import edu.wpi.first.cscore.UsbCamera;//Clase para crear una instancia de una camara USB
import edu.wpi.first.networktables.NetworkTable;//Clase para el Systema de envio de mensajes por red 
import edu.wpi.first.networktables.NetworkTableEntry;//Clase para crear entradas en el Systema de envio de mensajes por red 
import edu.wpi.first.networktables.NetworkTableInstance;//Clase para crear instancias de network tables
import edu.wpi.first.networktables.GenericEntry;//Clase para crear entradas genericas en network tables.

//Librerias de SmartDashboard y Shuffleboard.
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;//Clase para programar con codigo los widgets del shuffleboard.
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;//Clase para enviar datos al shuffleboard en lugar de smartdashboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;//Clase para seleccionar una pestaña en el shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;//Clase para crear un objeto de seleccion de autonomos
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Clase para lectura/escritura en Shuffleboard de forma directa.

//Librerias de utilidades de JAVA
import java.util.Map;//clase para establecer un minimo y un maximo en un widget custom.

//Librerias de Control PID
import edu.wpi.first.math.controller.PIDController;

//Librerias del giroscopio
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

//Librerias limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//Clase Principal de robot con framework timedrobot.
public class Robot extends TimedRobot {

	// Declaracion de constantes
	double setpoint;
	double anguloZ = 0;
	double ServoPosition = 0.5;
	double chasisSpeed = 0;

	double testSPC = 0;
	double testSPT = 0;
	double testSPS = 0;
	double errorSumIzq = 0;
	double errorSumDer = 0;
	double lastTimestamp = 0;
	double lastErrorIzq = 0;
	double lastErrorDer = 0;

	double lowposition = 1;
	double highposition = 9;
	double intakeSP = 0;

	// Creacion de instancias de Sensores
	AnalogInput distancia = new AnalogInput(1);
	private final AHRS navx = new AHRS(SPI.Port.kMXP);
	DigitalInput LimitSwitch = new DigitalInput(4);
	DigitalInput HallSwitch = new DigitalInput(5);
	DigitalInput ProxSwitch = new DigitalInput(6);
	// Se crean los encoders izquierdo y derecho y su mapeo de conexion.
	private final Encoder leftEncoder = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
	private final Encoder rightEncoder = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);

	// Creacion de instancias de motores
	private final Servo servo = new Servo(9);
	// Se crean los objetos de motores iquierdos y derechos.
	private final VictorSP leftMotors = new VictorSP(0);
	private final VictorSP rightMotors = new VictorSP(1);
	// Se crea el objeto de un drive diferencial y se le agregan los motores
	// izquierdos y derechos
	private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

	// Se agrega el controlador del PS4
	private final PS4Controller driverJoystick = new PS4Controller(0);
	// Se crea un timer
	private final Timer timer = new Timer();

	// Constantes de nombres de autonomos
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	// Se crean instancias de objetos del Shuffleboard
	ShuffleboardTab tab = Shuffleboard.getTab("ShuffleBoard");
	GenericEntry ShuffleBoard = tab.add("Distancia", 0).getEntry();
	GenericEntry motores = tab.addPersistent("Max Speed", 1).withWidget(BuiltInWidgets.kNumberSlider)
			.withProperties(Map.of("min", 0, "max", 1)).withSize(4, 2).withPosition(0, 5).getEntry();

	// Creacion de Instancia de networktable de limelight.
	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ty = table.getEntry("ty");
	NetworkTableEntry ta = table.getEntry("ta");

	// Creacion de instancias de objetos neumaticos.
	PneumaticHub m_pH = new PneumaticHub(1);
	private final Solenoid m_solenoid = m_pH.makeSolenoid(0);
	private final Compressor m_compressor = m_pH.makeCompressor();

	private final PIDController chasisPIDleft = new PIDController(0.025, 0.046, 0.0016);
	private final PIDController chasisPIDright = new PIDController(0.025, 0.046, 0.0016);

	// Motores Neos

	private final CANSparkMax intakeNeo = new CANSparkMax(3, MotorType.kBrushless);
	private final RelativeEncoder intakeEncoder = intakeNeo.getEncoder();
	private final PIDController intakePID = new PIDController(0.04, 0.003, 0.001);

	public Robot() {

	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		intakePID.setTolerance(0.5);
		SmartDashboard.putNumber("Intake Position", intakeEncoder.getPosition());
		SmartDashboard.putNumber("PID Intake", 0);

		// Menu para seleccionar Autonomo
		m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		m_chooser.addOption("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		// Activar modo Test en Driver station y shuffleboard.
		enableLiveWindowInTest(true);

		// Inicializar Sensor de distancia
		distancia.setOversampleBits(4);
		distancia.setAverageBits(4);

		// inicializar motores
		rightMotors.setInverted(true);// invierte el giro del motor derecho segun la posicion en el chasis.
		// inicializar Drive
		robotDrive.setMaxOutput(0.8); // Limita la velocidad maxima del Drive Train a 0.8 para pruebas.
		robotDrive.setDeadband(0.05);// Evita que se mueva el robot con movimientos muy minimos del joystick
		// Inicializar encoders
		leftEncoder.setSamplesToAverage(5); // El numero promedio de muestras antes de hacer el calculo.
		leftEncoder.setDistancePerPulse(1.0 / 360.0 * Math.PI * 6);// Ajusta la distancia recorrida por cada pulso de
																	// conteo.
		leftEncoder.setMinRate(1.0); //
		leftEncoder.reset();

		rightEncoder.setSamplesToAverage(5);
		rightEncoder.setDistancePerPulse(1.0 / 360.0 * Math.PI * 6);
		rightEncoder.setMinRate(1.0);// Es el movimiento minimo para indicar que el motor esta en movimiento.
		rightEncoder.reset();// Vuelve la cuenta del encoder a cero.

		// Inicializar datos en Dashboard para que inicien con valor inicial.
		SmartDashboard.putNumber("Chasis Speed", 0.5);
		SmartDashboard.putNumber("AutoCM", 0);
		SmartDashboard.putNumber("Set Point test", 100);
		SmartDashboard.putNumber("Motores/servo", 0.5);

		// Inicializacion de camara usb y stream automatico.
		UsbCamera camera = CameraServer.startAutomaticCapture("Camara Frontal", 0);
		camera.setResolution(320, 240);
		camera.setFPS(15);
		camera.setBrightness(50); // Ajustar el brillo
		camera.setExposureManual(50); // Ajustar la exposición

		// inicalizacion de Gyroscopio.
		navx.reset();

		// Limelight
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");

		
	}

	@Override
	public void robotPeriodic() {

		// read values periodically
		double x = tx.getDouble(0.0);
		double y = ty.getDouble(0.0);
		double area = ta.getDouble(0.0);

		// post to smart dashboard periodically
		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);

		// Lectura y acondicionamiento de señal de sensor de distancia.
		double sensorValue = distancia.getVoltage();
		final double scaleFactor = 1; // scale converting voltage to distance CM
		double distance = 5 * sensorValue * scaleFactor; // convert the voltage to distance
		SmartDashboard.putNumber("Sensores/Ultrasonic MB1200", distance); // write the value to the LabVIEW
																			// DriverStation

		// Envio de datos de sensores digitales al SmartDashboard.
		SmartDashboard.putBoolean("Sensores/Limit Switch", !LimitSwitch.get());
		SmartDashboard.putBoolean("Sensores/Hall Switch", !HallSwitch.get());
		SmartDashboard.putBoolean("Sensores/Prox Switch", ProxSwitch.get());
		// envio de datos de encoders al smartDashboard.
		SmartDashboard.putNumber("Sensores/LeftEncoder Distance inches",
				Math.round(leftEncoder.getDistance() * 100) / 100d);
		SmartDashboard.putNumber("Sensores/LeftEncoder Distance CM",
				Math.round(2.54 * leftEncoder.getDistance() * 100) / 100d);
		SmartDashboard.putNumber("Sensores/LeftEncoder Rate", leftEncoder.getRate());
		SmartDashboard.putNumber("Sensores/LeftEncoder raw", leftEncoder.getRaw());
		SmartDashboard.putNumber("Sensores/RightEncoder Distance inches",
				Math.round(rightEncoder.getDistance() * 100) / 100d);
		SmartDashboard.putNumber("Sensores/RightEncoder Distance CM",
				Math.round(2.54 * rightEncoder.getDistance() * 100) / 100d);
		SmartDashboard.putNumber("Sensores/RightEncoder Rate", rightEncoder.getRate());
		SmartDashboard.putNumber("Sensores/RightEncoder raw", rightEncoder.getRaw());

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
		anguloZ = (int) navx.getAngle();

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

		

		// Muestra los datos del sistema neumatico
		SmartDashboard.putBoolean("Sensores/Compresor", m_compressor.isEnabled());

		// Lectura de datos desde el smartdashboard.
		ServoPosition = SmartDashboard.getNumber("Motores/servo", 0.5);
		chasisSpeed = SmartDashboard.getNumber("Chasis Speed", 0.5);
		testSPC = SmartDashboard.getNumber("Set Point test", 100);
		testSPS = SmartDashboard.getNumber("Set Point square", 150);
		testSPT = SmartDashboard.getNumber("Set Point triangle", 200);

	}

	/** This function is run once each time the robot enters autonomous mode. */
	@Override
	public void autonomousInit() { // inicializa los encoders y timer en cero.
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
				double leftDistance = Math.round(2.54 * leftEncoder.getDistance() * 100) / 100d;// Escala la distancia
																								// del
																								// encoder en CM
				double rightDistance = Math.round(2.54 * rightEncoder.getDistance() * 100) / 100d;// Escala la distancia
																									// del
																									// encoder en CM //
																									// encoder en
																									// CM
				double chasisDistance = ((leftDistance + rightDistance) / 2); // obtiene un promedio de las 2
																				// distancias.
				// Control de avance en autonomo con encoders.
				int autoSPcm = 100;
				if (chasisDistance < autoSPcm) {
					robotDrive.arcadeDrive(0.5, 0);

				} else {

					robotDrive.arcadeDrive(0, 0);

				}
				break;
			case kDefaultAuto:
			default:
				// Control de autonomo por tiempo de 2 seconds
				if (timer.get() < 2.0) {
					// Drive forwards half speed, make sure to turn input squaring off
					robotDrive.arcadeDrive(0.5, 0.0, false);
				} else {
					robotDrive.stopMotor(); // stop robot
				}
				break;
		}

	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {

		navx.reset();
		navx.resetDisplacement();

	}

	/** This function is called periodically during teleoperated mode. */
	@Override
	public void teleopPeriodic() {
		// reset manual en teleop del gyroscopio.
		if (driverJoystick.getOptionsButton()) {
			navx.reset();
			navx.resetDisplacement();
		}

		servo.set(ServoPosition);

		double power = -driverJoystick.getLeftY();
		double turn = -driverJoystick.getRightX();
		robotDrive.arcadeDrive(power * chasisSpeed, turn * chasisSpeed);

	}

	/** This function is called once each time the robot enters test mode. */
	@Override
	public void testInit() {
		leftEncoder.reset();
		rightEncoder.reset();
		timer.restart();
		setpoint = 0;
		errorSumIzq = 0;
		errorSumDer = 0;
		lastTimestamp = Timer.getFPGATimestamp();
		lastErrorDer = 0;
		lastErrorIzq = 0;

	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {

		m_solenoid.set(true);

		if (driverJoystick.getCrossButton()) {

			setpoint = testSPC;
		} else if (driverJoystick.getTriangleButton()) {

			setpoint = testSPT;

		} else if (driverJoystick.getSquareButton()) {

			setpoint = testSPS;

		} else if (driverJoystick.getCircleButton()) {

			setpoint = 0;

		}

		if (driverJoystick.getL1Button()) {
			intakeSP = lowposition;

		} else if (driverJoystick.getR1Button()) {
			intakeSP = highposition;
		}

		double intakeoutPID = intakePID.calculate(intakeEncoder.getPosition(), intakeSP);

		intakeNeo.set(intakeoutPID);
		SmartDashboard.putNumber("Intake Position", intakeEncoder.getPosition());
		SmartDashboard.putNumber("PID Intake Output", intakeoutPID);
		SmartDashboard.putNumber("Intake Setpoint", intakeSP);

		double leftDistance = Math.round(2.54 * leftEncoder.getDistance() * 100) / 100d;// Escala la distancia del
																						// encoder
		double rightDistance = Math.round(2.54 * rightEncoder.getDistance() * 100) / 100d;// Escala la distancia del
																							// encoder

		// double chasisDistance = ((leftDistance + rightDistance) / 2); // obtiene un
		// promedio de las 2 distancias.
		/*
		 * double errorIzq = setpoint - leftDistance;
		 * double errorDer = setpoint - rightDistance;
		 * double dt = Timer.getFPGATimestamp() - lastTimestamp;
		 * double kI = 0.03;
		 * double iLimit = 1;
		 * 
		 * if (Math.abs(errorIzq)<iLimit) {
		 * errorSumIzq += errorIzq * dt;
		 * }
		 * 
		 * if (Math.abs(errorDer)<iLimit) {
		 * errorSumDer += errorDer * dt;
		 * }
		 * 
		 * double kD =0.001;
		 * 
		 * double errorrateDer = (errorDer - lastErrorDer)/dt;
		 * double errorrateIzq = (errorIzq - lastErrorIzq)/dt;
		 * 
		 * double kP = 0.03;
		 * double outputSpeedDerecha = (kP * errorDer) + (kI *
		 * errorSumDer)+(kD*errorrateDer);
		 * double outputSpeedIzquierda = (kP * errorIzq) + (kI *
		 * errorSumIzq)+(kD*errorrateIzq);
		 * 
		 * double ScaledOutputSpeedDer = Math.max(-1.0, Math.min(1.0,
		 * outputSpeedDerecha));
		 * double ScaledOutputSpeedIzq = Math.max(-1.0, Math.min(1.0,
		 * outputSpeedIzquierda));
		 * lastTimestamp = Timer.getFPGATimestamp();
		 * lastErrorDer = errorDer;
		 * lastErrorIzq = errorIzq;
		 * // Output to motors
		 */

		double ScaledOutputSpeedIzq = chasisPIDleft.calculate(leftDistance, setpoint);
		double ScaledOutputSpeedDer = chasisPIDright.calculate(rightDistance, setpoint);

		leftMotors.set(ScaledOutputSpeedIzq);
		rightMotors.set(ScaledOutputSpeedDer);
		//
		//
		if(m_limelight.hasTarget()){
			double tx = m_limelight.getTx
		}

		// robotDrive.arcadeDrive(outputSpeed, 0);

		// Muestra los valores en autonomo.
		SmartDashboard.putNumber("PIDChasis/Set Point", setpoint);
		SmartDashboard.putNumber("PIDChasis/Right Position", rightDistance);
		SmartDashboard.putNumber("PIDChasis/Left Position", leftDistance);
		SmartDashboard.putNumber("PIDIzquierda/outputSpeed %", ScaledOutputSpeedIzq * 100);
		SmartDashboard.putNumber("PIDDerecha/outputSpeed %", ScaledOutputSpeedDer * 100);

	}

}
