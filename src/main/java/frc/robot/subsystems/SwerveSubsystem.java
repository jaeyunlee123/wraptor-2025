package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
class SwerveModule{

    /* 
     * A SwerveModule must have a drive motor, a steer motor, and an Abs. Encoder
     * The drive motor will be used to move the robot, the steer motor will be used to change the direction of the robot
     */



    
    private Talon drivemotor;
    private Talon steermotor;
    //Variable regarding the state of the robot
    private SwerveModuleState currentstate;
    //private SwerveModuleState desiredstate;
    private SwerveModuleState desiredstate;
    //We need 2 PID controllers for DRive and steer
    PIDController drivecontroller;
    PIDController steercontroller;
    //Simulation
    FlywheelSim steeringsim = new FlywheelSim(DCMotor.getKrakenX60(1), 150.0/7.0, 0.004);

    public SwerveModule(int drivemotorport,int steermotorport){
        drivemotor=new Talon(drivemotorport);
        steermotor=new Talon(steermotorport);
        currentstate= new SwerveModuleState();
        drivemotor.set(1);
        //Initialize the PID 
        drivecontroller=new PIDController(0.1,0,0);
        steercontroller=new PIDController(0.1,0,0);
    }
    
    public SwerveModuleState getState(){
        return currentstate;
    }
    
    public void setdesiredState(SwerveModuleState newState){
        desiredstate=newState;
    }
    
    /* 
    public void setState(SwerveModuleState newState){
        currentstate=newState;
    }
    */
    public void periodic(){
        //Run the simulation
        steeringsim.update(0.02);

        //Get the new simulated angle since last time to compare
        double simulatedanglediffrad=steeringsim.getAngularVelocityRadPerSec()*0.02;

        //Set new angle
        currentstate=new SwerveModuleState(currentstate.speedMetersPerSecond, Rotation2d.fromDegrees(currentstate.angle.getDegrees()+Units.radiansToDegrees(simulatedanglediffrad)));




    }
}
public class SwerveSubsystem extends SubsystemBase{
//Constructor
    public SwerveDriveOdometry swerveodometry;
    public Pigeon2 gyro;
    SwerveModule frontleftmodule=new SwerveModule(0,1);
    SwerveModule frontrightmodule=new SwerveModule(2,3);
    SwerveModule backleftmodule=new SwerveModule(4,5);
    SwerveModule backrightmodule=new SwerveModule(6,7);
    //Define kinematics. (Param is chassisspeed, and return swervemodulestates)
    double chassisWidth=Units.inchesToMeters(32);
    double chassislength=Units.inchesToMeters(32);

    //Defining Translation2d(Location of swervemodules in respect to the center)
    Translation2d frontleftlocation=new Translation2d(chassislength/2, chassisWidth/2);
    Translation2d frontrightlocation=new Translation2d(chassislength/2, -chassisWidth/2);
    Translation2d backleftlocation=new Translation2d(-chassislength/2, chassisWidth/2);
    Translation2d backrightlocation=new Translation2d(-chassislength/2, -chassisWidth/2);

    SwerveDriveKinematics kinematics=new SwerveDriveKinematics(frontleftlocation, frontrightlocation, backleftlocation, backrightlocation);
    CommandXboxController controller;
    
    //Getting info regarding the inputs of the xbox controller
    public SwerveSubsystem(CommandXboxController m_driverController){
        System.out.println("hi");
        controller=m_driverController;
    }
    public void setChassisSpeed(ChassisSpeeds desired){

            //Get the desired states of wheels from desired speed of the robot
            SwerveModuleState[] newStates=kinematics.toSwerveModuleStates(desired);
            
            
            frontleftmodule.setdesiredState(newStates[0]);
            frontrightmodule.setdesiredState(newStates[1]);
            backleftmodule.setdesiredState(newStates[2]);
            backrightmodule.setdesiredState(newStates[3]);
    }

    @Override
    public void periodic(){


        //Read data from controller
        ChassisSpeeds newdesiredSpeeds = new ChassisSpeeds(
        controller.getLeftY(),
        //Pushing forward will make it forward
        controller.getLeftX(),
        //Pushing left will make it go elft
        controller.getRightX()
        );
        //System.out.println(newdesiredSpeeds);
        setChassisSpeed(newdesiredSpeeds);
        //fl,fr,bl,br
        double loggingState[] = {
            frontleftmodule.getState().angle.getDegrees(),
            frontleftmodule.getState().speedMetersPerSecond,
            frontrightmodule.getState().angle.getDegrees(),
            frontrightmodule.getState().speedMetersPerSecond,
            backleftmodule.getState().angle.getDegrees(),
            backleftmodule.getState().speedMetersPerSecond,
            backrightmodule.getState().angle.getDegrees(),
            backrightmodule.getState().speedMetersPerSecond
        };
        SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
    }
}
