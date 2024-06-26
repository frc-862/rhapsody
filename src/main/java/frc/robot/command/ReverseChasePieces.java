// package frc.robot.command;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.util.datalog.DataLog;
// import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.util.datalog.BooleanLogEntry;
// import edu.wpi.first.util.datalog.DoubleLogEntry;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AutonomousConstants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Collector;
// import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.Limelights;
// import frc.thunder.vision.Limelight;

// public class ReverseChasePieces extends Command {

//     private Swerve drivetrain;
//     private Collector collector;
//     private Indexer indexer;
//     private Limelight limelight;

//     private double pidOutput;
//     private double targetHeading;
//     private double previousTargetHeading;

//     private double targetPitch;
//     private double collectPower;
//     private double maxCollectPower;
//     private double drivePower;
//     private double rotPower;

//     private boolean onTarget;
//     private boolean hasPiece;
//     private boolean isDone;
//     private boolean hasTarget;
//     private boolean hasSeenTarget;

//     private Command smartCollect;
//     private PIDController headingController = VisionConstants.CHASE_CONTROLLER;

//     private BooleanLogEntry onTargetLog;
//     private BooleanLogEntry hasTargetLog;
//     private BooleanLogEntry trustValuesLog;
//     private BooleanLogEntry isDoneLog;
//     private BooleanLogEntry hasPieceLog;

//     private DoubleLogEntry targetHeadingLog;
//     private DoubleLogEntry targetYLog;
//     private DoubleLogEntry pidOutputLog;
//     private DoubleLogEntry smartCollectPowerLog;
//     private DoubleLogEntry drivePowerLog;
//     private DoubleLogEntry maxCollectPowerLog;

//     /**
//      * Creates a new ChasePieces.
//      *
//      * @param drivetrain to request movement
//      * @param collector  for smart collect
//      * @param indexer    for smart collect
//      * @param limelights to get vision data from dust
//      */
//     public ReverseChasePieces(Swerve drivetrain, Collector collector, Indexer indexer, Limelights limelights) {
//         this.drivetrain = drivetrain;
//         this.collector = collector;
//         this.indexer = indexer;

//         this.limelight = limelights.getDust();

//         this.drivePower = 1.5d;
//         this.rotPower = 1.5d;
//         this.maxCollectPower = 1d;

//         addRequirements(drivetrain, collector, indexer);

//         initLogging();
//     }

//     @Override
//     public void initialize() {
//         System.out.println("AUTO - Reverse Chase pieces INIT");

//         headingController.setTolerance(VisionConstants.ALIGNMENT_TOLERANCE);
//         collectPower = maxCollectPower;
        
//         smartCollect = new AutonSmartCollect(() -> collectPower, () -> collectPower, collector, indexer);

//         smartCollect.initialize();

//         hasSeenTarget = false;
//     }

//     /**
//      * Initialize logging
//      */
//     private void initLogging() {
//         DataLog log = DataLogManager.getLog();

//         onTargetLog = new BooleanLogEntry(log, "/ChasePieces/On Target");
//         hasTargetLog = new BooleanLogEntry(log, "/ChasePieces/Has Target");
//         trustValuesLog = new BooleanLogEntry(log, "/ChasePieces/Trust Values");
//         isDoneLog = new BooleanLogEntry(log, "/ChasePieces/Is Done");
//         hasPieceLog = new BooleanLogEntry(log, "/ChasePieces/Has Piece");

//         targetHeadingLog = new DoubleLogEntry(log, "/ChasePieces/Target Heading");
//         targetYLog = new DoubleLogEntry(log, "/ChasePieces/Target Y");
//         pidOutputLog = new DoubleLogEntry(log, "/ChasePieces/Pid Output");
//         smartCollectPowerLog = new DoubleLogEntry(log, "/ChasePieces/SmartCollectPower");
//         drivePowerLog = new DoubleLogEntry(log, "/ChasePieces/DrivePower");
//         maxCollectPowerLog = new DoubleLogEntry(log, "/ChasePieces/MaxCollectPower");
//     }

//     @Override
//     public void execute() {
//         hasTarget = limelight.hasTarget();
//         smartCollect.execute();

//         if (hasTarget) {
//             previousTargetHeading = targetHeading;
//             targetHeading = limelight.getTargetX();
//             targetPitch = limelight.getTargetY();
//         }

//         onTarget = Math.abs(targetHeading) < VisionConstants.ALIGNMENT_TOLERANCE;
//         hasPiece = indexer.hasNote();

//         pidOutput = headingController.calculate(0, targetHeading);

//         collectPower = maxCollectPower;
//         if (!hasPiece) {
//             if (hasTarget) {
//                 if (trustValues()) {
//                     hasSeenTarget = true;
//                     if (!onTarget) {
//                         drivetrain.setRobot(drivePower, 0, -pidOutput);
//                     } else {
//                         drivetrain.setRobot(drivePower, 0, 0);
//                     }
//                 }
//             } else {
//                 if (!hasSeenTarget) {
//                     if (drivetrain.getPose().getY() > VisionConstants.HALF_FIELD_HEIGHT) {
//                         if (DriverStation.getAlliance().get() == Alliance.Blue) {
//                             drivetrain.setRobot(0.5, 0, rotPower);
//                         } else {
//                             drivetrain.setRobot(0.5, 0, -rotPower);
//                         }
//                     } else {
//                         if (DriverStation.getAlliance().get() == Alliance.Blue) {
//                             drivetrain.setRobot(0.5, 0, -rotPower);
//                         } else {
//                             drivetrain.setRobot(0.5, 0, rotPower);
//                         }
//                     }
//                 } else {
//                     drivetrain.setRobot(drivePower, 0, 0);
//                 }
//             }
//         }


//         updateLogging();
//     }

//     /**
//      * Update logging
//      */
//     private void updateLogging() {
//         onTargetLog.append(onTarget);
//         hasTargetLog.append(hasTarget);
//         trustValuesLog.append(trustValues());
//         isDoneLog.append(isDone);
//         hasPieceLog.append(hasPiece);

//         targetHeadingLog.append(targetHeading);
//         targetYLog.append(targetPitch);
//         pidOutputLog.append(pidOutput);
//         smartCollectPowerLog.append(collectPower);
//         drivePowerLog.append(drivePower);
//         maxCollectPowerLog.append(maxCollectPower);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         collectPower = 0d;
//         smartCollect.end(interrupted);

//         System.out.println("AUTO - Chase pieces END");
//     }

//     /**
//      * Makes sure that the robot isn't jerking over to a different side while
//      * chasing pieces.
//      *
//      * @return t/f if the robot should trust the values
//      */
//     public boolean trustValues() {
//         if ((Math.abs(targetHeading) - Math.abs(previousTargetHeading)) < 6) {
//             return true;
//         }
//         return false;
//     }

//     @Override
//     public boolean isFinished() {
//         if (DriverStation.getAlliance().get() == Alliance.Blue) {
//             if (drivetrain.getPose().getX() > AutonomousConstants.CHASE_BOUNDARY) {
//                 return true;
//             }
//         } else {
//             if (drivetrain.getPose().getX() < AutonomousConstants.CHASE_BOUNDARY) {
//                 return true;
//             }
//         }
//         return indexer.hasNote();
//     }
// }