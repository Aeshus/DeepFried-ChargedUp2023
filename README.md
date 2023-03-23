# DeepFried-ChargedUp2023

Benjamin Franklin High School's Voodoo Voltage code-base for the Elevator Robot!

## Building

```bash
./gradelw
```

## Directories

├── `build` - Buildfiles  
├── `gradlew` - Gradle Executable  
├── `PathWeaver` - Paths(?)  
│   ├── Paths  
│   │   ├── DockPath2.path  
│   │   ├── DockPath.path  
│   │   ├── EngageB.path  
│   │   ├── MobilityPath1.path  
│   │   ├── MobilityPathR.path  
│   │   ├── PreloadPath1B.path  
│   │   └── Unnamed.path  
│   └── pathweaver.json  
├── `README.md` - This file  
├── `src` - Code source  
│   └── main  
│       ├── deploy  
│       │   ├── example.txt  
│       │   └── `output` - Put into `/output`  
│       │       ├── CubePickUp.wpilib.json  
│       │       ├── CubeReturn.wpilib.json  
│       │       ├── DockPath2.wpilib.json  
│       │       ├── DockPath.wpilib.json  
│       │       ├── EngageB.wpilib.json  
│       │       ├── MobilityPath1.wpilib.json  
│       │       ├── MobilityPathR.wpilib.json  
│       │       ├── PreloadPath1B.wpilib.json  
│       │       ├── RspA.wpilib.json  
│       │       ├── Test2.wpilib.json  
│       │       ├── Test.wpilib.json  
│       │       └── Unnamed.wpilib.json  
│       └── `java` - Robot source code  
│           └── frc  
│               └── robot  
│                   ├── commands  
│                   │   ├── AutoBalanceB.java  
│                   │   ├── AutoBalanceF.java  
│                   │   ├── ConeRelease.java  
│                   │   ├── CubeRelease.java  
│                   │   ├── ElevatorRaiseMid.java  
│                   │   ├── ElevatorRaiseTop.java  
│                   │   ├── IntakeRelease.java  
│                   │   ├── IntakeStow.java  
│                   │   ├── InvertMotors.java  
│                   │   ├── LowerELevator.java  
│                   │   └── LowerIntake.java  
│                   ├── Constants.java  
│                   ├── Main.java  
│                   ├── RobotContainer.java  
│                   ├── Robot.java  
│                   └── subsystems  
│                       ├── autoBalance.java  
│                       ├── DriveBase.java  
│                       ├── ElevatorPID.java  
│                       └── IntakeSub.java  
└── WPILib-License.md  
