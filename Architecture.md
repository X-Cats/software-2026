# Architecture
Living document linking the physical design with software design.

## Intake

### Functionality
- moves fuel into the hopper
- moves in and out of frame perims
- moves to 2 set positions (`STOWED` and `DEPLOYED`)

### Actuators
- Motor that extends and retracts the intake
- Motor the runs the rollers

### Sensors 

### Commands
- `STOW` and `DEPLOY` intake 
- stop and run intake 

## Hopper

### Functionality
- Moves fuel around, pushing it into the shooter
- moves at one set speed 
- rotates in both directions 

### Actuators
- Motor that spins hopper to agitate the fuel

### Sensors

### Commands
- run and stop the hopper motor (left or right)

## Turret

### Functionality
- Point the shooter towards the goal
- limited rotation 
- PID controller for turret position 
- the turret is `PRIMED` when its at the right position for firing

### Actuators
- Motor that spins the turret around

### Sensors
- limit switch 
- limelight

### Commands
- `PRIME` the turret
  - use feild localization to get robot position to set the shooter position for scoring
- rotate the turret

## Shooter

### Functionality
- Send balls into the goal
- shooter will run at different speed for range control 
- hood will be at different positions for range control 
- velocity PID controller for flywheel
- the hood is `PRIMED` when its at the right position for firing 

### Actuators
- 2 motors that spin the flywheel for shooting
- 1 motor that changes the angle of the hood

### Sensors
- limit switch

### Commands
- `PRIME` the hood
  - use field localization to get distance from hub to set the shooter hood position for scoring

## Climber

### Functionality
- Climb in auto and endgame
- Climber can go to different setpoints (`STOWED`,`L1`,`L2`,`L3`)

### Actuators
- 2 motors; one for each hook that moves it up and down.

### Sensors
- limit switches

### Commands
- raise climber elevator to `L1`,`L2`,`L3`
- lower climber elevator to `STOWED`