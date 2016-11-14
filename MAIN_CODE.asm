; MAIN_CODE.asm
; Created by team Harambe and the Boiz
; Team members: Randy Deng, Jeffrey Zhao, Tejasvi Nareddy, Kavin Krishnan, Hope Hong

;*************************************************
; Initialization
;*************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	OUT    BEEP        ; Stop any beeping (optional)

	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)

WaitForSafety:
	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety

WaitForUser:
	; This loop will wait for the user to press PB3, to ensure that
	; they have a chance to prepare for any movement in the main code.
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue


;**************************************************
; Main Code
;**************************************************

; If necessary, put any initialization
; data for main here
Main:
	; TODO initialize array to maxDistance
	
	; Reset odometer in case wheels move after programming
	OUT 	RESETPOS	
	CALL	InitializeVars
	CALL	InitialSearch

	; TODO we need some way to keep track of the number of objects left
	; while (numObjects > 0) { call FindAndTagClosestObject }
	CALL	FindAndTagClosestObject
	; Reset odometer in case wheels move after programming
	OUT 	RESETPOS

; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
Die:
		LOAD   Zero         ; Stop everything.
		OUT    LVELCMD
		OUT    RVELCMD
		OUT    SONAREN
		LOAD   DEAD         ; An indication that we are dead
		OUT    SSEG2        ; "dEAd" on the LEDs
	
	; Our version of HALT
	Forever:
		JUMP   Forever      ; Do this forever.
		DEAD:  DW &HDEAD    ; Example of a "local" variable
		
; =================== ;
; END OF CONTROL FLOW ;
; =================== ;

;**************************************************
; Important Subroutines
;**************************************************

InitializeVars:
		; SW0 = AlongLongWall --> Sets var to 1 if the robot is travelling along the long wall
		IN		SWITCHES
		AND		MASK0
		STORE 	AlongLongWall
	
		; ObjectsPosTheta: Sets var to 1 if the robot must turn in the positive direction to tag objects.
		; Note: We turn positive if we are along the short edge for the given use case
		JZERO	PositiveThetaLoad
		JPOS	ZeroThetaLoad
		
	PositiveThetaLoad:
		LOAD	ONE
		JUMP	ThetaStore
		
	ZeroThetaLoad:
		LOAD	ZERO
		JUMP ThetaStore
		
	ThetaStore:
		STORE	ObjectsPosTheta
	
		; Return!
		RETURN

; Initial search. Follow walls, updating the map based on objects that are perpendicular.
InitialSearch:
		
		; Enable sonar sensors 2 and 3 to make sure we don't run into anything
		; TODO do this with interrupts instead of just checking every loop cycle
		LOAD	MASK2
		ADD		MASK3
		OUT 	SONAREN

	; Go forward until we are at the end of the edge
	KeepGoingForward:

		; Update the map with the current sensor readings
		; Call Jeff's code here!
		CALL 	UpdateMap

		; Check the robot has gone too far in the x direction
		; Note that this distance depends on which wall we are following, stored in AlongLongWall
		LOAD	AlongLongWall
		JZERO	LoadShortDistance
		JPOS	LoadLongDistance

	; We are travelling along the long edge, so check this distance bound
	LoadLongDistance:
		IN		XPOS
		SUB		MaxLong
		JUMP	DistanceCheck

	; We are travelling along the short edge, so check this distance bound
	LoadShortDistance:
		IN		XPOS
		SUB		MaxShort
		JUMP 	DistanceCheck

	DistanceCheck:
		JPOS	DoneForward

		; TODO Check if we are about to hit an object with the ultrasonic sensors
		; TODO interrupts instead of checking at each loop?
		; CHECKME maybe we should aggregate this data as well? Should test :/

		; Keep going forward as we have not hit the max limit for the wall
		; FIXME tweak the speeds
		LOAD	ZERO
		STORE	DTheta
		LOAD	FMid
		STORE	DVel
		CALL	ControlMovement

		; Keep looping
		JUMP	KeepGoingForward

	; We are at the max bound of the wall now
	DoneForward:
		; Stop the wheels
		LOAD	ZERO
		OUT		LVELCMD
		OUT		RVELCMD

		; Rotate 180 (direction doesn't matter)
		LOAD	Deg180
		STORE	Angle
		CALL	Rotate

		; Return to main
		RETURN

UpdateMap:
	; Traverse an axis,and store the distance recieved (represents 32mm increment)
	LOAD	AlongLongWall
 	XOR 	XDir
	JPOS 	ELHS ; If bot is moving right, turn on sensors on the left side
	JZERO  	ERHS ; Same, but converse

	; FIXME turn on sensors based on which axis it is on AND the var XDir
		
	ERHS:
		;Enable sensors on right hand side of bot
	 	LOAD	MASK5
	 	OUT 	SONAREN
	 	IN 		DIST5 ;Turn on and read value from sensor 5
		CALL	CellIn ; If value read in less than the value already in cell, store it in cell
	 	RETURN

	ELHS:
		;Enable sensors on right hand side of bot
		LOAD	MASK0
		OUT 	SONAREN
		IN 		DIST0
	 	CALL	CellIn ; If value read in less than the value already in cell, store it in cell
		RETURN

	CellIn:
	; Store value of cell into memory adress pointed to by XposIndex
		STORE 	Cell ;store current distance read in cell
	 	IN		XPOS ;Take in xposition
		SHIFT 	five ;Index value of the array (applies same dist value cells of length 32 increments)
		ADDI	CellArrI ;Add the value of starting address (where the memory for array begins)
		STORE 	XposIndex ;Holds the adress where the dist value will be placed
		LOAD 	CELL
		ISTORE	XposIndex
		CALL	KillSonars
		RETURN

;Subroutine that filters the array created in update map
FilterArray:
	;TODO not every cell in the array will have a reading, we need to figure how to filter the readings to produce continuous object
	;Account for two object being at the same distance away from wall
	;Account for one object being behind another
	RETURN
	
; Finds the closest object (relative to the wall) based on the map
FindClosestObject:
	; TODO traverse through the array and get the xPos for the closest object
	; TODO store in ObjectXDist, store distance in ObjectYDist
	LOAD	TEN
	STORE	ObjectXDist
	STORE	ObjectYDist
	RETURN

; Goes to the x position the closest object is located at
; Turns toward object and tags it
; Then returns back home, retracing its path
FindAndTagClosestObject:

		; Call method to get information about the closest object
		CALL	FindClosestObject
		; Now, the x pos of the closest object is stored in ObjectXDist, the y pos is in ObjectYDist
		; TODO bounds check on the closest object (just in case?!?)

	; Go toward the object until we hit the x distance
	MoveTowardObject:

		;Checks where robot is relative to position to start
		IN	 	XPOS
		SUB 	ObjectXDist
		JNEG	GoRight
		JZERO 	AtObjectX
		JPOS	GoLeft

	; Robot has to go right on the X axis
	; XDir is stored as 1
	GoRight:
		LOAD 	ONE
		STORE 	XDir
		JUMP 	MoveLoop

	; Robot has to go left on the X axis
	; XDir is stored as 0
	GoLeft:
		LOAD 	ZERO
		STORE 	XDir

	;Decides which check to perform based on the XDir value
	MoveLoop:
		; Update the map with the current sensor readings
		CALL 	UpdateMap

		; Do the bounds check
		LOAD 	XDir
		JZERO 	CheckLess
		JPOS	CheckGreat

	; Checks if robot was before desired positon on X axis at start
	CheckGreat:
		IN		XPOS
		SUB		ObjectXDist
		JZERO 	AtObjectX
		JPOS	AtObjectX
		JUMP	KeepGoingInDirection

	;If robot was after desired positon on X axis at start
	CheckLess:
		IN		XPOS
		SUB		ObjectXDist
		JZERO 	AtObjectX
		JNEG	AtObjectX
		JUMP	KeepGoingInDirection

	;If robot is not yet at desired X position
	KeepGoingInDirection:
		
		; FIXME tweak the speeds
		LOAD	ZERO
		STORE	DTheta
		LOAD	FMid
		STORE	DVel
		CALL	ControlMovement
	
		; Check the bounds again!
		JUMP	MoveLoop

	AtObjectX:
		; Stop the robot
		LOAD	ZERO
		OUT		LVELCMD
		OUT		RVELCMD
	
		; TODO turn for Randy's tagging
		; TODO call Randy's tag method
		; TODO return to home

		; Return to main
		RETURN

; We are back at home now
BackAtHome:
	; TODO wait for user input to start again
	; For now just call die
	CALL 	Die
	RETURN

; GoHome function will have the robot go home after tagging
; Assumes DE2Bot is facing a wall and there is a clear straight path to wall
GoHome:
	; Go towards wall then stop when close
	CALL 	GoToWall
	; Determine which way to rotate
	LOADI 	90
	STORE 	Angle
	LOAD 	ObjectsPosTheta
	JZERO 	HomeRotate
	LOADI 	-90
	STORE 	Angle
HomeRotate:
	; Rotate to face wall then go home
	CALL  	Rotate
	CALL 	GoToWall
	JUMP 	BackAtHome

; Tag function will travel to an object X distance away, tag it, and rotate to face the wall
; Assumes DE2Bot is facing object and there is a clear, linear path to object
Tag:
	; Saves whatever is in AC
	STORE 	Temp
	; Update EncoderY (initial value)
	IN   	YPOS
	STORE 	EncoderY
	; Control Movement Variables
	LOADI 	THETA
	STORE 	DTheta
	LOAD 	FMid
	STORE 	DVel
TagIt:
	; Move robot
	CALL 	ControlMovement
	; Check distance traveled
	; Subtract initial EncoderY Pos, Cell Dist, and error margin
	IN 		YPOS
	CALL 	Abs
	SUB 	EncoderY
	SUB 	Cell
	ADDI 	-10
	JNEG 	TagIt
	; Prepare to move backwards a little
	; Update EncoderY and Control Movement
	IN 		YPOS
	CALL 	Abs
	ADDI 	-30
	STORE 	EncoderY
	LOAD 	RMid
	STORE 	DVel
MoveBack:
	; Move backwards a little
	CALL ControlMovement
	; Check distance
	IN 		YPOS
	CALL 	Abs
	SUB 	EncoderY
	JPOS 	MoveBack
	; Rotate 180 and GoHome
	LOADI 	180
	CALL 	Rotate
	CALL 	GoHome
	
;Goes to middle and searches for object to go towards
Middle:			DW 2090
GoToMiddleSearch:

	;Check if robot is at middle yet
	CheckIfMiddle:
		CALL 	UpdateMap
		LOAD 	XPOS
		ADD 	-2090
		JNEG 	NotAtMiddle
		JUMP 	AtMiddle

	;Robot Moves Forward if Not at middle yet
	NotAtMiddle:
		LOAD	FMid
		OUT		LVELCMD
		OUT		RVELCMD
		JUMP  CheckIfMiddle

;Deals with robot rotation at middle position
AtMiddle:
	LOAD	ZERO
	OUT		LVELCMD
	OUT		RVELCMD

	LOAD	MASK2
	OUT		SONAREN
	LOAD	MASK3
	OUT		SONAREN

	;Check if sensor 2 detects an Object
	CheckMidObj:
		CALL 	UpdateMap
		LOAD	DIST2
		ADDI	-915
		JNEG	TwoGot
		JUMP	Rotate10

	;Given sensor 2 found an object, Check if sensor 3 detects an Object
	TwoGot:
		LOAD	ONE
		LOAD	DIST3
		ADDI	-915
		JNEG	ThreeGot
		JUMP	Rotate10

	;Given sensor 2 and 3 found an object, handle remaining action
	ThreeGot:
		LOAD	ZERO
		OUT		LVELCMD
		OUT		RVELCMD
		;Temporarily
		JUMP	ThreeGot

	;Rotates 12 degrees if object was not in front of object based on s2 and s3
	Rotate10:
		LOAD 	ZERO
		ADDI 	12
		STORE Angle
		JUMP	Rotate
		JUMP	CheckMidObj
		
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

Ang0:		DW 90
Ang1:		DW 44
Ang2:		DW 12
Ang3:		DW -12
Ang4:		DW -44
Ang5:		DW -90
Ang6:		DW -144
Ang7:		DW 144

SensorToCheck: 	DW 0
SensorAngle:		DW 0

SensorDist: 		DW 0

SensorIndex: 		DW 0

SensorUpdate:		DW 0
; TODO: Update Array based on Sensor to Check

;**************************************************
; Helper Subroutines
;**************************************************
KillSonars:
	;stop all sensors
	LOAD	ZERO
	OUT		SONAREN
	RETURN
	
; Stops robot movement
StopMovement:
	LOAD 	ZERO
	OUT 	LVELCMD
	OUT 	RVELCMD
	RETURN

ShortBeep:
	STORE	Temp
	LOADI 	2
	OUT		BEEP
	LOADI	1
	STORE	WaitTime
	OUT		Timer

	BeepLoop:
		IN 		Timer
		SUB 	WaitTime
		JNEG	BeepLoop
		LOADI	0
		OUT		BEEP
		LOAD 	Temp
		RETURN

; Mod360 (keep angle between 0 and 359)
Mod360:
	JNEG	M360N
	ADDI 	-360
	JUMP 	Mod360

	M360N:
		ADDI 	360
		JNEG 	M360N
		RETURN

; Rotate X degrees. Store X in the var Angle
Rotate:
		STORE	Temp
		; Calculate Threshold Values
		; Calculate Lower Error Margin
		IN 		THETA
		ADD 	Angle
		SUB 	ErrMargin
		CALL 	Mod360
		STORE 	LowErr
		; Calculate High Error Margin
		IN 		THETA
		ADD 	Angle
		ADD 	ErrMargin
		CALL 	Mod360
		STORE 	HighErr
		; Check rotation direction
		LOAD 	Angle
		JNEG 	RotateCW ; else RotateCC

	; Rotate CounterClock
	RotateCC:
		LOAD 	FSlow
		OUT		RVELCMD
		LOAD 	RSlow
		OUT		LVELCMD
	; Check if Theta is correct
		IN 		THETA
		SUB 	HighErr
		JPOS	RotateCC
		IN 		THETA
		SUB  	LowErr
		JNEG	RotateCC
		JUMP 	RotateEnd
	; Rotate Clockwise
	RotateCW:
		LOAD 	RSlow
		OUT		RVELCMD
		LOAD 	FSlow
		OUT		LVELCMD
		; Check if Theta is correct
		IN 		THETA
		SUB 	HighErr
		JPOS	RotateCW
		IN 		THETA
		SUB  	LowErr
		JNEG	RotateCW
	RotateEnd:
	; Stop movement and return
		CALL 	StopMovement
		LOAD 	Temp
		RETURN

; Function tells DE2Bot to travel straight until wall is detected
; Uses sensors 2 and 3 to detect wall
GoToWall:
	; Initialize sensors
	LOAD 	MASK2
	OR 		MASK3
	OUT 	SONAREN
	; Initialize movement variables
	IN  	THETA
	STORE 	DTheta
	LOAD 	FMid
	STORE 	DVel
CheckWall:
	CALL ControlMovement
	; Check if distance is lower than threshold
	IN 		DIST2
	ADD 	WallThresh 	; 20 cm ~= 8 inches
	JPOS 	CheckWall
	IN 		DIST3
	ADD 	WallThresh 	; 20 cm ~= 8 inches
	JPOS 	CheckWall
	CALL 	StopMovement 	; stops movement

; Control code.  If called repeatedly, this code will attempt
; to control the robot to face the angle specified in DTheta
; and match the speed specified in DVel
DTheta:    DW 0
DVel:      DW 0
ControlMovement:
	; convenient way to get +/-180 angle error is
	; ((error + 180) % 360 ) - 180
	IN     THETA
	SUB    DTheta      ; actual - desired angle
	CALL   Neg         ; desired - actual angle
	ADDI   180
	CALL   Mod360
	ADDI   -180
	; A quick-and-dirty way to get a decent velocity value
	; for turning is to multiply the angular error by 4.
	SHIFT  2
	STORE  CMAErr      ; hold temporarily
	; For this basic control method, simply take the
	; desired forward velocity and add a differential
	; velocity for each wheel when turning is needed.
	LOAD   DVel
	ADD    CMAErr
	CALL   CapVel      ; ensure velocity is valid
	OUT    RVELCMD
	LOAD   CMAErr
	CALL   Neg         ; left wheel gets negative differential
	ADD    DVel
	CALL   CapVel
	OUT    LVELCMD

	RETURN

CMAErr: DW 0       ; holds angle error velocity

CapVel:
	; cap velocity values for the motors
	ADDI    -500
	JPOS    CapVelHigh
	ADDI    500
	ADDI    500
	JNEG    CapVelLow
	ADDI    -500
	RETURN
CapVelHigh:
	LOADI   500
	RETURN
CapVelLow:
	LOADI   -500
	RETURN

Abs:
	JPOS   Abs_r
Neg:
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN
	
	
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
Mult16s:
		LOADI  0
		STORE  m16sc        ; clear carry
		STORE  mres16sH     ; clear result
		LOADI  16           ; load 16 to counter
	Mult16s_loop:
		STORE  mcnt16s
		LOAD   m16sc        ; check the carry (from previous iteration)
		JZERO  Mult16s_noc  ; if no carry, move on
		LOAD   mres16sH     ; if a carry,
		ADD    m16sA        ;  add multiplicand to result H
		STORE  mres16sH
	Mult16s_noc: ; no carry
		LOAD   m16sB
		AND    One          ; check bit 0 of multiplier
		STORE  m16sc        ; save as next carry
		JZERO  Mult16s_sh   ; if no carry, move on to shift
		LOAD   mres16sH     ; if bit 0 set,
		SUB    m16sA        ;  subtract multiplicand from result H
		STORE  mres16sH
	Mult16s_sh:
		LOAD   m16sB
		SHIFT  -1           ; shift result L >>1
		AND    c7FFF        ; clear msb
		STORE  m16sB
		LOAD   mres16sH     ; load result H
		SHIFT  15           ; move lsb to msb
		OR     m16sB
		STORE  m16sB        ; result L now includes carry out from H
		LOAD   mres16sH
		SHIFT  -1
		STORE  mres16sH     ; shift result H >>1
		LOAD   mcnt16s
		ADDI   -1           ; check counter
		JPOS   Mult16s_loop ; need to iterate 16 times
		LOAD   m16sB
		STORE  mres16sL     ; multiplier and result L shared a word
		RETURN              ; Done

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second at 10Hz.
	JNEG   Wloop
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN

; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN

; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever

; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine for Cosine of an angle
; input THETA as degree
; output x_val as cosine of THETA
; using Taylor series -- will be accurate between -pi/2 and pi/2

; NEED MULT16S, lowbyte DW &HFF, DIV16S, neg
COSINE:
		LOAD	THETA
		STORE 	TCOPY
		LOAD 	THETA
		ADDI 	-180
		JNEG 	NEGSINE			; if THETA is in third or fourth quadrant, sine value will be output as negative
		ADDI	180
		ADDI 	-90
		JPOS	QUAD2
		ADDI	90
THETAQUAD2:
THETANEGSINE:
		LOAD 	THETA
		STORE 	m16sA
		STORE 	m16sB
		CALL	Mult16s
		LOAD    mres16sH
		SHIFT   8            ; move high word of result up 8 bits
		STORE   mres16sH
		LOAD    mres16sL
		SHIFT   -8           ; move low word of result down 8 bits
		AND     LowByte
		OR      mres16sH     ; combine high and low words of result
		STORE	THETAtemp2	 ; equivalent to THETA ^ 2
		STORE	THETA2
		SHIFT	-1			 ; divide by 2
		CALL 	Neg
		STORE	THETA2
		
		LOAD 	THETAtemp
		STORE 	m16sA
		STORE 	m16sB
		CALL	Mult16s
		LOAD    mres16sH
		SHIFT   8            ; move high word of result up 8 bits
		STORE   mres16sH
		LOAD    mres16sL
		SHIFT   -8           ; move low word of result down 8 bits
		AND     LowByte
		OR      mres16sH     ; combine high and low words of resultv
		STORE 	THETAtemp4	 ; equivalent to THETA ^ 4
		STORE 	THETA4
		
		LOADI 	2
		SHIFT	5			 ; immediate of 2 ^ 5 = 32
		ADDI 	-8			 ; 32 - 8 = 24
		STORE 	d16sD
		LOAD 	THETA4
		STORE 	d16sN
		CALL	Div16s
		LOAD	dres16sQ
		STORE 	THETA4
		
		LOAD 	THETAtemp4	 ; performing THETA^6
		STORE 	m16sA
		LOAD	THETAtemp2
		STORE 	m16sB
		CALL	Mult16s
		LOAD    mres16sH
		SHIFT   8            ; move high word of result up 8 bits
		STORE   mres16sH
		LOAD    mres16sL
		SHIFT   -8           ; move low word of result down 8 bits
		AND     LowByte
		OR      mres16sH     ; combine high and low words of result
		STORE	THETA6		 ; equivalent to THETA ^ 6
		
		LOADI	2
		SHIFT	10			 ; immediate of 1024
		ADDI	-304		 ; 1024 - 304 = 720
		LOAD 	d16sD
		LOAD 	THETA6
		STORE 	d16sN
		CALL	Div16s
		LOAD	dres16sQ
		Call	Neg
		STORE 	THETA6
		
		LOADI	1
		ADD		THETA2
		ADD 	THETA4
		ADD		THETA6
		STORE	x_val
		
		LOAD 	TCOPY
		ADDI	-180
		JPOS	NEGYVAL
		
		STORE	x_val
		RETURN	
		
NEGYVAL:	
		LOAD 	x_val
		CALL	Neg			; negate x_val if in quadrants III o IV
		STORE 	x_val
		RETURN
		
QUAD2:	
		LOAD 	THETA
		ADDI	-90
		J		THETAQUAD2		
		
NEGSINE:
		LOADI	360
		SUB		THETA
		J THETANEGSINE
		
		
; Subroutine for SINE function
; input: distance from the object (DISTX) <based on the sonar being used, THETA
	; sin(theta) = O/H = x_val/DISTX
; output: y_val
; NEEDED: COSINE

SINE:
		LOAD	THETA
		ADDI	-90		; finding other angle
 		CALL 	COSINE
		LOAD 	x_val
		STORE 	y_val	; by finding the cosine of the other angle, the sine of the original triangle can be found
	
		STORE 	d16sN
		LOAD  	DISTX
		STORE 	d16sN
		CALL	Div16s
		LOAD	dres16sQ
		STORE 	y_val
		RETURN
	

;***************************************************************
;* Variables
;***************************************************************
Temp:				DW 0 ; "Temp" is not a great name, but can be useful
WaitTime:			DW 0
Angle: 				DW 0 ; Used in Rotate function
LowErr: 			DW 0 ; Error margin variables
HighErr: 			DW 0 ; Used in Rotate function
ErrMargin: 			DW 4
XDir:				DW 0		; Current direction on the X access robot is moving. 1 = right, 0 = left
ObjectXDist:		DW 0 		; The x position of the next closest object
ObjectYDist:		DW 0		; The absolute value of the y position of the next closest object
AlongLongWall:		DW 0		; Boolean that signifies if robot is aligned along the longest wall
ObjectsPosTheta:	DW 0		; Boolean that signifies if the robot has to turn in a positive angle to tag objects
TagVelocity:		DW 0		; Number that signifies the speed and direction the robot has to go in to get to the next closest object along the wall
EncoderY: 			DW 0		; Stores current value of encoder in Y direction
WallThresh: 		DW -200 	; Defines distance away from wall before DE2Bot should stop moving (used in GoHome function)
Cell: 				DW 300		; Initialize cell value
CellCount:  		DW 0 		; How many values in the occupancy array
CellArrI:   		DW &H44C	; Memory location (starting index) of the cell array
XposIndex:			DW 0		; Initialize a temporary index for cell array indexing

y_val:			DW 0 
THETAtemp2:		DW 0 
THETAtemp4:		DW 0
THETA2 :		DW 0 
THETA4 :		DW 0
THETA6 :		DW 0 
TCOPY  :		DW 0
CosSum:			DW 0


;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
TwoFeet:  DW 586       ; ~2ft in 1.04mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500
MaxLong:	DW 2931	   	; 12 ft - 2ft (for home and robot) = 10ft = 3048 mm =~ 2900 increments in position
MaxShort:	DW 1740		; TEST 8ft - 2ft (for home and robot) = 6ft = 1740 mm =~ 1740 increments in position

MinBatt:  DW 140       ; 14.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9
;***************************************************************
;* Allocate space in memory for our x and y arrays
;* and our temporary array which will be used to estimate the distance by averaging a number of values
;* The x-array will inititialize at a location sufficiently far away from other instructions
;* Allows for dynamic length and known locations of words
;***************************************************************
		 

ORG     &H44C ; Start at location 1100 for the occupancy array
