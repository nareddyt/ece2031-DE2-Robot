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
		; Reset odometer in case wheels move after programming
		OUT 	RESETPOS
	
		; Repeat this portion forever, waiting for user input each time we return home
		LOAD	ZERO
		JUMP	MainLoopForever
		
	MainLoopForever:	
		CALL	FindAndTagClosestObject
		
		;CALL	WaitForUserSub
		
		; Reset odometer in case wheels move after programming
		OUT 	RESETPOS
		
		; Loop again
		;JUMP	MainLoopForever

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

; ------------------- ;
; END OF CONTROL FLOW ;
; ------------------- ;

;**************************************************
; Important Subroutines
;**************************************************

	TravelDist:		DW 0
; Goes to the x position the closest object is located at
; Turns toward object and tags it
; Then returns back home, retracing its path
FindAndTagClosestObject:
		OUT		RESETPOS
		LOAD	MASK5
		OR		MASK2
		OR		MASK3
		OUT		SONAREN

	NewKeepCheck:		
		IN		DIST5
		STORE	TravelDist
		OUT		SSEG1
		SUB		MaxShort
		JNEG	NewFound
		
		IN		XPOS
		SUB		MaxLong
		JZERO	TurnAroundGoHome
		JPOS	TurnAroundGoHome
		
		LOADI 	0
		STORE 	DTheta
		LOAD 	FFast
		STORE 	DVel
		; Move robot
		CALL 	ControlMovement
		
		; Check if we are about to run into anything
		IN 		DIST3
		ADDI 	-310
		JPOS 	NewKeepCheck
		
		; We detected an object in the robot's movement path!
		; Move 310 mm forward and tag
		; Update EncoderX (initial value)
		IN   	XPOS
		ADDI 	325
		STORE 	EncoderX
		
	HitDetectedAlongPath:
		; Travel a bit more
		; Move robot
		LOADI 	0
		STORE 	DTheta
		LOAD 	FFast
		STORE 	DVel
		CALL 	ControlMovement
		
		; Check distance traveled
		IN 		XPOS
		SUB 	EncoderX
		JNEG 	HitDetectedAlongPath
	
		; We just hit the object!
		; Prepare to move backwards a little
		; Update EncoderY and Control Movement
		IN 		XPOS
		ADDI 	-30
		STORE 	EncoderX
		LOADI 	0
		STORE 	DTheta
		LOAD 	FFast
		STORE 	DVel
	MoveBackABit:
		; Move backwards a little
		CALL 	ControlMovement
		
		; Check distance
		IN 		XPOS
		SUB 	EncoderX
		JPOS 	MoveBackABit
		
		; Now stop, turn around, and go back home
		CALL	StopMovement
		JUMP	TurnAroundGoHome
		
	TurnAroundGoHome:
		CALL	StopMovement
		
		LOADI	-160
		STORE	Angle
		CALL	Rotate
		
	GoingHome:
		LOADI 	180
		STORE 	DTheta
		LOAD 	FFast
		STORE 	DVel
		; Move robot
		CALL 	ControlMovement
		JUMP	GoingHome
			
	NewFound:
		; Stop the robot
		CALL	StopMovement
		
		; Call Randy's tag subroutine
		CALL 	Tag

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
	IN 		MASK2
	OR 		MASK3
	OUT 	SONAREN
	LOADI 	FMid
	STORE 	DVel
	LOADI 	90
	STORE 	DTheta
Detect1:
	CALL 	ControlMovement
	IN 		DIST2
	ADDI 	-500
	JPOS 	Detect1
	IN	 	DIST3
	ADDI 	-500
	JPOS 	Detect1
	; Rotate
	CALL 	StopMovement
	LOADI 	95
	STORE 	Angle
	CALL  	Rotate
	LOADI 	180
	STORE 	DTheta
	; Go Home
Detect2:
	CALL	ControlMovement
	IN 		DIST2
	ADDI 	-500
	JPOS 	Detect2
	IN	 	DIST3
	ADDI 	-500
	JPOS 	Detect2
	CALL 	StopMovement
	JUMP 	BackAtHome

; Tag function will travel to an object X distance away, tag it, and rotate to face the wall
; Assumes DE2Bot is facing object and there is a clear, linear path to object
Tag:
	; Initialize Sonar
	IN		MASK2
	OR 		MASK3
	OUT 	SONAREN
	; Control Movement Variables
	LOADI	-90
	STORE 	DTheta
	LOAD 	FMid
	STORE 	DVel
TagIt:
	; Read sensor 2 and 3 till 310 mm
	CALL 	ControlMovement
	CALL 	UpdateTag
	IN 		DIST2
	ADDI 	-310
	JNEG 	TagIt2
	IN 		DIST3
	ADDI 	-310
	JNEG 	TagIt2
	JUMP 	TagIt
TagIt2:
	; Move 310 mm forward and tag
	; Update EncoderY (initial value)
	IN   	YPOS
	CALL 	Abs
	ADDI 	370 ; 310 + 60
	STORE 	EncoderY
TapTag:
	; Travel a bit more
	; Move robot
	CALL 	ControlMovement
	; Check distance traveled
	IN 		YPOS
	CALL 	Abs
	SUB 	EncoderY
	JNEG 	TapTag
	; Prepare to move backwards a little
	; Update EncoderY and Control Movement
	IN 		YPOS
	CALL 	Abs
	ADDI 	-30
	STORE 	EncoderY
	LOAD 	RFast
	STORE 	DVel
MoveBack:
	; Move backwards a little
	CALL 	ControlMovement
	; Check distance
	IN 		YPOS
	CALL 	Abs
	SUB 	EncoderY
	JPOS 	MoveBack
	; Rotate 180 and GoHome
	LOADI 	180
	STORE 	Angle
	CALL 	Rotate
	CALL 	GoHome
	RETURN
; This subroutine updates the angle the robot should move
UpdateTag:
	RETURN


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
; TODO removed cause errors :(

;***************************************************************
;* Variables
;***************************************************************
Temp:				DW 0 ; "Temp" is not a great name, but can be useful
WaitTime:			DW 0
Angle: 				DW 0 ; Used in Rotate function
LowErr: 			DW 0 ; Error margin variables
HighErr: 			DW 0 ; Used in Rotate function
ErrMargin: 			DW 4
EncoderY: 			DW 0		; Stores current value of encoder in Y direction
EncoderX:			DW 0		; Stores current value of encoder in X direction
TagAng: 			DW -90		; Tells robot travel ang when tagging

y_val:			DW 0
THETAtemp2:		DW 0
THETAtemp4:		DW 0
THETA2:			DW 0
THETA4:			DW 0
THETA6:			DW 0
TCOPY:			DW 0
CosSum:			DW 0


;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegFive:	DW -5
NegOne:   	DW -1
Zero:     	DW 0
One:      	DW 1
Two:      	DW 2
Three:    	DW 3
Four:     	DW 4
Five:     	DW 5
Six:      	DW 6
Seven:    	DW 7
Eight:    	DW 8
Nine:     	DW 9
Ten:      	DW 10

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
FSlow:    DW 140       ; 100 is about the lowest velocity value that will move
RSlow:    DW -140
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

ORG     &H64c ; Start at location 1100 for the occupancy array
DW		0	; Array start	