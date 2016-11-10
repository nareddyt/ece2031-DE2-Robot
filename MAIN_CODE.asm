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
	OUT 	RESETPOS	; reset odometer in case wheels move after programming
	; TODO

; Main loop to search begins here
MainLoop:
	CALL	InitializeVars
	CALL	InitialSearch

	; TODO we need some way to keep track of the number of objects left
	; while (numObjects > 0) { call FindAndTagClosestObject }
	CALL	FindAndTagClosestObject

;**************************************************
; Important Subroutines
;**************************************************

InitializeVars:
	; SW0 = AlongLongWall --> Sets var to 1 if the robot is travelling along the long wall
	IN		SWITCHES
	AND		MASK0
	STORE 	AlongLongWall

	; SW1 = ObjectsPosTheta --> Sets var to 1 if the robot must turn in the positive direction to tag objects
	IN		SWITCHES
	AND		MASK1
	STORE	ObjectsPosTheta

	; Reset odometer in case wheels move after initialization
	OUT 	RESETPOS

	; Return!
	RETURN

; Initial search. Follow walls, updating the map based on objects that are perpendicular.
InitialSearch:
		; TODO change code based on 2 vars

		; Enable sonar sensors 2 and 3 to make sure we don't run into anything
		; TODO do this with interrupts instead of just checking every loop cycle
		LOAD	MASK2
		ADD		MASK3
		OUT 	SONAREN

	; Go forward until we are at the end of the edge
	KeepGoingForward:

		; Update the map with the current sensor readings
		; CALL Jeff's code here!
		CALL 	UpdateMap

		; Check the robot has gone too far in the x direction
		; Note that this distance depends on which wall we are following, stored in AlongLongWall
		IN		AlongLongWall
		JZERO	LoadShortDistance

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
		; CHECKME maybe we should aggregate this data as well?

		; Keep going forward as we have not hit the max limit for the wall
		; FIXME tweak the speeds
		LOAD	FMID
		OUT		LVELCMD
		OUT		RVELCMD

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
	;Traverse an axis, but store the SMALLEST value read by sonar sensor and associate an XPOS with that location
 	LOAD 	AlongLongWall
	JPOS 	LGO ; If no switches active, robot setup values for long axis traverse
	JZERO  	SGO ; If SW0 active, robot setup values for short axis traverse

	LGO:
	 	LOAD	MASK0
	 	OUT 	SONAREN
	 	IN 		DIST0 ;Turn on and read value from sensor 0
	 	SUB 	Cell ;subtract current value in cell
	 	JNEG	CellIn ; If value read in less than the value already in cell, store it in cell
	 	RETURN

	SGO:
		LOAD	MASK5
		OUT 	SONAREN
		IN 		DIST5
		SUB 	Cell ;subtract current value in cell
	 	JNEG	CellIn ; If value read in less than the value already in cell, store it in cell
		RETURN

	CellIn:
		; Add back the value of cell and store the dist measurement into cell
		ADD Cell
		STORE Cell
		IN XPOS
		STORE ObjLoc
		RETURN


; Goes to the x position the closest object is located at
; Turns toward object and tags it
; Then returns back home, retracing its path
FindAndTagClosestObject:

		; Call method to get information about the closest object
		CALL	FindClosestObject
		; Now, the x pos of the closest object is stored in ObjectXDist, the y pos is in ObjectYDist
		; TODO bounds check on the closest object (just in case?!?)

	; Go toward the object until we hit the y distance
	MoveTowardObject:

		; Do the bounds check
		LOAD 	XPOS
		SUB 	ObjectXDist
		JNEG	GoUp
		JZERO 	AtObjectX
		JPOS	GoDown

	GoUp:
		LOAD 	ONE
		STORE 	XDir
		JUMP 	MoveLoop

	GoDown:
		LOAD 	ZERO
		STORE 	XDir

	MoveLoop:
		; Update the map with the current sensor readings
		CALL 	UpdateMap
		
		; Do the bounds check for real
		LOAD XDir
		JZERO CheckLess
		JPOS CheckGreat

	CheckGreat:
		LOAD	XPOS
		SUB		ObjectXDist
		JZERO 	AtObjectX
		JPOS	AtObjectX
		JUMP	KeepGoing

	CheckLess:
		LOAD	XPOS
		SUB		ObjectXDist
		JZERO 	AtObjectX
		JNEG	AtObjectX
		JUMP	KeepGoing

	KeepGoing:
		LOAD	FMid
		OUT		LVELCMD
		OUT		RVELCMD
		JUMP	MoveLoop

	AtObjectX:
		; TODO turn for Randy's tagging
		; TODO call Randy's tag method
		; TODO return to home

		; Stop the robot
		LOAD	ZERO
		OUT		LVELCMD
		OUT		RVELCMD

		; Return to main
		RETURN


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


; Finds the closest object (relative to the wall) based on the map
FindClosestObject:
	; TODO CHECKME
	LOAD	CELL
	STORE	ObjectXDist
	STORE	ObjectYDist
	RETURN

; We are back at home now
BackAtHome:
	; TODO wait for user input to start again
	RETURN

; Tag object
Tag:
	STORE 	Temp
TagIt:
	LOAD 	FMid
	OUT 	LVELCMD
	OUT 	RVELCMD
	LOAD 	MASK2
	OR 		MASK3
	OUT 	SONAREN
	IN 		DIST2
	ADDI	-310
	JNEG 	TagHit

	IN 		DIST3
	ADDI 	-310
	JNEG 	TagHit

	JUMP 	TagIt

; Tag/Hit object
TagHit:
	JUMP 	Die
    LOAD 	FSlow
	LOAD 	Temp ; TODO
	JUMP 	TagHit
	RETURN

; Test Object Tagging
TestTag:
	LOAD 	MASK4
	OR 		MASK1
	OUT 	SONAREN

	IN 		DIST4
	ADDI	-610 ;2 feet
	JNEG 	Tag1
	IN 		DIST1
	ADDI	-610 ;2 feet
	JNEG 	Tag2

	LOAD 	FMid
	OUT 	LVELCMD
	OUT 	RVELCMD
	JUMP 	TestTag
Tag1:
	LOADI 	-40
	STORE 	Angle
	CALL 	Rotate
	CALL 	Tag
	CALL 	Die
Tag2:
	LOADI 	40
	STORE 	Angle
	CALL 	Rotate
	CALL 	Tag
	CALL 	Die


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

;**************************************************
; Helper Subroutines
;**************************************************

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
		IN 		THETA
		ADD 	Angle
		SUB 	ErrMargin
		CALL 	Mod360
		STORE 	LowErr

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
		LOAD 	ZERO
		OUT 	LVELCMD
		OUT 	RVELCMD
		LOAD 	Temp
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

;***************************************************************
;* Variables
;***************************************************************
Temp:				DW 0 ; "Temp" is not a great name, but can be useful
WaitTime:			DW 0
Angle: 				DW 0 ; Used in Rotate function
LowErr: 			DW 0 ; Error margin variables
HighErr: 			DW 0 ; Used in Rotate function
ErrMargin: 			DW 4
XDir:						DW 0	; Direction on the X access robot is moving
ObjectXDist:		DW 0 	; The x position of the next closest object
ObjectYDist:		DW 0	; The absolute value of the y position of the next closest object
AlongLongWall:		DW 0	; Boolean that signifies if robot is aligned along the longest wall
ObjectsPosTheta:	DW 0	; Boolean that signifies if the robot has to turn in a positive angle to tag objects
TagVelocity:		DW 0	; Number that signifies the speed and direction the robot has to go in to get to the next closest object along the wall
Cell: 				DW &H7FFF	; Initialize cell value
ObjLoc:				DW 0	 	; Stores the location of the object to be tagged


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
MaxShort:	DW 1740		; 8ft - 2ft (for home and robot) = 6ft = 1740 mm =~ 1740 increments in position

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
