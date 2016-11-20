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

WaitForUserSub:
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
	JPOS   WaitForUserSub ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue
	RETURN

	TravelDist:		DW 0
; Goes to the x position the closest object is located at
; Turns toward object and tags it
; Then returns back home, retracing its path
FindAndTagClosestObject:
		OUT		RESETPOS
		LOAD	MASK5
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
		JUMP	NewKeepCheck
		
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

; ; GoHome function will have the robot go home after tagging
; ; Assumes DE2Bot is facing a wall and there is a clear straight path to wall
GoHome:
	; Go towards wall then stop when close
	IN 		MASK2
	OR 		MASK3
	OUT 	SONAREN
	LOADI 	FMid
	STORE 	DVel
	LOADI 	HomeAng
	STORE 	DTheta
; Detect1:
; 	CALL 	ControlMovement
; 	IN 		DIST2
; 	ADDI 	-500
; 	JPOS 	Detect1
; 	IN	 	DIST3
; 	ADDI 	-500
; 	JPOS 	Detect1
; 	; Rotate
; 	CALL 	StopMovement
; 	LOADI 	-60
; 	STORE 	Angle
; 	CALL  	Rotate
; 	LOADI 	90
; 	STORE 	DTheta
; 	; Go Home till its close to wall
; Detect2:
; 	CALL	ControlMovement
; 	IN 		DIST2
; 	ADDI 	-500
; 	JPOS 	Detect2
; 	IN	 	DIST3
; 	ADDI 	-500
; 	JPOS 	Detect2
; 	CALL 	StopMovement
; 	JUMP 	BackAtHome
	
; GoHome:
; 	; Go towards wall then when XPOS is close enough
; 	LOADI 	FMid
; 	STORE 	DVel
; 	LOADI 	180
; 	STORE 	DTheta
Detect1:
	CALL 	ControlMovement
	IN 		XPos
	ADDI 	-300 ; distance away from x origin
	JPOS 	Detect1
	; Rotate
	;CALL 	StopMovement
	;LOADI 	-55
	;STORE 	Angle
	;CALL  	Rotate
	LOAD	HomeAng
	STORE 	DTheta
	; Go Home till its close to wall
Detect2:
	CALL	ControlMovement
	IN 		YPos
	CALL 	abs
	ADDI 	-200
	JPOS 	Detect2
	;IN 		DIST2
	;ADDI 	-500
	;JPOS 	Detect2
	;IN	 	DIST3
	;ADDI 	-500
	;JPOS 	Detect2
	;CALL 	StopMovement
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
	ADDI 	290 ; 310 + 30
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
	;Move backwards a little
	CALL 	ControlMovement
	; Check distance
	IN 		YPOS
	CALL 	Abs
	SUB 	EncoderY
	JPOS 	MoveBack
	CALL 	StopMovement
	; Rotate 180 and GoHome
	IN 		XPos
	STORE 	ATanX
	IN 		YPos
	STORE 	ATanY
	CALL 	ATan2
	ADDI 	180 ;mess with change angle
	CALL	mod360
	STORE 	Dtheta
	STORE	HomeAng
	LOAD	ZERO
	STORE	DVEL
	CALL 	GoHome
	
	
MoveHome:
	In		Theta
	Sub		Dtheta
	Call 	ABS
	ADDI	-10
	JNEG 	GoHome
	Jump 	MoveHome
; MoveBack:
; 	; Move backwards a little
; 	CALL 	ControlMovement
; 	; Check distance
; 	IN 		YPOS
; 	CALL 	Abs
; 	SUB 	EncoderY
; 	JPOS 	MoveBack
; 	; Rotate to face shorter wall
; 	LOADI 	-60
; 	STORE 	Angle
; 	CALL 	Rotate
; 	CALL 	GoHome
; 	RETURN


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
		
;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW 0                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	CALL   Neg          ; Negatge the number
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	CALL   Neg          ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOADI  9            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

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


;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
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
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high
		
;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	CALL   Neg
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

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
ErrMargin: 			DW 5
EncoderY: 			DW 0		; Stores current value of encoder in Y direction
TagAng: 			DW -90		; Tells robot travel ang when tagging
HomeAng:			DW 0		; Tells robot what angle to go home at

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