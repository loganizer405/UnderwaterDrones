# Buttons #

## Bit Shifting ##

See motorTest.py:
buttons = 1 + (1 << 3) + (1 << 7)

This is an operation called bit shifting. This is a simple method of changing binary numbers. It is what it sounds like: the bits are shifted, left or right. << is left bit shifting and >> is right bit shifting. The first number is the operand which we modify, and the second number is how many bits to shift. For example,
15 << 2
# Solution:
15 in binary (8-bit representation) is 00001111
This is a left shift of two bits. Move all bits two positions to the left:
00111100
Converted to decimal is 60

15 << 2 = 60

# How to use this for buttons #
The value set to 'buttons' and passed to master.mav.manual_control_send(..., buttons) in binary represents which buttons are on and off.
The bit position represents the button. 1 in that place corresponds to on, 0 is off.

Each of the buttons on the joystick are assigned a number. We do not have a list of these but the mapping is apparently available in QGroundControl.

From motorTest.py:
buttons = 1 + (1 << 3) + (1 << 7)
Solution to the bit shift:
137 = b10001001

This means buttons 0, 3, and 7 are on, while the others are off.

So, in order to turn buttons on, simply use 1 and the bit shift. To turn multiple buttons on, use addition. For example, to turn on buttons 6 and 7, use
buttons = (1 << 6) + (1 << 7)


# Button Mapping #
|Button	| Function		| Binary	| Decimal |
| ----- | --------------------- | ------------- | ------- |
| 0	| 			| 		| 	  |
|1	| Manual mode		|000000000000010|	2 |
|2	| Depth hold mode	|000000000000100|	4 |
|3	| Stabilize mode	|000000000001000|	8 |
|4	| Disarm	   	|000000000010000|	16|
|5	| Shift			|000000000100000|	32|
|6	| Arm			|000000001000000|	64|
|7	| Mount center		|000000010000000|	128|
|8	| Input hold set	|000000100000000|	256|
|9	| Tilt camera down	|000001000000000|	512|
|10	| Tilt camera up	|000010000000000|	1024|
|11	| Increase gain		|000100000000000|	2048|
|12	| Decrease gain		|001000000000000|	4196|
|13	| Brighten lights	|010000000000000|	8192|
|14 	| Dim lights		|100000000000000|	16384|

# Joystick [Motor] Control #

Manual control syntax:

master.mav.manual_control_send(  
    master.target_system,  
    x,  
    y,  
    z,   
    r,  
    buttons)  

x is forward and backward movement. 1000 is full forward, -1000 is full backward.  
y is left and right movement. 	    1000 is full right, -1000 is full left.  
z corresponds to separate slider movement. 1000 is maximum positive thrust, -1000 is maxiumum negative thrust  
r is rotation (yaw).	  	 1000 is full counterclockwise, -1000 is full clockwise.  

Each of these must have a value from -1000 to 1000.  
Buttons can be set to anything shown above, or, if only control of the motors is desired, simply use 0.

To control motors individually, use RC_CONTROL_RAW as specified in the MAVLink docs.
