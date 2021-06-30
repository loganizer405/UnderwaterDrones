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
