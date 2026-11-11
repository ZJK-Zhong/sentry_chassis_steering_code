.syntax unified
.cpu cortex-m4
.thumb
.section .vectors,"a",%progbits
.word 0x20040000
.word TestReset
.section .text.TestReset,"ax",%progbits
.global TestReset
.thumb_func
TestReset:
    bl test_main
    b .
