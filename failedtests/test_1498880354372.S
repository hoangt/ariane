// random assembly code generated by RISC-V torture test generator
// nseqs = 200
// memsize = 1024

#include "riscv_test.h"

RVTEST_RV64U
RVTEST_CODE_BEGIN

	j test_start

crash_backward:
	RVTEST_FAIL

test_start:

xreg_init:
	la x31, xreg_init_data
	ld x0, 0(x31)
	ld x1, 8(x31)
	ld x2, 16(x31)
	ld x3, 24(x31)
	ld x4, 32(x31)
	ld x5, 40(x31)
	ld x6, 48(x31)
	ld x7, 56(x31)
	ld x8, 64(x31)
	ld x9, 72(x31)
	ld x10, 80(x31)
	ld x11, 88(x31)
	ld x12, 96(x31)
	ld x13, 104(x31)
	ld x14, 112(x31)
	ld x15, 120(x31)
	ld x16, 128(x31)
	ld x17, 136(x31)
	ld x18, 144(x31)
	ld x19, 152(x31)
	ld x20, 160(x31)
	ld x21, 168(x31)
	ld x22, 176(x31)
	ld x23, 184(x31)
	ld x24, 192(x31)
	ld x25, 200(x31)
	ld x26, 208(x31)
	ld x27, 216(x31)
	ld x28, 224(x31)
	ld x29, 232(x31)
	ld x30, 240(x31)
	ld x31, 248(x31)

	j pseg_0

pseg_0:
	addi x24, x0, -441
	blt x22, x22, crash_backward
	la x14, test_memory+1548
	or x27, x7, x6
	xor x18, x24, x2
	sub x21, x18, x18
	addi x15, x0, -1015
	addi x26, x0, -792
	xor x30, x25, x8
	la x12, test_memory-1349
	addi x20, x0, 1
	sll x20, x20, 63
	addi x23, x0, 1
	sra x25, x26, x26
	addi x8, x0, -1535
	addi x9, x0, -1298
	sraiw x19, x23, 19
	addi x3, x0, 1
	sll x3, x3, 63
	sll x23, x23, 63
	lhu x4, 1355(x12)
	addi x1, x24, -774
	addi x7, x0, -1
	sllw x26, x8, x8
	bne x0, x0, crash_forward
	sra x22, x15, x9
	xori x25, x14, 1318
	add x13, x8, x8
	xor x7, x7, x3
	and x16, x20, x7
	slli x29, x28, 7
	addi x21, x0, -42
	addi x2, x0, -1
	addi x26, x0, 1
	sra x27, x9, x0
	lh x6, -722(x14)
	xor x2, x2, x23
	addi x10, x0, -1
	sll x26, x26, 63
	or x5, x13, x3
	addi x9, x0, 1697
	addi x18, x0, -1754
	xor x10, x10, x20
	and x28, x4, x2
	and x17, x3, x10
	or x11, x28, x23
	addi x12, x0, 1783
	blt x16, x5, crash_forward
	addi x15, x0, -1
	la x1, test_memory+2168
	addi x5, x0, -786
	sltu x25, x9, x28
	lw x13, -1828(x1)
	srlw x30, x21, x18
	addi x16, x0, 1139
	addi x19, x0, 753
	addi x27, x0, 0
	sub x14, x10, x7
	subw x29, x9, x9
	or x31, x13, x20
	slt x4, x12, x19
	sllw x7, x5, x16
	addi x6, x0, 0
	slt x5, x6, x1
	addi x19, x0, -202
	addi x7, x0, -623
	and x24, x4, x14
	la x23, test_memory+2884
	addi x18, x0, 0
	sra x5, x2, x2
	addi x13, x0, 0
	la x17, test_memory+772
	addi x16, x4, 0
	xor x15, x15, x26
	and x8, x22, x15
	slt x3, x11, x12
	or x22, x8, x26
	sraw x12, x7, x7
	addi x31, x0, 0
	addi x10, x0, 940
	addi x1, x16, 0
	addi x29, x0, 100
	bltu x16, x1, crash_forward
	la x28, test_memory+362
	la x5, test_memory+1494
	addi x14, x0, 539
	addi x2, x10, 1554
	slt x30, x19, x19
	srliw x26, x20, 9
	or x18, x22, x28
	addi x7, x0, 1569
	addi x8, x0, -1120
	sll x4, x19, x24
	sh x12, -1106(x5)
	addi x22, x0, 1271
	addi x6, x0, -1232
	add x13, x1, x1
	subw x2, x0, x0
	addi x25, x0, 628
	addi x10, x8, -1580
	ori x13, x5, -2000
	bgeu x10, x8, crash_forward
	addi x13, x0, 439
	addw x4, x4, x9
	xor x21, x29, x29
	lwu x24, -384(x17)
	addi x26, x0, 1903
	addi x18, x13, 649
	sub x15, x6, x6
	slt x16, x22, x22
	bltu x18, x13, crash_forward
	xor x9, x14, x25
	addi x19, x0, 1509
	addi x3, x0, -360
	addi x10, x0, -1325
	sltu x2, x26, x26
	addi x8, x10, -756
	and x12, x21, x2
	addi x4, x0, 186
	sb x9, 557(x28)
	addi x20, x0, 0
	addi x14, x0, 1210
	la x18, test_memory+906
	sll x27, x19, x3
	xor x9, x14, x14
	addi x16, x0, -474
	blt x3, x3, crash_backward
	lb x11, -1969(x23)
	ld x22, -274(x18)
	addi x27, x0, -821
	la x24, test_memory+1776
	addi x17, x16, -1119
	addw x2, x11, x11
	srli x6, x31, 22
	addi x8, x0, -1236
	addi x5, x0, -1905
	la x25, test_memory-641
	addi x1, x0, 501
	addi x13, x5, -1118
	bge x17, x16, crash_backward
	sb x6, 1591(x25)
	addi x19, x0, 1
	sll x19, x19, 63
	subw x3, x18, x0
	srl x15, x19, x15
	sltiu x12, x20, 885
	slt x28, x27, x8
	blt x5, x13, crash_backward
	sltu x29, x14, x17
	lhu x26, -1716(x24)
	sub x21, x4, x4
	la x10, test_memory+682
	slt x30, x7, x1
	sb x26, -40(x10)
	addi x9, x0, -1
	xor x9, x9, x19
	la x7, test_memory+1982
	and x18, x31, x9
	sltu x27, x20, x16
	addi x28, x0, 1
	addi x1, x3, 0
	la x4, test_memory-640
	sraw x31, x19, x19
	lui x5, 187465
	addi x15, x0, 1934
	srl x11, x14, x17
	srl x23, x15, x15
	addi x8, x1, 0
	addi x10, x0, -73
	srlw x3, x10, x10
	addi x23, x0, 1
	or x14, x30, x19
	addi x29, x0, 0
	la x12, test_memory-1333
	lbu x2, -1285(x7)
	lh x25, 1172(x4)
	addi x22, x0, 1373
	sh x15, 1861(x12)
	addi x20, x0, 0
	sll x28, x28, 63
	sll x23, x23, 63
	addi x31, x0, -1
	addi x17, x0, 1325
	addi x6, x0, -366
	srl x26, x6, x6
	xor x31, x31, x23
	addi x2, x0, -867
	addi x24, x0, 1
	addi x26, x29, 0
	addi x15, x2, -1208
	addi x20, x26, 0
	slti x6, x19, -537
	beq x2, x15, crash_forward
	addi x13, x0, -1
	sub x21, x22, x17
	srlw x5, x30, x30
	bltu x26, x20, crash_forward
	addi x8, x0, 374
	xor x13, x13, x28
	and x30, x2, x13
	sll x3, x8, x8
	addw x11, x30, x30
	sll x24, x24, 63
	and x14, x17, x17
	or x16, x30, x28
	la x20, test_memory+940
	sb x27, -753(x20)
	and x27, x1, x31
	addi x18, x0, 1373
	blt x24, x24, crash_forward
	addi x4, x18, 915
	bltu x25, x25, crash_forward
	la x15, test_memory-1197
	addi x7, x0, -1
	xor x7, x7, x24
	addi x5, x0, -1960
	sb x18, 2029(x15)
	addi x6, x0, 1
	and x29, x27, x7
	addi x26, x5, -1919
	or x10, x30, x23
	sltu x22, x20, x20
	addi x9, x0, 735
	addi x17, x0, 1
	addi x8, x0, -388
	or x12, x17, x24
	sll x17, x17, 63
	sll x6, x6, 63
	la x15, test_memory-1613
	sltu x10, x20, x7
	sllw x28, x0, x25
	addi x1, x0, -1
	addi x20, x0, 983
	addi x31, x0, -594
	slliw x26, x13, 11
	addi x4, x0, 1582
	sra x5, x20, x4
	xor x1, x1, x6
	lwu x22, 1793(x15)
	la x16, test_memory+423
	sllw x2, x9, x8
	addi x27, x0, -1211
	addi x11, x0, -1
	and x19, x20, x1
	la x30, test_memory-1140
	slt x23, x31, x27
	lhu x14, 27(x16)
	or x21, x24, x6
	addi x24, x15, 0
	sll x22, x3, x1
	addi x9, x0, -261
	bltu x0, x0, crash_backward
	addw x15, x29, x29
	addi x19, x0, -1403
	addi x12, x24, 0
	addi x14, x0, -1979
	addi x18, x0, 1821
	srl x15, x7, x7
	sltu x28, x9, x9
	addi x29, x0, 1442
	la x16, test_memory+379
	lh x2, -367(x16)
	addi x6, x0, 1892
	addi x4, x0, -640
	slli x27, x12, 31
	addi x20, x6, 642
	lh x13, 1658(x30)
	addi x1, x0, -1456
	la x16, test_memory+494
	addi x2, x22, 1049
	addi x7, x0, -449
	subw x21, x19, x18
	sll x10, x4, x7
	xori x9, x16, 1035
	addi x5, x29, 0
	slti x28, x0, -1164
	srl x15, x31, x15
	xori x31, x28, 67
	sh x29, -220(x16)
	addi x9, x0, -931
	addi x21, x0, 237
	srl x19, x9, x19
	la x2, test_memory-1139
	add x22, x29, x1
	addi x28, x0, -2032
	srl x6, x15, x15
	addi x23, x5, 0
	srl x24, x9, x21
	la x30, test_memory-570
	subw x5, x11, x20
	addi x27, x0, -1649
	lbu x20, 1545(x30)
	addi x18, x0, -1125
	addi x7, x0, -988
	or x4, x18, x18
	addi x26, x0, 1915
	la x31, test_memory+1869
	addi x24, x0, 110
	sll x13, x28, x27
	subw x8, x14, x26
	slt x16, x22, x22
	xor x11, x11, x17
	lwu x10, 1379(x2)
	addi x23, x0, 1214
	lwu x1, -1729(x31)
	sraw x5, x28, x28
	addi x31, x0, 1
	sll x22, x23, x23
	addi x28, x0, 1849
	xor x12, x7, x7
	sll x31, x31, 63
	addi x19, x0, -667
	addi x6, x19, -952
	and x25, x6, x11
	addi x9, x0, 1753
	addi x8, x0, -247
	slti x15, x24, 844
	addiw x2, x29, -505
	bne x0, x0, crash_backward
	addi x27, x0, -1
	blt x19, x6, crash_forward
	addi x10, x0, 1
	addi x14, x0, 1
	la x4, test_memory+1223
	sll x10, x10, 63
	addi x22, x0, -1
	sll x14, x14, 63
	xor x27, x27, x31
	sraw x7, x27, x20
	or x21, x24, x9
	lbu x16, -289(x4)
	addw x30, x28, x8
	and x13, x9, x27
	or x3, x25, x17
	addi x26, x0, -1
	sltu x29, x15, x3
	xor x22, x22, x10
	srli x5, x1, 17
	xor x26, x26, x14
	bgeu x25, x3, crash_forward
	or x1, x13, x31
	and x20, x28, x26
	or x18, x20, x14
	and x15, x24, x22
	blt x20, x18, crash_forward
	or x12, x15, x10
	addi x13, x0, -956
	srl x1, x12, x12
	addi x31, x0, 1998
	sll x29, x13, x31
	addi x25, x0, 0
	la x5, test_memory+1974
	sub x11, x8, x29
	addi x7, x0, 1083
	addi x3, x0, 1
	addi x13, x0, 0
	addi x12, x0, 1
	sll x12, x12, 63
	la x18, test_memory+1514
	sd x16, -1346(x18)
	or x8, x26, x26
	sraw x16, x7, x7
	addi x10, x0, 547
	lwu x15, -1802(x5)
	add x19, x22, x22
	srl x17, x19, x19
	addi x30, x0, -1
	xor x30, x30, x12
	addi x8, x0, 1139
	la x20, test_memory+1047
	sll x3, x3, 63
	addi x27, x0, -757
	addi x29, x0, -1
	addi x7, x0, 1
	bltu x25, x13, crash_forward
	addi x21, x0, -1493
	addi x23, x10, 1036
	addi x14, x0, -738
	bne x0, x0, crash_forward
	la x6, test_memory+2381
	and x2, x27, x14
	and x11, x24, x30
	addi x16, x0, 0
	xor x29, x29, x3
	ld x26, -191(x20)
	addi x9, x0, 1927
	addi x15, x0, 0
	blt x23, x10, crash_forward
	xor x19, x8, x8
	srlw x4, x21, x9
	addi x25, x0, -1105
	and x28, x26, x29
	or x22, x23, x12
	bltu x22, x11, crash_forward
	bltu x16, x15, crash_forward
	addi x1, x0, 1126
	and x13, x25, x1
	sll x7, x7, 63
	and x11, x8, x9
	addw x21, x27, x27
	and x8, x23, x4
	xor x19, x2, x2
	sub x2, x23, x3
	addi x13, x0, 1905
	addi x5, x0, -1
	lw x24, -1777(x6)
	xor x23, x22, x22
	addi x12, x0, 1
	addi x25, x0, -1455
	xor x5, x5, x7
	and x17, x27, x5
	or x31, x28, x3
	sub x20, x25, x25
	or x18, x17, x7
	addi x26, x0, -854
	beq x28, x31, crash_backward
	sll x12, x12, 63
	bltu x18, x17, crash_backward
	srl x15, x13, x26
	addi x22, x0, -1
	addi x30, x0, 304
	addi x16, x0, 1624
	xor x22, x22, x12
	slt x10, x30, x16
	and x9, x27, x22
	or x14, x9, x12
	j reg_dump

reg_dump:
	la x1, loop_count
	lw x2, 0(x1)
	addi x3, x2, -1
	sw x3, 0(x1)
	bnez x2, pseg_0
	la x1, xreg_output_data
	sd x0, 0(x1)
	sd x2, 16(x1)
	sd x3, 24(x1)
	sd x4, 32(x1)
	sd x5, 40(x1)
	sd x7, 56(x1)
	sd x8, 64(x1)
	sd x9, 72(x1)
	sd x10, 80(x1)
	sd x11, 88(x1)
	sd x12, 96(x1)
	sd x13, 104(x1)
	sd x14, 112(x1)
	sd x15, 120(x1)
	sd x16, 128(x1)
	sd x17, 136(x1)
	sd x18, 144(x1)
	sd x19, 152(x1)
	sd x20, 160(x1)
	sd x21, 168(x1)
	sd x22, 176(x1)
	sd x23, 184(x1)
	sd x24, 192(x1)
	sd x25, 200(x1)
	sd x26, 208(x1)
	sd x27, 216(x1)
	sd x28, 224(x1)
	sd x29, 232(x1)
	sd x30, 240(x1)
	sd x31, 248(x1)

	j test_end

crash_forward:
	RVTEST_FAIL

test_end:
	RVTEST_PASS

RVTEST_CODE_END


	.data

hidden_data:
	.align 8
xreg_init_data:
reg_x0_init:	.dword 0xff92ffc2ca0bbe7b
reg_x1_init:	.dword 0x166649dc02ff3f4f
reg_x2_init:	.dword 0xffffffffffffffff
reg_x3_init:	.dword 0xe0ae5d9fdd4809b8
reg_x4_init:	.dword 0x11309b6128ec1610
reg_x5_init:	.dword 0x0000000000000000
reg_x6_init:	.dword 0xeb59007640bc57dc
reg_x7_init:	.dword 0xedb999fdec848259
reg_x8_init:	.dword 0x81e0012642fd86f9
reg_x9_init:	.dword 0x0000000000000000
reg_x10_init:	.dword 0x1b803d493f09b644
reg_x11_init:	.dword 0x0000000000000005
reg_x12_init:	.dword 0x0000000000000000
reg_x13_init:	.dword 0x118963d6f1549a11
reg_x14_init:	.dword 0xb3ab4dc53714316c
reg_x15_init:	.dword 0x0000000000000000
reg_x16_init:	.dword 0x409e57b75f96bfc8
reg_x17_init:	.dword 0x2846e77e62c716da
reg_x18_init:	.dword 0x2b16f36ebe2875d0
reg_x19_init:	.dword 0xffffffffffffff80
reg_x20_init:	.dword 0x0000000000000000
reg_x21_init:	.dword 0x1ed4f77fb27dd3dc
reg_x22_init:	.dword 0x0000000000000001
reg_x23_init:	.dword 0xe5973f36dbfb7072
reg_x24_init:	.dword 0x93f22207eeadece8
reg_x25_init:	.dword 0x4c54a10a1fb7c234
reg_x26_init:	.dword 0x12b9c4f9efee6177
reg_x27_init:	.dword 0x39f6a8aabab7b7b8
reg_x28_init:	.dword 0xffffffff80000003
reg_x29_init:	.dword 0x0800000000000007
reg_x30_init:	.dword 0x027bd1a95f66eb34
reg_x31_init:	.dword 0xffffffffffffffff

RVTEST_DATA_BEGIN

	.align 8
xreg_output_data:
reg_x0_output:	.dword 0xd08d4f4137a984b9
reg_x1_output:	.dword 0xdf827c90a30532bc
reg_x2_output:	.dword 0xaa13f8047e432244
reg_x3_output:	.dword 0x2319744405965cfc
reg_x4_output:	.dword 0x9a821ea2984f0e6b
reg_x5_output:	.dword 0xc5fc6106a4903a5a
reg_x6_output:	.dword 0x2043d910ce73b8d1
reg_x7_output:	.dword 0x888f3402d080648b
reg_x8_output:	.dword 0x18e23e25962fb70d
reg_x9_output:	.dword 0x3fdc45bd116fa5c3
reg_x10_output:	.dword 0xdca844d6538585d1
reg_x11_output:	.dword 0xe06bea68095f987a
reg_x12_output:	.dword 0xcc59e2a4e03fb84d
reg_x13_output:	.dword 0xc1107773ae9bff73
reg_x14_output:	.dword 0xd69a1d5a763e39d5
reg_x15_output:	.dword 0x81f48e6a577b19f9
reg_x16_output:	.dword 0xeb3403f26e2385c0
reg_x17_output:	.dword 0x1ac2c0faf918786b
reg_x18_output:	.dword 0xed45c2d6c7c2f993
reg_x19_output:	.dword 0x009ae254894a14fb
reg_x20_output:	.dword 0x698c4751041fa55f
reg_x21_output:	.dword 0xbafa99b7d1c7b4b2
reg_x22_output:	.dword 0xc701263c7bdd3e67
reg_x23_output:	.dword 0x13040a489b78d4b6
reg_x24_output:	.dword 0x6ce3b9faf6fb1805
reg_x25_output:	.dword 0xd3e6a425dd5da7cf
reg_x26_output:	.dword 0x3d61ababe5cb46cd
reg_x27_output:	.dword 0xb2db72b93ecbe827
reg_x28_output:	.dword 0x06a25b9ca59651f2
reg_x29_output:	.dword 0xda677c5179008151
reg_x30_output:	.dword 0x203bdcfcc77e16e8
reg_x31_output:	.dword 0x04ac06edf493568c

// Memory Blocks
	.align 8
test_memory:
	.dword 0x384f78d65438c34b, 0x64b5d65be46aa947
	.dword 0x74f6ec399d6708ff, 0x8439b7825d9fe3e8
	.dword 0x138ad5bbd3d6f097, 0xf33decfaca9fccc8
	.dword 0xdade06227336edfa, 0x471550e523ec6582
	.dword 0x3f10a857a5d31b43, 0x970a3ff8a286fa6d
	.dword 0x685974a63830bde9, 0xe464a1dbfb6d2666
	.dword 0x495263251afa7a1f, 0xba57a2b478fbced2
	.dword 0x2839916dfbf1d87d, 0xb618f1fc92b75be2
	.dword 0xf86e4f9542c101e0, 0x892c9a9b9628b119
	.dword 0x597d8854f6d1d58d, 0xb50dc832f072aa0b
	.dword 0x368ff88c5dfdbd9c, 0x990e0db7183c1f66
	.dword 0xb73fe9e8c33be593, 0x26ca013fa6c6b836
	.dword 0x510364e2eea9b751, 0x782b41200dcb0b64
	.dword 0x59b30debb7a3c22d, 0xb0dd36367a36d57f
	.dword 0xff8521e66c63705e, 0xbd9b2782d27246be
	.dword 0x345745d5e2f82e9c, 0x2b1372fc959dad45
	.dword 0x10991dd61bb8ea07, 0x0d84ffa37088ffa7
	.dword 0xd0d4df4fc7b50848, 0xb6f93cd1293b0bac
	.dword 0xa6f25267847a3df5, 0x454e42d492f64d53
	.dword 0xa923f6b993f6e739, 0x3869e8c15f05b219
	.dword 0xab353f448996c6b4, 0xad27e382227bcc7b
	.dword 0x6eec19a63f324ea0, 0x48224230e2b33bba
	.dword 0x749f34883346d834, 0x9d94dba902074bdd
	.dword 0xa145b959a8706d4c, 0x1e918158d82811fc
	.dword 0xb25c6d42d2840849, 0x80c40b5a7ddd88ba
	.dword 0x0d476d518a336e36, 0xc26291c2ff417113
	.dword 0x4e96f9fa4312072b, 0x78d7673be9affb57
	.dword 0x390905bcdb6458f6, 0xfb77a8d0f02b7c75
	.dword 0x5b887b25b79a07f7, 0x3d72770799efb37c
	.dword 0x44066dfafeec824a, 0x28522c33cfb5635f
	.dword 0x1a18abb2a99ab92b, 0xf42a240adcef44f8
	.dword 0x7b0c5f24f5dc8c96, 0x1920e8759755954f
	.dword 0xd0d2bb823d2ecd2e, 0xe0afd7a7d8a5c41b
	.dword 0xb918e284a242f85f, 0x80ee977210d00688
	.dword 0x793b79e7a6ec12c6, 0xcad1e9d43a24d23a
	.dword 0xb1d9a4190b1444cf, 0x62aab6d5f1b08491
	.dword 0x40d2502ce85d1d9f, 0xe1ab5f987413ba2e
	.dword 0x6db215157bc6a035, 0x1d06aea8170b1ab6
	.dword 0x7a470c0d85d77ecb, 0xb49cf992e1c19cd5
	.dword 0x92735e5e4c832881, 0x5d6147bdbe348685
	.dword 0xe2ae7a3baeb08d3d, 0x93e9927584a0ad90
	.dword 0x8f30230f1a0ebbcc, 0x4f3a7243bb64bc53
	.dword 0x0dbfd0daaf12be4d, 0x4c0fcd853a3c7ec4
	.dword 0x64d612f836cc58ce, 0xd2e95c3a1b757bb5
	.dword 0x6364ea0468f58e37, 0xb83b151d38fd5e59
	.dword 0x90774d1a287b0f66, 0x638ed7f4522ef407
	.dword 0x58274e42fff9f89b, 0x1965e9285a4ac23e
	.dword 0x8f6d922c18918116, 0x997e129812d478e8
	.dword 0x71440f18da2d80fe, 0x79077daa38dec4e2
	.dword 0x90c147193758f185, 0xe3959aa334e346bc
	.dword 0x3470717a95f5cb48, 0x9f66a938725cb1bd
	.dword 0x2a7e47803429e6fb, 0x2742b66493355729
	.dword 0x5a9968509ba6c445, 0x4eec237c6b7ca1c5
	.dword 0x54e08833176c5ae8, 0x74290a7fa7fa459a
	.dword 0x7c27def87f855a24, 0x870e13b2ce5e27f9
	.dword 0x808289b7fd3d7a5d, 0xdda8906cc43843bf
	.dword 0xf493d3c17f38c4fb, 0x8aaeef2d9ad40dda
	.dword 0x57f79acb69dcf691, 0xd9bbdcf625c71e0b
	.dword 0xea50426f03ec5f40, 0x4709a7d408bb1fed
	.dword 0xa07beada3343e55d, 0x39e8b6bcdaf75035
	.dword 0x3f59debd38a40e51, 0x81852a9eacae0a3a
	.dword 0xb8a46c87458731c1, 0xf03cfcb4575729b8
	.dword 0xadf29ed672853aee, 0xe4092e1b47a844d0
	.dword 0x67388829d9e79c5d, 0x5791dca2d9480252

.align 8
loop_count: .word 0x40

RVTEST_DATA_END
