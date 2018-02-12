/*
Kevin Middleton
CS354 - Four-Bit CPU
*/

module li_decoder(op, out);
	input [2:0] op;
	output out;
	wire not_op;

	not g1(not_op, op[2]);
	or g2(out, not_op, op[1], op[0]);
endmodule

module D_latch(D,C,Q);
   input D,C; 
   output Q;
   wire x,y,D1,Q1;

   nand nand1 (x,D, C), 
        nand2 (y,D1,C), 
        nand3 (Q,x,Q1),
        nand4 (Q1,y,Q); 
   not  not1  (D1,D);
endmodule

module D_flip_flop(D,CLK,Q);
   input D,CLK; 
   output Q; 
   wire CLK1, Y;

   not  not1 (CLK1,CLK);
   D_latch D1(D,CLK, Y),
           D2(Y,CLK1,Q);
endmodule 
 
module quad_mux_2(a, b, c, result);
	input [3:0] a,b;
	input c;
	output [3:0] result;
	wire [7:0] X;
	wire not_c;
	
	not g1(not_c, c);
	and g2(X[0], a[0], not_c),
	    g3(X[1], a[1], not_c),
	    g4(X[2], a[2], not_c),
	    g5(X[3], a[3], not_c),
	    g6(X[4], b[0], c),
	    g7(X[5], b[1], c),
	    g8(X[6], b[2], c),
	    g9(X[7], b[3], c);
    or  g10(result[0], X[0], X[4]),
	    g11(result[1], X[1], X[5]),
	    g12(result[2], X[2], X[6]),
	    g13(result[3], X[3], X[7]);
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////ALU Start

module full_adder(S,C,x,y,z);
	input x,y,z;
	output S,C;
	wire not_x,not_y,not_z,w1,w2,w3,w4,w5,w6,w7;

	not g1(not_x,x),
	    g2(not_y,y),
	    g3(not_z,z);
	and g4(w1,not_x,not_y,z),
	    g5(w2,not_x,y,not_z),
	    g6(w3,x,not_y,not_z),
	    g7(w4,x,y,z);
	or  g8(S,w1,w2,w3,w4);
	
	and g9(w5,x,y),
	    g10(w6,x,z),
	    g11(w7,y,z);
	or  g12(C,w5,w6,w7);
endmodule

module mux_2(result,a,b,c);
	input a,b,c;
	output result;
	wire w1,w2,w3;

	not (w3,c);
	and g1(w1,b,c),
	    g2(w2,a,w3);
	or  g3(result,w1,w2);
endmodule

module mux_4(result,choice,a,b,c,d);
	input a, b, c, d;
	input [2:0] choice;
	output result;
	wire w1,w2,w3,w4;
	wire [1:0] not_choice;
	
	not g1(not_choice[0], choice[0]),
	    g2(not_choice[1], choice[1]);

	and g3(w1, not_choice[0], not_choice[1], a),
	    g4(w2, choice[0], not_choice[1], b),
	    g5(w3, not_choice[0], choice[1], c),
	    g6(w4, choice[0], choice[1], d);
	or  g7(result, w1, w2, w3, w4);
endmodule

module detect_overflow(overflow, carry_in, carry_out, op);
	input carry_in, carry_out;
	input [2:0] op;
	output overflow;
	wire nop, arith, over;
	
	not    g1(nop,op[0]);
	and    g2(arith, nop, op[1]);
	xor    g3(over, carry_in, carry_out);
	mux_2 mux(overflow,0,over,arith);
endmodule

module onebit_alu(result, carry_out, carry_in, less, a, b, op);
	input carry_in, less, a, b;
	input [2:0] op;
	output result, carry_out;
	wire w1, w2, w3, w4, w5;
	
	not        g3(w4, b);
	mux_2      g5(w5,b,w4, op[2]);
	and        g1(w1, a, w5);
	or         g2(w2, a, w5);
	full_adder g4(w3, carry_out, a, w5, carry_in);
	mux_4      mux(result, op, w1, w2, w3, less);
endmodule

module set_onebit_alu(result, overflow, set, carry_in, less, x, y, op);
	input carry_in, less, x, y;
	input [2:0] op;
	output result, overflow, set;
	wire w1, w2, w4, w5, carry_out;

	not            g3(w4, y);
	mux_2          g5(w5,y,w4, op[2]);
	and            g1(w1, x, w5);
	or             g2(w2, x, w5);
	full_adder     g3(set, carry_out, x, w5, carry_in);
	mux_4          g4(result, op, w1, w2, set, less);
	detect_overflow o(overflow, carry_out, carry_in, op);
endmodule

module fourbit_alu(a,b,op,result,overflow,zero);
	input [3:0] a, b;
	input [2:0] op;
	output [3:0]result;
	output overflow,zero;
	wire [3:0] carry_out;
	wire set;

	onebit_alu     a1(result[0], carry_out[0], op[2], set,a[0],b[0],op),
	               a2(result[1], carry_out[1], carry_out[0], 0,a[1],b[1],op),
	               a3(result[2], carry_out[2], carry_out[1], 0,a[2],b[2],op);
	set_onebit_alu a4(result[3], overflow, set, carry_out[2], 0, a[3], b[3], op);	
	nor            a5(zero, result[0], result[1], result[2], result[3]);
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////ALU End

module cpu (Instruction, WriteData, CLK);
   input [8:0] Instruction;
   input CLK;
   output [3:0] WriteData;
   wire [8:0] IR;
   wire [3:0] x,y, alu_out;
   //wire [2:0] instr; --- XCredit
   //wire [1:0] a, b, waddr; --- XCredit
   wire decoder, overflow, zero;

   instr_reg instrR(Instruction, IR, CLK);  //instr_reg instrR({instr, a, b, waddr}, instruction, CLK); --- XCredit
   quad_mux_2   mux(IR[5:2], alu_out, decoder, WriteData);
   li_decoder   dec(IR[8:6], decoder);
   regfile     regs(IR[5:4], IR[3:2], IR[1:0], WriteData, x, y, CLK); //regfile regs(IR[5:4],IR[3:2],x,y,IR[1:0],WriteData,CLK); --- XCredit
   fourbit_alu  alu(x, y, IR[8:6], alu_out, overflow, zero);
endmodule

module instr_reg (Instruction,IR,CLK);
   input [8:0] Instruction;
   input CLK;
   output [8:0] IR;
 
   D_flip_flop d0(Instruction[0], CLK, IR[0]);
   D_flip_flop d1(Instruction[1], CLK, IR[1]);
   D_flip_flop d2(Instruction[2], CLK, IR[2]);
   D_flip_flop d3(Instruction[3], CLK, IR[3]);
   D_flip_flop d4(Instruction[4], CLK, IR[4]);
   D_flip_flop d5(Instruction[5], CLK, IR[5]);
   D_flip_flop d6(Instruction[6], CLK, IR[6]);
   D_flip_flop d7(Instruction[7], CLK, IR[7]);
   D_flip_flop d8(Instruction[8], CLK, IR[8]);
endmodule


module regfile (ReadReg1,ReadReg2,WriteReg,WriteData,ReadData1,ReadData2,CLK);
  input [1:0] ReadReg1,ReadReg2,WriteReg;
  input [3:0] WriteData;
  input CLK;
  output [3:0] ReadData1,ReadData2;
  reg [3:0] Regs[0:3]; 
  assign ReadData1 = Regs[ReadReg1];
  assign ReadData2 = Regs[ReadReg2];
  initial Regs[0] = 0;
  always @(negedge CLK)
     Regs[WriteReg] <= WriteData;
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////Extra Credit Start
/*
module fourbit_register (A, I, CLK);
   output [3:0] A;
   input  [3:0] I;
   input        CLK;
   
   D_flip_flop D0 (A[0], I[0], CLK),
               D1 (A[1], I[1], CLK),
               D2 (A[2], I[2], CLK),
               D3 (A[3], I[3], CLK);
endmodule

module twofour_decoder (Decode, WriteAddress);
  input  [1:0] WriteAddress;
  output [3:0] Decode;
  wire not_x, not_y;  
 
  not n1 (not_x, WriteAddress[1]),
      n2 (not_y, WriteAddress[0]);
  and n3 (Decode[0],not_x,not_y),
      n4 (Decode[1],not_x,WriteAddress[0]),
      n5 (Decode[2],WriteAddress[1],not_y),
      n6 (Decode[3],WriteAddress[1],WriteAddress[0]);
endmodule

module regfile (ReadReg1, ReadReg2, ReadData1, ReadData2, WriteReg, WriteData, CLK);
   input  [1:0] ReadData1, ReadData2, WriteReg;
   input  [3:0] WriteData;
   input        CLK;
   output [3:0] ReadReg1, ReadReg2;
   wire   [3:0] zerobit_mux, onebit_mux, twobit_mux, threebit_mux, decode;
   wire   C1, c2, c3;
 
   and               and1 (c1, decode[1], CLK),
                     and2 (c2, decode[2], CLK),
                     and3 (c3, decode[3], CLK);
   
   twofour_decoder        dec1 (decode, WriteReg);
 
   fourbit_register  r1 ({threebit_mux[1], twobit_mux[1], onebit_mux[1], zerobit_mux[1]}, WriteData, c1),
                     r2 ({threebit_mux[2], twobit_mux[2], onebit_mux[2], zerobit_mux[2]}, WriteData, c2),
                     r3 ({threebit_mux[3], twobit_mux[3], onebit_mux[3], zerobit_mux[3]}, WriteData, c3);
   mux_4             zerobitmux1  (ReadReg1[0], {zerobit_mux[3],  zerobit_mux[2],  zerobit_mux[1],  1'b0}, ReadData1), //ReadReg1 muxs
                     onebitmux1   (ReadReg1[1], {onebit_mux[3],   onebit_mux[2],   onebit_mux[1],   1'b0}, ReadData1),
                     twobitmux1   (ReadReg1[2], {twobit_mux[3],   twobit_mux[2],   twobit_mux[1],   1'b0}, ReadData1),
                     threebitmux1 (ReadReg1[3], {threebit_mux[3], threebit_mux[2], threebit_mux[1], 1'b0}, ReadData1),
                     zerobitmux2  (ReadReg2[0], {zerobit_mux[3],  zerobit_mux[2],  zerobit_mux[1],  1'b0}, ReadData2), //ReadReg2 muxs
                     onebitmux2   (ReadReg2[1], {onebit_mux[3],   onebit_mux[2],   onebit_mux[1],   1'b0}, ReadData2),
                     twobitmux2   (ReadReg2[2], {twobit_mux[3],   twobit_mux[2],   twobit_mux[1],   1'b0}, ReadData2),
                     threebitmux2 (ReadReg2[3], {threebit_mux[3], threebit_mux[2], threebit_mux[1], 1'b0}, ReadData2);
endmodule
*/
///////////////////////////////////////////////////////////////////////////////////////////////////Extra Credit End

module test_cpu;
   reg [8:0] Instruction;
   reg CLK;
   wire signed [3:0] WriteData;
   cpu cpu1 (Instruction, WriteData, CLK);

   initial
   begin
         #1 CLK = 1; Instruction = 9'b100111110; //LI  $2, 15
         #1 CLK = 0; Instruction = 9'b100111110;
         #1 CLK = 1; Instruction = 9'b100100011; //LI  $3, 8
         #1 CLK = 0; Instruction = 9'b100100011;
         #1 CLK = 1; Instruction = 9'b000101101; //AND $1, $2, $3
         #1 CLK = 0; Instruction = 9'b000101101;
         #1 CLK = 1; Instruction = 9'b110011011; //SUB $3, $1, $2
         #1 CLK = 0; Instruction = 9'b110011011;
         #1 CLK = 1; Instruction = 9'b111110010; //SLT $2, $3, $0
         #1 CLK = 0; Instruction = 9'b111110010;
         #1 CLK = 1; Instruction = 9'b010101101; //ADD $1, $2, $3
         #1 CLK = 0; Instruction = 9'b010101101;
         #1 CLK = 1; Instruction = 9'b110000101; //SUB $1, $0, $1
         #1 CLK = 0; Instruction = 9'b110000101;
         #1 CLK = 1; Instruction = 9'b001011011; //OR  $3, $1, $2
         #1 CLK = 0; Instruction = 9'b001011011;
   end

   initial
   begin
      $display("\nCLK __ Instruction __ WriteDataBinary __ WriteDataInt\n---------------------------------------------------------------------"); 
      $monitor("%b _____ %b _______  %b __________ %d", CLK, Instruction, WriteData, WriteData);
   end

endmodule