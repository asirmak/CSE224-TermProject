`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   10:10:34 05/22/2023
// Design Name:   VerySimpleCPU
// Module Name:   C:/Users/ASI/Desktop/CSE224/vscpu/tb.v
// Project Name:  vscpu
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: VerySimpleCPU
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb;

	parameter SIZE = 14, DEPTH = 2**14;

	reg clk;
	initial begin
	  clk = 1;
	  forever
		  #5 clk = ~clk;
	end
	
	reg interrupt;
	initial begin
	  interrupt = 1'b0;
	  repeat (56) @(negedge clk);
	  interrupt <=#1 1;
	  @ (posedge clk);
		interrupt <=#1 0;
	end
	
	reg rst;
	initial begin
	  rst = 1;
	  repeat (10) @(posedge clk);
	  rst <= #1 0;
	  repeat (300) @(posedge clk);
	  $finish;
	end
	  
	wire wrEn;
	wire [SIZE-1:0] addr_toRAM;
	wire [31:0] data_toRAM, data_fromRAM;

	VerySimpleCPU inst_VerySimpleCPU(
		.clk(clk),
		.rst(rst),
		.wrEn(wrEn),
		.data_fromRAM(data_fromRAM),
		.addr_toRAM(addr_toRAM),
		.data_toRAM(data_toRAM),
		.interrupt(interrupt)
	);

	 blram #(SIZE, DEPTH) inst_blram(
		.clk(clk),
		.i_we(wrEn),
		.i_addr(addr_toRAM),
		.i_ram_data_in(data_toRAM),
		.o_ram_data_out(data_fromRAM)
	);

	initial begin
		inst_blram.memory[0] = 32'hc000c010;
		inst_blram.memory[1] = 32'hd0010000;
		inst_blram.memory[3] = 32'h46;
		inst_blram.memory[4] = 32'h1e;
		inst_blram.memory[5] = 32'h8;
		inst_blram.memory[8] = 32'hd0018000;
		inst_blram.memory[10] = 32'h0;
		inst_blram.memory[12] = 32'h0;
		inst_blram.memory[15] = 32'h5;
		inst_blram.memory[16] = 32'h3;
		inst_blram.memory[17] = 32'h0;
		inst_blram.memory[18] = 32'h0;
		inst_blram.memory[19] = 32'h3c;
		inst_blram.memory[20] = 32'hffffffff;
		inst_blram.memory[21] = 32'h44;
		inst_blram.memory[23] = 32'h0;
		inst_blram.memory[24] = 32'h28;
		inst_blram.memory[30] = 32'h8002800f;
		inst_blram.memory[31] = 32'h80030010;
		inst_blram.memory[32] = 32'h20030014;
		inst_blram.memory[33] = 32'h10030001;
		inst_blram.memory[34] = 32'hd0060000;
		inst_blram.memory[40] = 32'h80044010;
		inst_blram.memory[41] = 32'h8005c00a;
		inst_blram.memory[42] = 32'h6005c011;
		inst_blram.memory[43] = 32'hc004c017;
		inst_blram.memory[60] = 32'hc005400a;
		inst_blram.memory[61] = 32'h10048001;
		inst_blram.memory[62] = 32'h2800c;
		inst_blram.memory[63] = 32'h8005c00a;
		inst_blram.memory[64] = 32'h5c00c;
		inst_blram.memory[65] = 32'h8004400c;
		inst_blram.memory[66] = 32'h60044017;
		inst_blram.memory[67] = 32'hc0060011;
		inst_blram.memory[70] = 32'h80048014;
	end
  
endmodule
