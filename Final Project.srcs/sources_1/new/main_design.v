`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/12/2020 05:53:19 PM
// Design Name: 
// Module Name: main_design
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


//--------------------------------------//------------------------------------
module PCReg (
    input clk,
    input [31:0] new,
    output reg [31:0] pc
);
initial begin
    pc <= 32'd100;
end
always @(posedge clk) begin
    pc <= new;
end
endmodule

module PCAdder (
    input [31:0] val,
    output reg [31:0] sum
);
always @(val) begin
    sum <= val + 32'd4;
end
endmodule

module InstrMem (
    input [31:0] addr,
    output reg [31:0] instr
);
reg [31:0]instrmem[511:0];
wire [31:0] word_addr = addr >> 2;
integer i;
initial begin
    /*for (i=0; i<511; i=i+1) begin
        instrmem[i] = 32'd0;
    end*/
    instrmem[25] = 32'h00221820;
    instrmem[26] = 32'h01232022;
    instrmem[27] = 32'h00692825;
    instrmem[28] = 32'h00693026;
    instrmem[29] = 32'h00693824;
end
always @(addr) begin
    instr <= instrmem[word_addr];
end
endmodule

module IFIDReg (
    input clk,
    input [31:0] do,
    output reg [31:0] instr
);
always @(posedge clk) begin
    instr <= do;
end
endmodule

module RegFile (
    input clk,
    input [4:0] read1,
    input [4:0] read2,
    input [4:0] wreg,
    input regw,
    input [31:0] WriteData,
    output [31:0] rd1,
    output [31:0] rd2
);
reg [31:0]RFile[31:0];
integer i;
initial begin
    RFile[0] <= 32'h00000000;
    RFile[1] <= 32'hA00000AA;
    RFile[2] <= 32'h10000011;
    RFile[3] <= 32'h20000022;
    RFile[4] <= 32'h30000033;
    RFile[5] <= 32'h40000044;
    RFile[6] <= 32'h50000055;
    RFile[7] <= 32'h60000066;
    RFile[8] <= 32'h70000077;
    RFile[9] <= 32'h80000088;
    RFile[10] <= 32'h90000099;
end
assign rd1 = RFile[read1];
assign rd2 = RFile[read2];
always @(negedge clk) begin
    if (regw) begin
        $display("regwrite %h", WriteData);
        RFile[wreg] <= WriteData;
    end
end
endmodule

module Mux (
    input sel,
    input [4:0] rd,
    input [4:0] rt,
    output [4:0] regOut
);
assign regOut = sel ? rt:rd;
endmodule

module IDEXEReg (
    input clk,
    input WriteReg,
    input MemToReg,
    input WriteMem,
    input [3:0] ALUc,
    input ALUImm,
    input [4:0] DstReg,
    input [31:0] dataA,
    input [31:0] dataB,
    input [31:0] immediate,
    input [4:0] rs_in,
    input [4:0] rt_in,
    output reg wreg,
    output reg m2reg,
    output reg wmem,
    output reg [3:0] aluc,
    output reg aluimm,
    output reg [4:0] dst,
    output reg [31:0] qa,
    output reg [31:0] qb,
    output reg [31:0] imm,
    output reg [4:0] rs,
    output reg [4:0] rt
);
always @(posedge clk) begin
    wreg <= WriteReg;
    m2reg <= MemToReg;
    wmem <= WriteMem;
    aluc <= ALUc;
    aluimm <= ALUImm;
    dst <= DstReg;
    qa <= dataA;
    qb <= dataB;
    imm <= immediate;
    rs <= rs_in;
    rt <= rt_in;
end
endmodule

module Extender (
    input [15:0] imm,
    output reg [31:0] extend
);
always @(imm) begin
    extend <= {{16{imm[15]}}, imm[15:0]};
end
endmodule

module Control (
    input [5:0] op,
    input [5:0] func,
    output reg wreg,
    output reg m2reg,
    output reg wmem,
    output reg [3:0] aluc,
    output reg aluimm,
    output reg regr1
);
always @(*) begin
        case(op) 
            6'b100011: begin //lw
             regr1 <= 1;
             aluc <= 4'b0010; 
             aluimm <= 1;
             m2reg <= 1;
             wreg <= 1; 
             wmem <= 0; end
            6'b101011: begin //sw
             aluimm <= 1; 
             wreg <= 0;
             wmem <= 1;
             aluc <= 4'b0010; end
            6'b001000: begin //addi
             regr1 <= 1;
             m2reg <= 0;
             wreg <= 1;
             wmem <= 0;
             aluimm <= 1;
             aluc <= 4'b0010; end
            6'b001100: begin //andi
             regr1 <= 1;
             m2reg <= 0;
             wreg <= 1;
             wmem <= 0;
             aluimm <= 1;
             aluc <= 4'b0000; end
            6'b001101: begin //ori
             regr1 <= 1;
             m2reg <= 0;
             wreg <= 1;
             wmem <= 0;
             aluimm <= 1;
             aluc <= 4'b0001; end
            6'b000100: begin //beq
             wreg <= 0;
             wmem <= 0;
             aluimm <= 0;
             aluc <= 4'b0110; end
            6'b000101: begin //bne
             wreg <= 0;
             wmem <= 0;
             aluimm <= 0;
             aluc <= 4'b0110; end
            6'b001111: begin //lui
             regr1 <= 1;
             m2reg <= 0;
             wreg <= 1;
             wmem <= 0;
             aluimm <= 1;
             aluc <= 4'b0000; end
            6'b000000: begin //RFORMAT instructions
             regr1 <= 0;
             m2reg <= 0;
             wreg <= 1;
             wmem <= 0;
             aluimm <= 0;
             case(func)
                 6'b100000: begin //add
                  aluc <= 4'b0010; end
                 6'b100010: begin //sub
                  aluc <= 4'b0110; end
                 6'b100100: begin //and
                  aluc <= 4'b0000; end
                 6'b100101: begin //or
                  aluc <= 4'b0001; end
                 6'b100110: begin //xor
                  aluc <= 4'b0011; end
             endcase
            end
        endcase
end
endmodule

module ALU (
    input [31:0] a,
    input [31:0] b,
    input [3:0] aluc,
    output reg [31:0] r
);
always @(*) begin
    case(aluc)
        4'b0000: r <= a&b;
        4'b0001: r <= a|b;
        4'b0010: r <= a+b;
        4'b0110: r <= a-b; 
        4'b0011: r <= (~a&b)|(a&~b);
        4'b0111: begin
            if (a<b) begin
                r <= 1;
            end 
            else begin
                r <= 0;
            end end
        4'b1100: r <= ~(a|b);
    endcase
end
endmodule

module Mux32x2 (
    input sel,
    input [31:0] b,
    input [31:0] imm,
    output [31:0] res
);
assign res = sel ? imm:b;
endmodule

module EXEMEMReg (
    input clk,
    input wreg,
    input m2reg,
    input wmem,
    input [4:0] dstReg,
    input [31:0] res,
    input [31:0] qb,
    output reg mwreg,
    output reg mm2reg,
    output reg mwmem,
    output reg [4:0] mdstreg,
    output reg [31:0] mres,
    output reg [31:0] mqb
);
always @(posedge clk) begin
    mwreg <= wreg;
    mm2reg <= m2reg;
    mwmem <= wmem;
    mdstreg <= dstReg;
    mres <= res;
    mqb <= qb;
end
endmodule

module DataMem (
    input write,
    input [31:0] a,
    input [31:0] di,
    output reg [31:0] do
);
reg [31:0]mem[511:0];
initial begin
    mem[0] <= 32'hA00000AA;
    mem[1] <= 32'h10000011;
    mem[2] <= 32'h20000022;
    mem[3] <= 32'h30000033;
    mem[4] <= 32'h40000044;
    mem[5] <= 32'h50000055;
    mem[6] <= 32'h60000066;
    mem[7] <= 32'h70000077;
    mem[8] <= 32'h80000088;
    mem[9] <= 32'h90000099;
end
wire [31:0] addr = a >> 2;
always @(a) begin
    if (write) begin
        mem[addr] <= di;
        do <= 0;
    end
    else begin
        do <= mem[addr];
    end
end
endmodule
module MEMWBReg (
    input clk,
    input wreg,
    input m2reg,
    input [4:0] dstReg,
    input [31:0] res,
    input [31:0] do,
    output reg wwreg,
    output reg wm2reg,
    output reg [4:0] wdstReg,
    output reg [31:0] wres,
    output reg [31:0] wd
);
always @ (posedge clk) begin
    wwreg <= wreg;
    wm2reg <= m2reg;
    wdstReg <= dstReg;
    wres <= res;
    wd <= do;
end
endmodule
module ForwardingUnit(
    input [4:0] rsidex,
    input [4:0] rtidex,
    input [4:0] rdexemem,
    input [4:0] rdmemwb,
    input regWriteExemem,
    input regWriteMemwb,
    input m2regExemem,
    output reg [1:0] fa,
    output reg [1:0] fb
);
always @(*) begin
    if (rdexemem && regWriteExemem && rdexemem == rsidex) begin
        fa <= 2'b10;
    end
    else if (rdmemwb && regWriteMemwb && rdmemwb == rsidex) begin
        fa <= 2'b1;
    end
    else if (rdexemem && m2regExemem && regWriteExemem && rdexemem == rsidex) begin
        fa <= 2'b11;
    end
    else begin
        fa <= 2'b0;
    end
    if (rdexemem && regWriteExemem && rdexemem == rtidex) begin
        fb <= 2'b10;
    end
    else if (rdmemwb && regWriteMemwb && rdmemwb == rtidex) begin
        fb <= 2'b1;
    end
    else if (rdexemem && m2regExemem && regWriteExemem && rdexemem == rtidex) begin
        fb <= 2'b11;
    end
    else begin
        fb <= 2'b0;
    end
end
endmodule
module Mux3to1(
    input [31:0] in1,
    input [31:0] in2,
    input [31:0] in3,
    input [31:0] in4,
    input [1:0] sel,
    output reg [31:0] out
);
always @(*) begin
    case(sel)
        2'b0: begin
            assign out = in1;
        end
        2'b1: begin
            assign out = in2;
        end
        2'b10: begin
            assign out = in3;
        end
        2'b11: begin
            assign out = in4;
        end
    endcase
end
endmodule
