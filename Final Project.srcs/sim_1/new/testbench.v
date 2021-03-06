
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/12/2020 06:06:26 PM
// Design Name: 
// Module Name: testbench
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

`timescale 1ns / 1ps
module testbench();
reg clock;
wire [31:0] PC, sum_tb, IF, IF_reg;
wire wreg, m2reg, wmem, aluimm, regr1;
wire [3:0]aluc;
wire [31:0] imm_ext;
wire [4:0] DstReg;
wire [31:0]qa, qb;
wire ewreg, em2reg, ewmem, ealuimm;
wire [3:0] ealuc;
wire [4:0] edstReg, ers, ert;
wire [31:0] eqa, eqb, eimm, aluina, aluinb, finalb, r;
wire [1:0] fa, fb;
wire mwreg, mm2reg, mwmem; 
wire [4:0] mdstReg;
wire [31:0] mres, mqb, dout;
wire wwreg, wm2reg;
wire [4:0] wdstReg;
wire [31:0] wres, wdout, wrd;
PCReg pc_reg_tb(clock, sum_tb, PC);    //IF Stage
PCAdder add_tb(PC, sum_tb);
InstrMem imem_tb(PC, IF);
IFIDReg IFID(clock, IF, IF_reg);            //ID Stage
Control control(IF_reg[31:26], IF_reg[5:0], wreg, m2reg, wmem, aluc, aluimm, regr1);
Extender sign_ext_tb(IF_reg[15:0], imm_ext);
Mux mux(regr1, IF_reg[15:11], IF_reg[20:16], DstReg);
RegFile RF(clock, IF_reg[25:21], IF_reg[20:16], wdstReg, wwreg, wrd, qa, qb);
IDEXEReg IDEXE(clock, wreg, m2reg, wmem, aluc, aluimm, DstReg, qa, qb, imm_ext, IF_reg[25:21], IF_reg[20:16],
ewreg, em2reg, ewmem, ealuc, ealuimm, edstReg, eqa, eqb, eimm, ers, ert);                     //EXE Stage
Mux32x2 immMux(ealuimm, aluinb, eimm, finalb);
Mux3to1 muxa(eqa, wrd, mres, dout, fa, aluina);
Mux3to1 muxb(eqb, wrd, mres, dout, fb, aluinb);
ALU alu(aluina, finalb, ealuc, r);
EXEMEMReg EXEMEM(clock, ewreg, em2reg, ewmem, edstReg, r, aluinb, mwreg, mm2reg,      //MEM Stage
mwmem, mdstReg, mres, mqb);
DataMem DMEM(mwmem, mres, mqb, dout);
MEMWBReg MEMWB(clock, mwreg, mm2reg, mdstReg, mres, dout, wwreg, wm2reg, wdstReg, wres, wdout); //Beginning of WB Stage
Mux32x2 writeMux(wm2reg, wres, wdout, wrd);
ForwardingUnit fowardingUnit(ers, ert, mdstReg, wdstReg, mwreg, wwreg, mm2reg, fa, fb);
initial begin
    clock = 0;
    forever #5 clock = ~clock;
end
endmodule
