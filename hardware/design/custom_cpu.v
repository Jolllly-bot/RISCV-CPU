`timescale 10ns / 1ns

module custom_cpu(
	input  rst,
	input  clk,

	//Instruction request channel
	output reg [31:0] PC,
	output reg Inst_Req_Valid,
	input Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input Inst_Valid,
	output reg Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output MemWrite,
	output [31:0] Write_data,
	output [3:0] Write_strb,
	output MemRead,
	input Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input Read_data_Valid,
	output reg Read_data_Ready, 

    output [31:0]	cpu_perf_cnt_0,
    output [31:0]	cpu_perf_cnt_1,
    output [31:0]	cpu_perf_cnt_2,
    output [31:0]	cpu_perf_cnt_3,
    output [31:0]	cpu_perf_cnt_4,
    output [31:0]	cpu_perf_cnt_5,
    output [31:0]	cpu_perf_cnt_6,
    output [31:0]	cpu_perf_cnt_7,
    output [31:0]	cpu_perf_cnt_8,
    output [31:0]	cpu_perf_cnt_9,
    output [31:0]	cpu_perf_cnt_10,
    output [31:0]	cpu_perf_cnt_11,
    output [31:0]	cpu_perf_cnt_12,
    output [31:0]	cpu_perf_cnt_13,
    output [31:0]	cpu_perf_cnt_14,
    output [31:0]	cpu_perf_cnt_15

);

  //TODO: Please add your RISC-V CPU code here

	//FSM define
	localparam RST = 9'b000000001;
	localparam IF  = 9'b000000010;
	localparam IW  = 9'b000000100;
	localparam ID  = 9'b000001000;
	localparam EX  = 9'b000010000;
	localparam WB  = 9'b000100000;
	localparam ST  = 9'b001000000;
	localparam LD  = 9'b010000000;
	localparam RDW = 9'b100000000;
	localparam HIGH = 1'b1;
	localparam LOW  = 1'b0;
	reg   [8:0]     current_state;
	reg   [8:0]     next_state;
	reg   [31:0]    ALUReg;
	reg   [31:0]    ResultReg;
	reg   [31:0]    MemReg;
	reg   [31:0]    InstReg;
	reg   [31:0]    rdata1Reg;
	reg   [31:0]    rdata2Reg;
	//alu define
	wire  [31:0]    ALU_A;
	wire  [31:0]    ALU_B;
	wire  [2:0]     ALUop;
	wire  [31:0]    ALU_result;
	wire		    ALU_overflow;
	wire            ALU_carryout;
	wire            ALU_zero;
	//reg_file define
	wire			RF_wen;
	wire  [4:0]		RF_waddr;
	wire  [31:0]	RF_wdata;
	wire  [4:0]     raddr1;
	wire  [4:0]     raddr2;
	wire  [31:0]    rdata1;
	wire  [31:0]    rdata2;
	//shifter define
	wire  [31:0]    Shift_A;
	wire  [31:0]    Shift_B;
	wire  [1:0]     Shiftop;
	wire  [31:0]    Shift_result;
	//instruction
	wire  [6:0]     opcode;
	wire  [2:0]     funct;
	wire  [4:0]     rd;
	wire  [4:0]     rs1;
	wire  [4:0]     rs2;
	wire  [6:0]     func7;
	//type decode
	wire Utype;
	wire jalr ;
	wire jal  ;
	wire Btype;
	wire Iload;
	wire Stype;
	wire Ioprt;
	wire Rtype;
	wire lui;
	wire auipc;
	wire shift;
	//control signals
	wire Mem2Reg;
	wire ALUsrc;
	wire Branch;
	//sign extesion
	wire  [31:0]    Ioprt_ext;
	wire  [31:0]    Btype_ext;
	wire  [31:0]    Stype_ext;
	wire  [31:0]    Utype_ext;
	wire  [31:0]    Jtype_ext;
	wire  [31:0]    imm_data;
	wire  [31:0]    Data_result;
	//PC
	wire  [31:0]    PC_next;
	wire  [31:0]    PC_plus4;
	wire  [31:0]    PC_tar;
	wire  [31:0]    jalr_addr;
	//load & store
	wire 			sb;
	wire 			sh;
	wire 			sw;
	wire  [3:0]     addrtype;//one-hot code
	wire  			lbu;     //others are same as store
	wire  			lhu;
	wire  [31:0]    lb_data;
	wire  [31:0]    lh_data;
	wire  [31:0]    lw_data;
	wire  [31:0]    lbu_data;
	wire  [31:0]    lhu_data;
	wire  [31:0]    Load_data;
	//cnt define
	reg   [31:0]    cycle_cnt;
	reg   [31:0]    store_cnt;

	// **********************************

	/*
	performance counter
	*/
	always @(posedge clk) begin
		if (rst) 
			cycle_cnt <= 32'd0;
		else
			cycle_cnt <= cycle_cnt + 32'd1;
	end
	assign cpu_perf_cnt_0 = cycle_cnt;

	always @(posedge clk) begin
		if (rst) 
			store_cnt <= 32'd0;
		else if(current_state == ST)
			store_cnt <= store_cnt + 32'd1;
	end
	assign cpu_perf_cnt_1 = store_cnt;

	/*
	state machine
	*/
	always @(posedge clk) begin
        if (rst) begin
            current_state <= RST;
        end 
		else begin
            current_state <= next_state;
        end
    end

	always @(*) begin
		case(current_state) 
			RST : begin
				if(rst) next_state = RST;
				else next_state = IF;
			end
			IF : begin
				if(Inst_Req_Ready) next_state = IW;
				else next_state = IF;
			end
			IW : begin
				if(Inst_Valid) next_state = ID;
				else next_state = IW;
			end
			ID : begin
				next_state = EX;
			end
			EX : begin
				if(Iload) next_state = LD;
				else if(Stype) next_state = ST;
				else if(Btype) next_state = IF;
				else next_state = WB;
			end
			LD : begin
				if (Mem_Req_Ready) next_state = RDW;
				else next_state = LD;
			end
			RDW : begin
				if(Read_data_Valid) next_state = WB;
				else next_state = RDW;
			end
			ST : begin
				if(Mem_Req_Ready) next_state = IF;
				else next_state = ST;
			end
			WB : begin
				next_state = IF;
			end
			default : begin
				next_state = IF;
			end
		endcase
	end

	//output
	always @(*) begin
		if(current_state == RST)
			Inst_Req_Valid = LOW;
		else if(current_state == IF)
			Inst_Req_Valid = HIGH;
		else if(current_state == IW)
			Inst_Req_Valid = LOW;
	end

	always @(*) begin
		if(current_state == RST)
			Inst_Ready = HIGH;
		else if(current_state == IF)
			Inst_Ready = LOW;
		else if(current_state == IW)
			Inst_Ready = HIGH;
		else if(current_state == ID)
			Inst_Ready = LOW;
	end

	always @(*) begin
		if(next_state == RST)
			Read_data_Ready = HIGH;
		else if(current_state == IF)
			Read_data_Ready = LOW;
		else if(current_state == RDW)
			Read_data_Ready = HIGH;
		else if(current_state == WB)
			Read_data_Ready = LOW;
	end

	always @(*) begin
		if (current_state == RST)
			InstReg = 32'd0;
		else if(current_state == IW && Inst_Valid) begin
			InstReg = Instruction;
		end
	end

	always @(*) begin
		if (current_state == RST)begin
			rdata1Reg = 32'd0;
			rdata2Reg = 32'd0;
		end
		else if(current_state == ID) begin
			rdata1Reg = rdata1;
			rdata2Reg = rdata2;
		end
	end

	always @(*) begin
		if(current_state == RST) begin
			ALUReg = 32'd0;
			ResultReg = 32'd0;
		end
		else if(current_state == EX) begin
			ALUReg = ALU_result;
			ResultReg = Data_result;
		end
	end

	always @(*) begin
		if(current_state == RST)
			MemReg = 32'd0;
		else if(current_state == RDW && Read_data_Valid) begin
			MemReg = Load_data;
		end
	end

	/*
	instruction
	*/
	assign opcode  = InstReg[6:0];
	assign rd      = InstReg[11:7];
	assign funct   = InstReg[14:12];
	assign rs1     = InstReg[19:15];
	assign rs2     = InstReg[24:20];
	assign func7   = InstReg[31:25];

	/*
	decoder
	*/
	assign Utype   = (~opcode[6] &  opcode[4]) & (~opcode[3] &  opcode[2]);
	assign jalr    = ( opcode[6] &  opcode[5]) & (~opcode[4] & ~opcode[3]) &  opcode[2];
	assign jal     = ( opcode[6] &  opcode[5]) & (~opcode[4] &  opcode[3]) &  opcode[2];
	assign Btype   = ( opcode[6] &  opcode[5]) & (~opcode[4] & ~opcode[3]) & ~opcode[2];
	assign Iload   = (~opcode[6] & ~opcode[5]) & (~opcode[4] & ~opcode[3]) & ~opcode[2];
	assign Stype   = (~opcode[6] &  opcode[5]) & (~opcode[4] & ~opcode[3]) & ~opcode[2];
	assign Ioprt   = (~opcode[6] & ~opcode[5]) & ( opcode[4] & ~opcode[3]) & ~opcode[2];
	assign Rtype   = (~opcode[6] &  opcode[5]) & ( opcode[4] & ~opcode[3]) & ~opcode[2];

	assign lui = Utype & opcode[5];
	assign auipc = Utype & ~opcode[5];
	assign shift = (Rtype | Ioprt) & (funct[0] & ~funct[1]);

	/*
	control unit
	*/
	assign MemRead = current_state == LD;
	assign MemWrite = current_state == ST;
	assign ALUsrc = Iload | Stype | Ioprt;
	assign Mem2Reg = Iload;

	/*
	sign extension
	*/
	assign Ioprt_ext = {{20{InstReg[31]}} , InstReg[31:20]};
	assign Btype_ext = {{20{InstReg[31]}} , {InstReg[31], InstReg[7], InstReg[30:25], InstReg[11:6]}};
	assign Stype_ext = {{20{InstReg[31]}} , {InstReg[31:25], rd}};
	assign Utype_ext = {    InstReg[31:12], 12'd0};
	assign Jtype_ext = {{12{InstReg[31]}} , {InstReg[31], InstReg[19:12], InstReg[20], InstReg[30:21]}};
	assign imm_data = ({32{Ioprt | Iload | jalr}} & Ioprt_ext)
					| ({32{Btype}} & Btype_ext)
					| ({32{Stype}} & Stype_ext)
					| ({32{Utype}} & Utype_ext)
					| ({32{jal}} & Jtype_ext);

	/*
	PC
	*/
	assign Branch = Btype & (funct[0] ^~ ALU_zero);
	assign PC_plus4 = PC + 32'd4; 
	assign PC_tar = PC + imm_data;
	assign jalr_addr = {ALUReg[31:1] , 1'b0};
	assign PC_next = ({32{Branch | jal}} & PC_tar)
					|({32{jalr}} & jalr_addr)
					|({32{~Branch & ~jal & ~jalr}} & PC_plus4);

	always @(posedge clk) begin
		if(rst) 
			PC <= 32'd0; 
		else if(current_state == EX)
			PC <= PC_next;
	end

	/*
	store
	*/
	assign sb  = ~funct[1] & ~funct[0];
	assign sh  =  funct[0];
	assign sw  =  funct[1];

	assign addrtype[0] = ~ALUReg[1] & ~ALUReg[0];//2'b00;
	assign addrtype[1] = ~ALUReg[1] &  ALUReg[0];//2'b01;
	assign addrtype[2] =  ALUReg[1] & ~ALUReg[0];//2'b10;
	assign addrtype[3] =  ALUReg[1] &  ALUReg[0];//2'b11;

	assign Write_strb[3] = sw | sb & addrtype[3] | sh & addrtype[2] ;
	assign Write_strb[2] = sw | sb & addrtype[2] | sh & addrtype[2] ;
	assign Write_strb[1] = sw | sb & addrtype[1] | sh & addrtype[0] ;
	assign Write_strb[0] = addrtype[0] ;

	assign Write_data = ({32{sb}}  & {4{rdata2Reg[7:0]}})
					  | ({32{sh}}  & {2{rdata2Reg[15:0]}})
					  | ({32{sw}}  & rdata2Reg);

	/*
	load
	*/
	assign lbu =  funct[2] & sb;
	assign lhu =  funct[2] & sh;

	assign lb_data = ({32{addrtype[3]}} & {{24{Read_data[31]}}, Read_data[31:24]})
				   | ({32{addrtype[2]}} & {{24{Read_data[23]}}, Read_data[23:16]})
				   | ({32{addrtype[1]}} & {{24{Read_data[15]}}, Read_data[15:8]})
				   | ({32{addrtype[0]}} & {{24{Read_data[7]}} , Read_data[7:0]});
	assign lh_data = ({32{(addrtype[3] | addrtype[2])}} & {{16{Read_data[31]}}, Read_data[31:16]})
				   | ({32{(addrtype[1] | addrtype[0])}} & {{16{Read_data[15]}}, Read_data[15:0]});
	assign lw_data  =  Read_data[31:0];
	assign lbu_data = {24'b0, lb_data[7:0]};
	assign lhu_data = {16'b0, lh_data[15:0]};
	assign Load_data = ({32{sb}} & lb_data)
					 | ({32{sh}} & lh_data)
					 | ({32{sw}} & lw_data)
					 | ({32{lbu}} & lbu_data)
					 | ({32{lhu}} & lhu_data);

	/*
	data path
	*/
	assign raddr1 = rs1;
	assign raddr2 = rs2;
	assign RF_wen = current_state[5];
	assign RF_waddr = rd;
	assign Data_result = ({32{jal | jalr}}  & PC_plus4)
						|({32{auipc}}   & PC_tar)
						|({32{lui}}   & Utype_ext)
						|({32{shift}} & Shift_result)
						|({32{(Rtype | Ioprt) & ~shift}} & ALU_result);
	assign RF_wdata = Mem2Reg? MemReg: ResultReg;
	assign Address  = {ALUReg[31:2], 2'b00};

	/*
	alu control
	*/
	assign ALUop = ({3{Iload | Stype | jalr}} & 3'b000)//add
				 | ({3{Btype}} & {1'b0, funct[2], funct[2] ^~ funct[1]})
				 | ({3{Rtype}} & {funct[2:1], func7[5] | funct[2:1]})
				 | ({3{Ioprt}} & {funct[2:0]});
	assign ALU_A = Btype? rdata2Reg: rdata1Reg;
	assign ALU_B = Btype? rdata1Reg: ALUsrc? imm_data : rdata2Reg;

	/*
	shifter
	*/
	assign Shiftop = {func7[5],funct[2]};
	assign Shift_A = rdata1Reg;
	assign Shift_B = opcode[5] ? rdata2Reg : {27'b0,rs2};

	/*
	instantiation
	*/
	alu u_alu(
		.A(ALU_A),
		.B(ALU_B),
		.ALUop(ALUop),
		.Overflow(ALU_overflow),
		.CarryOut(ALU_carryout),
		.Zero(ALU_zero),
		.Result(ALU_result)
	);
	reg_file u_reg_file(
		.clk(clk),
		.rst(rst),
		.waddr(RF_waddr),
		.raddr1(raddr1),
		.raddr2(raddr2),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(rdata1),
		.rdata2(rdata2)
	);
	shifter u_shifter(
		.A(Shift_A),
		.B(Shift_B),
	    .Shiftop(Shiftop),
		.Result(Shift_result)
	);

endmodule