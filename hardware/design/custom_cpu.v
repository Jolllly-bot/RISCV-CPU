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
	reg [8:0]     current_state;
	reg [8:0]     next_state;
	reg [31:0]    ALUReg;
	reg [31:0]    ResultReg;
	reg [31:0]    MemReg;
	reg [31:0]    InstReg;
	reg [31:0]    rdata1Reg;
	reg [31:0]    rdata2Reg;
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
	wire  [5:0]     Opcode;
	wire  [5:0]     Func;
	wire  [4:0]     rs;
	wire  [4:0]     rd;
	wire  [4:0]     rt;
	wire  [4:0]     sa;
	//type decode
	wire NOP;
	wire Rtype;
	wire REGIMM;
	wire Jtype;
	wire Ibranch;
	wire Ioprt;
	wire Iload;
	wire Istore;
	wire Ibeq;
	wire Iblez;
	wire op_shift;
	wire op_jump;
	wire op_mov;
	wire jumpal;
	wire jr;
	wire jal;
	wire lui;
	//control signals
	wire RegDst;
	wire Jump;
	wire Mem2Reg;
	wire ALUsrc;
	wire RegWrite;
	wire ALUop0;
	wire ALUop1;
	wire PCsrc;
	//sign extesion
	wire  [31:0]    sign_ext;
	wire  [31:0]    zero_ext;
	wire  [31:0]    shift_ext;
	wire  [31:0]    imm_data;
	//PC
	wire  [31:0]    PC_next;
	wire  [31:0]    PC_plus4;
	wire  [31:0]    PC_add;
	wire  [31:0]    PC_result;
	wire  [31:0]    PC_tar;
	wire  [31:0]    Jump_tar;
	wire  [31:0]    Jump_addr;
	//load & store
	wire 			sb;
	wire 			sh;
	wire 			sw;
	wire 			swl;
	wire 			swr;
	wire  [3:0]     addrtype;//one-hot code
	wire  [31:0]    swl_data;
	wire  [31:0]    swr_data;
	wire  			lbu;     //others are same as store
	wire  			lhu;
	wire  [31:0]    lb_data;
	wire  [31:0]    lh_data;
	wire  [31:0]    lw_data;
	wire  [31:0]    lbu_data;
	wire  [31:0]    lhu_data;
	wire  [31:0]    lwl_data;
	wire  [31:0]    lwr_data;
	wire  [31:0]    Load_data;
	//others
	wire  [31:0]    lui_data;
	wire  [31:0]    Data_result;
	wire 			mov_judge;//if (==1) rf_wen=0
	wire  [3:0]		func_m;
	wire  [3:0]		opcode_modified;
	//cnt define
	reg   [31:0]    cycle_cnt;
	reg   [31:0]    store_cnt;

	// **********************************

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
				if(NOP) next_state = IF;
				else next_state = EX;
			end
			EX : begin
				if(Iload) next_state = LD;
				else if(Istore) next_state = ST;
				else if(Rtype | Ioprt | jumpal) next_state = WB;
				else next_state = IF;
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
	assign Opcode = InstReg[31:26];
	assign rs     = InstReg[25:21];
	assign rt     = InstReg[20:16];
	assign rd     = InstReg[15:11];
	assign sa     = InstReg[10:6];
	assign Func   = InstReg[5:0];

	/*
	decoder
	*/
	assign NOP     = InstReg == 32'b0;
	assign Rtype   = (~Opcode[5] & ~Opcode[4]) & (~Opcode[3] & ~Opcode[2]) & (~Opcode[1] & ~Opcode[0]);//6'b000000
	assign REGIMM  = (~Opcode[5] & ~Opcode[4]) & (~Opcode[3] & ~Opcode[2]) & (~Opcode[1] & Opcode[0]);//6'b000001
	assign Jtype   = (~Opcode[5] & ~Opcode[4]) & (~Opcode[3] & ~Opcode[2]) &   Opcode[1];//5'b000001
	assign Ibranch = (~Opcode[5] & ~Opcode[4]) & (~Opcode[3] &  Opcode[2]);//4'b0001
	assign Ioprt   = (~Opcode[5] & ~Opcode[4]) &   Opcode[3];//3'b001
	assign Iload   = ( Opcode[5] & ~Opcode[4]) &  ~Opcode[3];//3'b100
	assign Istore  = ( Opcode[5] & ~Opcode[4]) &   Opcode[3];//3'b101

	assign Ibeq = Ibranch & ~Opcode[1];
	assign Iblez = Ibranch & Opcode[1];
	assign op_shift = Rtype & Func[5:3]==3'b000;
	assign op_jump = Rtype & {Func[5:3], Func[1]} == 4'b0010;
	assign op_mov = Rtype & {Func[5:3],Func[1]} == 4'b0011;
	assign jumpal = (op_jump & Func[0]) | (Jtype & Opcode[0]);
	assign jr = op_jump & ~Func[0];
	assign jal = Jtype & Opcode[0];
	assign lui = Ioprt & Opcode[2:0]==3'b111;

	/*
	control unit
	*/
	assign MemRead = current_state == LD;
	assign MemWrite = current_state == ST;
	assign RegDst = Rtype;
	assign Jump = Jtype | op_jump;
	assign ALUsrc = Iload | Istore | Ioprt;
	assign Mem2Reg = Iload;
	assign RegWrite = Rtype | Iload | Ioprt | jal;
	assign ALUop1 = Rtype | Ioprt | Iblez | REGIMM;
	assign ALUop0 = Ibranch | REGIMM ;
	assign PCsrc = (Ibranch & (Opcode[0] ^ ALU_zero)) | (REGIMM & (rt[0] ^~ ALU_zero));

	/*
	sign extension
	*/
	assign sign_ext  = {{16{InstReg[15]}}, InstReg[15:0]};
	assign zero_ext  = { 16'b0           , InstReg[15:0]};
	assign shift_ext = { sign_ext[29:0]  , 2'b00};
	assign imm_data = Opcode[2]? zero_ext : sign_ext;

	/*
	PC
	*/
	assign PC_plus4 = PC + 32'd4;
	assign PC_add = jumpal ? 32'd4 : shift_ext;
	assign PC_result = PC_plus4 + PC_add;
	assign Jump_tar = {PC_plus4[31:28],InstReg[25:0],2'b00};
	assign Jump_addr = op_jump? rdata1Reg : Jump_tar;
	assign PC_tar = PCsrc? PC_result : PC_plus4;
	assign PC_next = Jump? Jump_addr : PC_tar;

	always @(posedge clk) begin
		if(rst) PC<=32'd0; 
		else if(current_state == EX || current_state == ID && NOP)
			PC <= PC_next;
	end

	/*
	store
	*/
	assign sb  = ~Opcode[2] & ~Opcode[1] & ~Opcode[0];//Opcode[2:0]==3'b000;
	assign sh  = ~Opcode[2] & ~Opcode[1] &  Opcode[0];//3'b001;
	assign sw  = ~Opcode[2] &  Opcode[1] &  Opcode[0];//3'b011;
	assign swl = ~Opcode[2] &  Opcode[1] & ~Opcode[0];//3'b010;
	assign swr =  Opcode[2] &  Opcode[1] & ~Opcode[0];//3'b110;
	assign addrtype[0] = ~ALUReg[1] & ~ALUReg[0];//2'b00;
	assign addrtype[1] = ~ALUReg[1] &  ALUReg[0];//2'b01;
	assign addrtype[2] =  ALUReg[1] & ~ALUReg[0];//2'b10;
	assign addrtype[3] =  ALUReg[1] &  ALUReg[0];//2'b11;

	assign Write_strb[3] = sw | sb & addrtype[3] | sh & addrtype[2] | swl &  addrtype[3] 				 | swr;
	assign Write_strb[2] = sw | sb & addrtype[2] | sh & addrtype[2] | swl & (addrtype[3] | addrtype[2])  | swr & ~addrtype[3];
	assign Write_strb[1] = sw | sb & addrtype[1] | sh & addrtype[0] | swl & ~addrtype[0] 				 | swr & (addrtype[0] | addrtype[1]);
	assign Write_strb[0] = addrtype[0] | swl;

	assign swr_data = ({32{addrtype[3]}} & {rdata2Reg[ 7:0],24'd0})
				    | ({32{addrtype[2]}} & {rdata2Reg[15:0],16'd0})
				    | ({32{addrtype[1]}} & {rdata2Reg[23:0], 8'd0})
				    | ({32{addrtype[0]}} & rdata2Reg);
	assign swl_data = ({32{addrtype[3]}} & rdata2Reg)
				    | ({32{addrtype[2]}} & { 8'd0,rdata2Reg[31:8]})
				    | ({32{addrtype[1]}} & {16'd0,rdata2Reg[31:16]})
				    | ({32{addrtype[0]}} & {24'd0,rdata2Reg[31:24]});
	assign Write_data = ({32{sb}}  & {4{rdata2Reg[7:0]}})
					  | ({32{sh}}  & {2{rdata2Reg[15:0]}})
					  | ({32{sw}}  & rdata2Reg)
					  | ({32{swl}} & swl_data)
					  | ({32{swr}} & swr_data);

	/*
	load
	*/
	assign lbu =  Opcode[2] & ~Opcode[1] & ~Opcode[0];//Opcode[2:0]==3'b100;
	assign lhu =  Opcode[2] & ~Opcode[1] &  Opcode[0];//Opcode[2:0]==3'b101;

	assign lb_data = ({32{addrtype[3]}} & {{24{Read_data[31]}}, Read_data[31:24]})
				   | ({32{addrtype[2]}} & {{24{Read_data[23]}}, Read_data[23:16]})
				   | ({32{addrtype[1]}} & {{24{Read_data[15]}}, Read_data[15:8]})
				   | ({32{addrtype[0]}} & {{24{Read_data[7]}} , Read_data[7:0]});
	assign lh_data = ({32{(addrtype[3] | addrtype[2])}} & {{16{Read_data[31]}}, Read_data[31:16]})
				   | ({32{(addrtype[1] | addrtype[0])}} & {{16{Read_data[15]}}, Read_data[15:0]});
	assign lw_data  =  Read_data[31:0];
	assign lbu_data = {24'b0, lb_data[7:0]};
	assign lhu_data = {16'b0, lh_data[15:0]};
	assign lwl_data =  ({32{addrtype[3]}} &  Read_data[31:0])
					 | ({32{addrtype[2]}} & {Read_data[23:0],  rdata2Reg[7:0]})
					 | ({32{addrtype[1]}} & {Read_data[15:0],  rdata2Reg[15:0]})
					 | ({32{addrtype[0]}} & {Read_data[7:0] ,  rdata2Reg[23:0]});
	assign lwr_data = ({32{addrtype[3]}} & {rdata2Reg[31:8] , Read_data[31:24]})
					| ({32{addrtype[2]}} & {rdata2Reg[31:16], Read_data[31:16]})
					| ({32{addrtype[1]}} & {rdata2Reg[31:24], Read_data[31:8]})
					| ({32{addrtype[0]}} &  Read_data[31:0]);
	assign Load_data = ({32{sb}} & lb_data)
					 | ({32{sh}} & lh_data)
					 | ({32{sw}} & lw_data)
					 | ({32{lbu}} & lbu_data)
					 | ({32{lhu}} & lhu_data)
					 | ({32{swl}} & lwl_data)
				     | ({32{swr}} & lwr_data);

	/*
	data path
	*/
	assign mov_judge = op_mov & (Func[0] ^~ rdata2Reg==0);
	assign lui_data = {InstReg[15:0],16'd0};
	assign raddr1 = rs;
	assign raddr2 = REGIMM? 0:rt;
	assign RF_wen = current_state == WB & ((jr | mov_judge)? 0:RegWrite);
	assign RF_waddr = jal? 6'd31 :RegDst? rd:rt;
	assign Data_result = ({32{jumpal}}   & PC_result)
						|({32{lui}}      & lui_data)
						|({32{op_mov}}   & rdata1Reg)
						|({32{op_shift}} & Shift_result)
						|({32{~jumpal & ~lui & ~op_mov & ~op_shift}} & ALU_result);
	assign RF_wdata = Mem2Reg? MemReg: ResultReg;
	assign Address  = {ALUReg[31:2], 2'b00};

	/*
	alu control
	*/
	assign opcode_modified = (Opcode[2:1]==2'b01)? Opcode[3:0] : {1'b0,Opcode[2:0]};
	assign func_m = ({4{Rtype}} & InstReg [3:0]) | ({4{Ioprt}} & opcode_modified);
	assign op2 = (~func_m[3] & func_m[1]) | (func_m[1] & ~func_m[0]);
	assign op1 = ~func_m[2];
	assign op0 = (func_m[2] & func_m[0]) | func_m[3];
	assign ALUop[2] = ALUop0 | (op2 & ALUop1);
	assign ALUop[1] = ~ALUop1 | ALUop0 | op1;
	assign ALUop[0] = (ALUop1 & ALUop0) | (ALUop1 & ~ALUop0 & op0);

	assign ALU_A = Iblez? rdata2Reg: op_mov? 32'b0 : rdata1Reg;
	assign ALU_B = Iblez? rdata1Reg: ALUsrc? imm_data : rdata2Reg;

	/*
	shifter
	*/
	assign Shiftop = Func[1:0];
	assign Shift_A = rdata2Reg;
	assign Shift_B = Func[2]? rdata1Reg : {27'b0,sa};

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