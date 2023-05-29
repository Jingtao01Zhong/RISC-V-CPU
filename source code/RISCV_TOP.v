module RISCV_TOP (
	//General Signals
	input wire CLK,
	input wire RSTn,

	//I-Memory Signals
	output wire I_MEM_CSN,
	input wire [31:0] I_MEM_DI,//input from IM
	output reg [11:0] I_MEM_ADDR,//in byte address

	//D-Memory Signals
	output wire D_MEM_CSN,
	input wire [31:0] D_MEM_DI,
	output wire [31:0] D_MEM_DOUT,
	output wire [11:0] D_MEM_ADDR,//in word address
	output wire D_MEM_WEN,
	output wire [3:0] D_MEM_BE,

	//RegFile Signals
	output wire RF_WE,
	output wire [4:0] RF_RA1,
	output wire [4:0] RF_RA2,
	output wire [4:0] RF_WA1,
	input wire [31:0] RF_RD1,
	input wire [31:0] RF_RD2,
	output wire [31:0] RF_WD,
	output wire HALT,
	output reg [31:0] NUM_INST,
	output wire [31:0] OUTPUT_PORT
	);
	// TODO: implement multi-cycle CPU
	reg [3:0] pipe_state; 
	reg [11:0] PC_nxt;
	reg [11:0] PC_4;
	reg [11:0] PC_IF, PC_ID, PC_EX, PC_MEM, PC_WB, PC_Branch;
	wire [1:0] PC_Source;
	wire PC_enable;
	wire IR_Write;
	reg [31:0] Ins_IF, Ins_ID, Ins_EX;
	wire [31:0] Imm_ID;
	reg [31:0] Imm_EX;
	wire ALUSrc;
	wire [3:0] ALU_Control;
	wire RegWrite;
	wire MemWrite;
	wire [1:0] MemtoReg;
	reg [31:0] A, B;
	reg [31:0] A_EX, B_EX, num_1, num_2, num_2_nxt, MEM_Data, result_MEM;
	wire [31:0] result_EX;
	wire Branch_EX;
	reg Branch_MEM, Branch_WB;
	reg [31:0] data_WB_1;
	reg [31:0] data_WB_2;
	reg WB_done;
	reg [4:0] Rd_ID, Rd_EX, Rd_MEM, Rd_WB;
	reg [4:0] Rs1_ID, Rs1_EX, Rs2_ID, Rs2_EX;
	reg [31:0] Reg_WD;
	wire [1:0] ForwardA, ForwardB;
	wire ForwardA_ID, ForwardB_ID;
	wire stall_ID, stall_MEM;
	reg HALT_nxt;
	reg HALT_flag_1_IF, HALT_flag_1_ID, HALT_flag_1_EX, HALT_flag_1_MEM, HALT_flag_1_WB;
	reg HALT_flag_2_IF, HALT_flag_2_ID, HALT_flag_2_EX, HALT_flag_2_MEM, HALT_flag_2_WB;
	
	initial begin
		NUM_INST <= 0;
		PC_IF <= -4;
		PC_nxt <= 0;
		MEM_Data <= 0;
	end

//////////////////////////////////////////////////// IF stage
	always @(*) begin
		if(PC_Source == 2'b00)
			PC_nxt = PC_4;
		else if(PC_Source == 2'b01)
			PC_nxt = PC_Branch;
		else if(PC_Source == 2'b10)
			PC_nxt = result_EX[11:0];
	end
	always @(*) begin
		PC_4 = PC_IF + 4;
	end	

	always @(posedge CLK) begin
		if(RSTn == 1) begin
			if(PC_enable == 1) begin
				PC_IF <= PC_nxt;
				I_MEM_ADDR <= PC_nxt;
				pipe_state[0] <= 1;
			end
			else begin
				I_MEM_ADDR <= PC_IF;
				pipe_state[0] <= 1;
			end
		end
		else begin
			pipe_state[0] <= 0;
			HALT_flag_1_IF <= 0;
			HALT_flag_2_IF <= 0;
		end
	end

	assign I_MEM_CSN = ~RSTn; // write instruction to the instruction register(Ins_IF)
	always @(*) begin
		if(RSTn & IR_Write) begin
			Ins_IF = I_MEM_DI;
		end
	end

	always @(*) begin
		if(I_MEM_DI == 32'h00c00093)
			HALT_flag_1_IF = 1;
		else HALT_flag_1_IF = 0;

		if(I_MEM_DI == 32'h00008067 & HALT_flag_1_ID)
			HALT_flag_2_IF = 1;
		else HALT_flag_2_IF = 0;
	end

//////////////////////////////////////////////////////   ID stage
	always @(posedge CLK) begin
		if(pipe_state[0]) begin
			Ins_ID <= Ins_IF;
			HALT_flag_1_ID <= HALT_flag_1_IF;
			HALT_flag_2_ID <= HALT_flag_2_IF;
			pipe_state[1] <= 1;
		end
		else pipe_state[1] <= 0;
	end

	always @(posedge CLK) begin
		if(pipe_state[0] & ~stall_ID)
			PC_ID <= PC_IF;
	end

	assign RF_WE = RegWrite;
	assign RF_RA1 = Ins_ID[19:15]; //decode the instruction
	assign RF_RA2 = Ins_ID[24:20];

	always @(*) begin
		if(ForwardA_ID)
			A = Reg_WD;
		else A = RF_RD1;
		
		if(ForwardB_ID)
			B = Reg_WD;
		else B = RF_RD2;
	end

	always @(*) begin
		Rs1_ID = Ins_ID[19:15];
		Rs2_ID = Ins_ID[24:20];
		Rd_ID = Ins_ID[11:7];
	end

	pipe_control pipe_control(
		.CLK (CLK),
		.pipe_state (pipe_state),
		.opcode (Ins_ID[6:0]),
		.funct ({Ins_ID[30],Ins_ID[14:12]}),
		.cum_funct ({Ins_ID[26:25],Ins_ID[14:12]}),
		.branch_taken (Branch_EX),
		.Rd_ID (Rd_ID),
		.Rs1_ID (Rs1_ID),
		.Rs2_ID (Rs2_ID),
		.PC_enable (PC_enable),
		.IR_Write (IR_Write),
		.RegWrite (RegWrite),
		.ALUSrc (ALUSrc),
		.ALU_Control (ALU_Control),
		.PC_Source (PC_Source),
		.MemWrite (MemWrite),
		.MemtoReg (MemtoReg),
		.ForwardA (ForwardA),
		.ForwardB (ForwardB),
		.ForwardA_ID (ForwardA_ID),
		.ForwardB_ID (ForwardB_ID),
		.o_stall_MEM (stall_MEM),
		.o_stall_ID (stall_ID)
	);
	
	ImmGen ImmGen(
		.Ins (Ins_ID),
		.Imm_out (Imm_ID)
	);

/////////////////////////////////////////////////////// EX stage
	always @(posedge CLK) begin
		if(pipe_state[1]) begin
			A_EX <= A;
			B_EX <= B;
			Imm_EX <= Imm_ID;
			Rd_EX <= Rd_ID;
			Rs1_EX <= Rs1_ID;
			Rs2_EX <= Rs2_ID;
			HALT_flag_1_EX <= HALT_flag_1_ID;
			HALT_flag_2_EX <= HALT_flag_2_ID;
			Ins_EX <= Ins_ID;
			pipe_state[2] <= 1;
		end
		else begin
			pipe_state[2] <= 0;
		end
	end
	
	always @(posedge CLK) begin
		if(pipe_state[1] & ~stall_ID)
			PC_EX <= PC_ID;
	end
 
	always @(*) begin
		PC_Branch = PC_EX + Imm_EX[11:0]; 
	end	
	
	always @(*) begin
		if(ForwardA == 2'b01)
			num_1 = Reg_WD;
		else if(ForwardA == 2'b10)
			num_1 = result_MEM;
		else num_1 = A_EX;
	end

	always @(*) begin
		if(ALUSrc)
			num_2 = Imm_EX;
		else num_2 = num_2_nxt;
	end
	always @(*) begin
		if(ForwardB == 2'b01)
			num_2_nxt = Reg_WD;
		else if(ForwardB == 2'b10)
			num_2_nxt = result_MEM;
 		else num_2_nxt = B_EX;
	end

	ALU ALU(
		.A (num_1),
		.B (num_2),
		.OP (ALU_Control),
		.C (result_EX),
		.Branch (Branch_EX)
	);
	
/////////////////////////////////////////////////////////////// MEM stage
	always @(posedge CLK) begin
		if(pipe_state[2]) begin
			result_MEM <= result_EX;
			Branch_MEM <= Branch_EX;
			MEM_Data <= num_2_nxt;
			Rd_MEM <= Rd_EX;
			PC_MEM <= PC_EX;
			HALT_flag_1_MEM <= HALT_flag_1_EX;
			HALT_flag_2_MEM <= HALT_flag_2_EX;
			pipe_state[3] <= 1;
		end
		else pipe_state[3] <= 0;
	end
	assign D_MEM_WEN = ~MemWrite;
	assign D_MEM_BE = 4'b1111;
	assign D_MEM_CSN = ~RSTn;
	assign D_MEM_ADDR = result_MEM[11:0];
	assign D_MEM_DOUT = MEM_Data;
	
//////////////////////////////////////////////////////////////  WB stage
	always @(posedge CLK) begin
		if(pipe_state[3] & ~stall_MEM) begin
			PC_WB <= PC_MEM;
			data_WB_1 <= D_MEM_DI;
			data_WB_2 <= result_MEM;
			Rd_WB <= Rd_MEM;
			Branch_WB <= Branch_MEM;
			HALT_flag_1_WB <= HALT_flag_1_MEM;
			HALT_flag_2_WB <= HALT_flag_2_MEM;
			WB_done <= 1;
		end
		else WB_done <= 0;
	end
	assign RF_WA1 = Rd_WB;
	always @(*) begin
		if(MemtoReg == 2'b11)     // memory to register
			Reg_WD = data_WB_1;
		else if(MemtoReg == 2'b00)
			Reg_WD = data_WB_2; // ALU result to register
		else if(MemtoReg == 2'b01)
			Reg_WD = {{31{1'b0}},Branch_WB}; //Branch taken to register
		else if(MemtoReg == 2'b10)
			Reg_WD = PC_WB + 4;
	end
	assign RF_WD = Reg_WD;
	assign OUTPUT_PORT = RF_WD;
	always @(negedge CLK) begin
		if (RSTn & WB_done) begin
			NUM_INST <= NUM_INST + 1;
		end
	end

//////////////////////////////////////////////////////////////////////  HALT
	assign HALT = HALT_nxt;
	always @(*) begin
		if(HALT_flag_1_WB & HALT_flag_2_MEM)
			HALT_nxt = 1;
		else HALT_nxt = 0;
	end

	always @(negedge CLK) begin
		if(PC_IF == 20) begin
			$display("num_1:%d, num_2:%d, ALU_Control:%d",num_1, num_2, ALU_Control);
		end
	end
endmodule //

module ImmGen(
	input [31:0] Ins,
	output reg [31:0] Imm_out
);
	wire [6:0] opcode;
	wire [2:0] funct;
	assign opcode = Ins[6:0];
	assign funct = Ins[14:12];
	always @(*) begin
		if(opcode == 7'b0000011 | opcode == 7'b1100111) //Load and JALR
			Imm_out = {{20{Ins[31]}},Ins[31:20]};
		else if(opcode == 7'b0010011) begin
			if(funct == 3'b001 | funct == 3'b101)
				Imm_out = {{27{Ins[24]}},Ins[24:20]};
			else Imm_out = {{20{Ins[31]}},Ins[31:20]};
		end
		else if(opcode == 7'b0100011) //S-type
			Imm_out = {{20{Ins[31]}},Ins[31:25],Ins[11:7]};
		else if(opcode == 7'b1100011) //B-type
			Imm_out = {{20{Ins[31]}},Ins[7],Ins[30:25],Ins[11:8],1'b0};//shifting left 1 is done
		else if(opcode == 7'b1101111) //JAL
			Imm_out = {{12{Ins[31]}},Ins[19:12],Ins[20],Ins[30:21],1'b0};//shifting left 1 is done
		else Imm_out = 0;
	end
endmodule

module ALU(A,B,OP,C,Branch);
	input [31:0] A;
	input [31:0] B;
	input [3:0] OP;
	output reg [31:0]C;
	output reg Branch;
       always @(*) begin
		case(OP[3:0])
			4'b0000: C = A + B;  // ADD
			4'b1000: begin
				C = A - B; //SUB
				if(C == 0)
					Branch = 1;
				else Branch = 0;
			end
			4'b0001: C = A << B[4:0];    //SLL
			4'b0010: begin     //SLT
				if($signed(A) < $signed(B)) begin
					C = 1;
					Branch = 1;
				end
				else begin 
					C = 0;
					Branch = 0;
				end
			end
			4'b0011: begin     //SLTU
				if(A < B) begin
					C = 1;
					Branch = 1;
				end
				else begin
					C = 0;
					Branch = 0;
				end
			end
			4'b0100: C = A ^ B; //XOR
			4'b0101: C = A >> B[4:0];      //SRL
			4'b1101: C = A >>> B[4:0];   // SRA
			4'b0110: C = A | B;   // OR
			4'b0111: C = A & B;   // AND
			4'b1001: begin //BNE if A != B , C == 1
				if(A != B) begin
					C = 1;
					Branch = 1;
				end
				else begin
					C = 0;
					Branch = 0;
				end
			end
			4'b1010: begin  // BGE if A>= B zero = 1
				if($signed(A) >= $signed(B)) begin
					C = 1;
					Branch = 1;
				end
				else begin 
					C = 0;
					Branch = 0;
				end
			end
			4'b1111: begin   //BEGU if A>B zero = 1
				if(A >= B) begin
					C = 1;
					Branch = 1;
				end
				else begin
					C = 0;
					Branch = 0;
				end
			end
			4'b1110: C = A * B;
			4'b1100: C = A % B;
			4'b1011: begin
				if(A[0] == 1'b0)
					C = 1;
				else C = 0;
			end
		endcase
	end
endmodule