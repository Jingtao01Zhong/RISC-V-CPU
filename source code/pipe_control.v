module pipe_control (
	input CLK,
	input [3:0] pipe_state,
	input [6:0] opcode,
	input wire [3:0] funct,
	input wire [4:0] cum_funct,
	input wire branch_taken,
	input [4:0] Rd_ID,
	input [4:0] Rs1_ID,
	input [4:0] Rs2_ID,

	output wire PC_enable, 
	output wire IR_Write,  
	output wire RegWrite,
	output wire ALUSrc,
	output reg [3:0] ALU_Control,
	output reg [1:0] PC_Source,
	output wire MemWrite,
	output wire [1:0] MemtoReg,
	output wire [1:0] ForwardA,
	output wire [1:0] ForwardB,
	output wire ForwardA_ID,
	output wire ForwardB_ID,
	output wire o_stall_MEM,
	output wire o_stall_ID
);

	reg RegWrite_ID, RegWrite_EX, RegWrite_MEM, RegWrite_WB;
	reg ALUSrc_ID, ALUSrc_EX;
	reg [2:0] ALUOp_ID, ALUOp_EX;
	wire [2:0] ALUOp;
	reg [1:0] PC_Source_ID, PC_Source_EX, PC_Source_MEM;
	reg MemWrite_ID, MemWrite_EX, MemWrite_MEM;
	reg [1:0] MemtoReg_ID, MemtoReg_EX, MemtoReg_MEM, MemtoReg_WB;
	reg MemRead_EX, MemRead_ID;
	wire stall_ID;
	reg stall_EX, stall_MEM, stall_WB;
	wire [1:0] Control_enable;
	reg [3:0] funct_nxt;
	reg [4:0] cum_funct_nxt;
	reg Jump_ID, Jump_EX, Jump_nxt_ID, Jump_nxt_EX;
	reg Branch_ID, Branch_EX;	
	reg [4:0] Rs1_ID_nxt, Rs1_EX, Rs2_ID_nxt, Rs2_EX;
	reg [4:0] Rd_ID_nxt, Rd_EX, Rd_MEM, Rd_WB;
	
	initial begin
		MemRead_EX <= 0;
		PC_Source_EX <= 0;
		Jump_EX <= 0;
		Jump_nxt_EX <= 0;
		RegWrite_WB <= 0;
	end

	assign ALUOp = ALUOp_EX;
	assign RegWrite = RegWrite_WB;
	assign ALUSrc = ALUSrc_EX;
	assign MemWrite = MemWrite_MEM;
	assign MemtoReg = MemtoReg_WB;
	assign o_stall_MEM = stall_MEM;
	assign o_stall_ID = stall_ID;
	always @(*) begin
		if(Branch_EX) begin
			if(branch_taken)
				PC_Source = PC_Source_EX;
			else PC_Source = 2'b00;
		end
		else begin
			PC_Source = PC_Source_EX;
		end
	end

	always @(*) begin //decode the ALUOp and funct, generate ALU_Control
		if(ALUOp[2:0] == 3'b010) //R-type
			ALU_Control[3:0] = funct_nxt[3:0];
		else if(ALUOp[2:0] == 3'b011) //I-type
			ALU_Control[3:0] = {1'b0,funct_nxt[2:0]};
		else if(ALUOp[2:0] == 3'b000) //L-type and S-type and JALR
			ALU_Control[3:0] = 4'b0000; // -->ADD
		else if(ALUOp[2:0] == 3'b001) begin//B-type
			if(funct_nxt[2:0] == 3'b001)
				ALU_Control[3:0] = 4'b1000; //BEQ --> SUB1
			else if(funct_nxt[2:0] == 3'b001)
				ALU_Control[3:0] = 4'b1001; //BNE --> SUB2
			else if(funct_nxt[2:0] == 3'b100)
				ALU_Control[3:0] = 4'b0010; //BLT --> SLT
			else if(funct_nxt[2:0] == 3'b101)
				ALU_Control[3:0] = 4'b1010; // BGE 
			else if(funct_nxt[2:0] == 3'b110)
				ALU_Control[3:0] = 4'b0011; // BLTU --> SLTU
			else if(funct_nxt[2:0] == 3'b111)
				ALU_Control[3:0] = 4'b1111; //BGEU
		end
		else if(ALUOp[2:0] == 3'b100) begin
			if(cum_funct_nxt[4:0] == 5'b00111)
				ALU_Control[3:0] = 4'b1110;
			else if(cum_funct_nxt[4:0] == 5'b01111)
				ALU_Control[3:0] = 4'b1100;
			else if(cum_funct_nxt[4:0] == 5'b10110)
				ALU_Control[3:0] = 4'b1011;
		end
	end

	always @(*) begin
		if(Control_enable == 2'b11) begin
			//stall_ID = 0;
			Rs1_ID_nxt = Rs1_ID;
			Rs2_ID_nxt = Rs2_ID;
			Rd_ID_nxt = Rd_ID;
			Jump_nxt_ID = 0;
			if(opcode == 7'b0110011) begin//R-type
				ALUOp_ID = 3'b010;
				PC_Source_ID  = 2'b00;
				RegWrite_ID  = 1;
				ALUSrc_ID  = 0;
				MemtoReg_ID  = 2'b00;
				MemWrite_ID  = 0;
				MemRead_ID = 0;
				Jump_ID = 0;
				Branch_ID = 0;
			end
			else if(opcode == 7'b0010011) begin //I-type
				PC_Source_ID  = 2'b00;
				RegWrite_ID  = 1;
				ALUSrc_ID  = 1;
				MemtoReg_ID  = 2'b00;
				MemWrite_ID  = 0;
				MemRead_ID = 0;
				Jump_ID = 0;
				Branch_ID = 0;
				if(funct[2:0] == 3'b001 | funct[2:0] == 3'b101)
					ALUOp_ID  = 3'b010;
				else ALUOp_ID = 3'b011;
			end
			else if(opcode == 7'b0001011) begin //custom type
				PC_Source_ID  = 2'b00;
				ALUSrc_ID  = 0;
				MemWrite_ID  = 0;
				MemtoReg_ID  = 2'b00;
				ALUOp_ID  = 3'b100;
				RegWrite_ID = 1;
				MemRead_ID = 0;
				Jump_ID = 0;
				Branch_ID = 0;
			end
			else if(opcode == 7'b1101111) begin //JAL
				PC_Source_ID  = 2'b01;
				RegWrite_ID  = 1;
				ALUSrc_ID  = 0;
				MemWrite_ID  = 0;
				MemtoReg_ID  = 2'b10;
				ALUOp_ID  = 3'b000;
				MemRead_ID = 0;
				Jump_ID = 1;
				Branch_ID = 0;
			end
			else if(opcode == 7'b1100111) begin //JALR
				PC_Source_ID  = 2'b10;
				RegWrite_ID  = 1;
				ALUSrc_ID  = 1;  // Reg + Imm
				MemWrite_ID  = 0;
				MemtoReg_ID  = 2'b10;
				ALUOp_ID  = 3'b000;
				MemRead_ID = 0;
				Jump_ID = 1;
				Branch_ID = 0;
			end
			else if(opcode == 7'b0000011) begin //Load
				PC_Source_ID  = 2'b00;
				RegWrite_ID  = 1;
				ALUSrc_ID  = 1;
				MemWrite_ID  = 0;
				MemtoReg_ID  = 2'b11;
				ALUOp_ID  = 3'b000;
				MemRead_ID = 1;
				Jump_ID = 0;
				Branch_ID = 0;
			end
			else if(opcode == 7'b0100011) begin //store
				PC_Source_ID  = 2'b00;
				RegWrite_ID  = 0;
				ALUSrc_ID  = 1;
				MemWrite_ID  = 1;
				MemtoReg_ID  = 2'b00;
				ALUOp_ID  = 3'b000;
				MemRead_ID = 0;
				Jump_ID = 0;
				Branch_ID = 0;
			end
			else if(opcode == 7'b1100011) begin//Branch
				PC_Source_ID  = 2'b01;  // PC + Imm
				RegWrite_ID  = 0;
				ALUSrc_ID  = 0;
				MemWrite_ID  = 0;
				MemtoReg_ID  = 2'b01;
				ALUOp_ID  = 3'b001;
				MemRead_ID = 0;
				Jump_ID = 0;
				Branch_ID = 1;
			end
		end
		else if(Control_enable == 2'b00) begin
			PC_Source_ID  = 2'b00;
			RegWrite_ID  = 1'bx;
			ALUSrc_ID  = 1'bx;
			MemWrite_ID  = 1'b0;
			MemRead_ID = 0;
			MemtoReg_ID  = 2'bxx;
			ALUOp_ID  = 3'bxxx;
			//stall_ID = 1;
			Jump_ID = 0;
			Jump_nxt_ID = 0;
			Branch_ID = 0;
			Rs1_ID_nxt = 5'b00000;
			Rs2_ID_nxt = 5'b00000;
			Rd_ID_nxt = 5'b00000;
		end
		else if(Control_enable == 2'b01) begin
			PC_Source_ID  = 2'b00;
			RegWrite_ID  = 1'bx;
			ALUSrc_ID  = 1'bx;
			MemWrite_ID  = 1'b0;
			MemRead_ID = 0;
			MemtoReg_ID  = 2'bxx;
			ALUOp_ID  = 3'bxxx;
			Jump_ID = 0;
			//stall_ID = 1;
			Jump_nxt_ID = 1;
			Branch_ID = 0;
			Rs1_ID_nxt = 5'b00000;
			Rs2_ID_nxt = 5'b00000;
			Rd_ID_nxt = 5'b00000;
		end
	end
	
	always @(posedge CLK) begin
		if(pipe_state[1]) begin
			PC_Source_EX <= PC_Source_ID;
			RegWrite_EX <= RegWrite_ID; 
			ALUSrc_EX <= ALUSrc_ID;
			MemWrite_EX <= MemWrite_ID;
			MemtoReg_EX <= MemtoReg_ID;
			ALUOp_EX <= ALUOp_ID;
			MemRead_EX <= MemRead_ID;
			stall_EX <= stall_ID;
			funct_nxt <= funct;
			cum_funct_nxt <= cum_funct;
			Jump_EX <= Jump_ID;
			Jump_nxt_EX <= Jump_nxt_ID;
			Rs1_EX <= Rs1_ID_nxt;
			Rs2_EX <= Rs2_ID_nxt;
			Rd_EX <= Rd_ID_nxt;
			Branch_EX <= Branch_ID;
		end
	end

	always @(posedge CLK) begin
		if(pipe_state[2]) begin
			PC_Source_MEM <= PC_Source_EX;
			RegWrite_MEM <= RegWrite_EX; 
			MemWrite_MEM <= MemWrite_EX;
			MemtoReg_MEM <= MemtoReg_EX;
			stall_MEM <= stall_EX;
			Rd_MEM <= Rd_EX;
		end
	end

	always @(posedge CLK) begin
		if(pipe_state[3]) begin
			MemtoReg_WB <= MemtoReg_MEM;
			RegWrite_WB <= RegWrite_MEM;
			stall_WB <= stall_MEM;
			Rd_WB <= Rd_MEM;
		end
	end

	Hazard_Detection Harzard_Detection(
		.MemRead_EX (MemRead_EX),
		.Rd_EX (Rd_EX),
		.Rs1_ID (Rs1_ID),
		.Rs2_ID (Rs2_ID),
		.Jump_EX (Jump_EX),
		.Jump_nxt_EX (Jump_nxt_EX),
		.Branch_EX (Branch_EX),
		.branch_taken (branch_taken),
		.PCWrite (PC_enable),
		.IR_Write (IR_Write),
		.Control_enable (Control_enable),
		.stall_ID (stall_ID)
	);
	
	Forwarding_Unit Forwarding_Unit(
		.Rd_WB (Rd_WB),
		.Rd_MEM (Rd_MEM),
		.Rs1_EX (Rs1_EX),
		.Rs2_EX (Rs2_EX),
		.Rs1_ID (Rs1_ID),
		.Rs2_ID (Rs2_ID),
		.RegWrite_WB (RegWrite_WB),
		.RegWrite_MEM (RegWrite_MEM),
		.ForwardA (ForwardA),
		.ForwardB (ForwardB),
		.ForwardA_ID (ForwardA_ID),
		.ForwardB_ID (ForwardB_ID)
	);

endmodule

module Hazard_Detection(
	input MemRead_EX,
	input [4:0] Rd_EX,
	input [4:0] Rs1_ID,
	input [4:0] Rs2_ID,
	input Jump_EX,
	input Jump_nxt_EX,
	input Branch_EX,
	input branch_taken,
	output reg [1:0] Control_enable,
	output reg PCWrite,
	output reg IR_Write,
	output reg stall_ID
);
	always @(*) begin
		if(MemRead_EX & ((Rd_EX == Rs1_ID) | (Rd_EX == Rs2_ID))) begin // if load instruction face the data harzard, stall for 1 cycle
			PCWrite = 0;
			IR_Write = 0;
			Control_enable = 2'b00;
			stall_ID = 1;
		end
		else if(Jump_EX) begin // Jump instruction: stall for 2 cycles
			PCWrite = 1;   // This part is used to enable the first stall cycle
			IR_Write = 1;
			Control_enable = 2'b01;
			stall_ID = 1;
		end
		else if(Jump_nxt_EX) begin  // after the first stall cycle, continue the second stall cycle (for branch taken and jump)
			PCWrite = 1;
			IR_Write = 1;
			Control_enable = 2'b00;
			stall_ID = 1;
		end
		else if(branch_taken & Branch_EX) begin // always-not-taken branch perdictor
			PCWrite = 1;                    // if branch is taken, stall for 2 cycles
			IR_Write = 1;                   // if branch is taken, stall will be started when the branch instruction is in the EX stage, the following 2 instructions will be discarded
			Control_enable = 2'b01;
			stall_ID = 1;
		end
		else begin
			PCWrite = 1;                 // if branch is not taken, or for the normal instructions, execute the next instruction(PC+4)
			IR_Write = 1;                // since the 2 instruction that behind the branch instruction is already in the IF stage and ID stage, there will be no stall if branch is not taken
			Control_enable = 2'b11;
			stall_ID = 0;
		end
	end
endmodule

module Forwarding_Unit(    // Forwarding Unit
	input [4:0] Rd_WB,
	input [4:0] Rd_MEM,
	input [4:0] Rs1_ID,
	input [4:0] Rs2_ID,
	input [4:0] Rs1_EX,
	input [4:0] Rs2_EX,
	input RegWrite_WB,
	input RegWrite_MEM,
	output reg [1:0] ForwardA,
	output reg [1:0] ForwardB,
	output reg ForwardA_ID,
	output reg ForwardB_ID
);
	always @(*) begin
		if(RegWrite_MEM & (Rd_MEM != 5'b00000) & (Rd_MEM == Rs1_EX))
			ForwardA = 2'b10;
		else if(RegWrite_WB & (Rd_WB != 5'b00000) & ~(RegWrite_MEM & (Rd_MEM != 5'b00000) & (Rd_MEM == Rs1_EX)) & (Rd_WB == Rs1_EX))
			ForwardA = 2'b01;
		else ForwardA = 2'b00;

		if(RegWrite_MEM & (Rd_MEM != 5'b00000) & (Rd_MEM == Rs2_EX))
			ForwardB = 2'b10;
		else if(RegWrite_WB & (Rd_WB != 5'b00000) & ~(RegWrite_MEM & (Rd_MEM != 5'b00000) & (Rd_MEM == Rs2_EX)) & (Rd_WB == Rs2_EX))
			ForwardB = 2'b01;
		else ForwardB = 2'b00;
	
		if(RegWrite_WB & (Rd_WB != 5'b00000) & (Rd_WB == Rs1_ID))
			ForwardA_ID = 1;
		else ForwardA_ID = 0;
		
		if(RegWrite_WB & (Rd_WB != 5'b00000) & (Rd_WB == Rs2_ID))
			ForwardB_ID = 1;
		else ForwardB_ID = 0;	
	end
endmodule
