module imm_gen(input [31:0] inst, output signed [31:0] out, input [2:0] imm_sel); 
    reg [31:0] out_reg; 
    localparam i_type = 3'b000;
    localparam s_type = 3'b001; 
    localparam b_type = 3'b010; 
    localparam u_type = 3'b011;
    localparam j_type = 3'b100;
    localparam csr_rwi_type = 3'b101;
    localparam csr_rw_type = 3'b110; // outputs a zero because we add 0 to rs1 in the alu so we can just write rs1 to the register
    localparam shift_type = 3'b111;
    always @(*) begin 
        case(imm_sel) 
          i_type: out_reg = {{21{inst[31]}}, inst[30:20]}; 
          s_type: out_reg = {{21{inst[31]}}, inst[30:25], inst[11:7]}; 
          b_type: out_reg = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};
          u_type: out_reg = {inst[31:12], 12'd0}; 
          j_type: out_reg = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0}; 
          csr_rwi_type: out_reg = {27'd0, {inst[19:15]}};
          csr_rw_type: out_reg = 32'd0;
          shift_type: out_reg = {27'd0, {inst[24:20]}};
          default: out_reg = 32'd0;
          endcase 
    end 

    assign out = out_reg;

endmodule