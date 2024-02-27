module alu(input [31:0] a, input[3:0] alu_sel, input [31:0] b, output [31:0] out);
  reg [31:0] out_reg; 
  localparam add_alu = 4'b0000;
  localparam sll_alu = 4'b0001;
  localparam slt_alu = 4'b0010; 
  localparam xor_alu = 4'b0100;
  localparam srl_alu = 4'b0101;
  localparam or_alu = 4'b0110;
  localparam and_alu = 4'b0111;
  localparam sub_alu = 4'b1100;
  localparam sra_alu = 4'b1101;
  localparam bsel_alu = 4'b1111;
  localparam sltu_alu = 4'b0011;
    always @(*) begin 
        case(alu_sel) 
            add_alu: out_reg = $signed(a) + $signed(b); // add
            sll_alu: out_reg = a << b[4:0]; // sll
            slt_alu: out_reg = $signed(a) < $signed(b) ? 1 : 0; // slt
            sltu_alu: out_reg = a < b ? 1 : 0;
            xor_alu: out_reg = a ^ b; // xor
            srl_alu: out_reg = a >> b[4:0]; // srl
            or_alu: out_reg = a | b; // or
            and_alu: out_reg = a & b; // and 
            sub_alu: out_reg = a - b; //sub
            sra_alu: out_reg = $signed(a) >>> b[4:0]; // sra
            bsel_alu: out_reg = b; // bsel
            default: out_reg = 0; 
          endcase 
    end 

    assign out = out_reg;

endmodule