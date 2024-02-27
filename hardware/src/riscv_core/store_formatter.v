module store_formatter (input [3:0] mem_mask, input [31:0] rs2_dout, output reg [31:0] formatted_data); 
    always @(*) begin 
        case (mem_mask)
            4'b0001: begin 
                formatted_data = {24'hxx_xx_xx, rs2_dout[7:0]};
            end 
            4'b0010: begin 
                formatted_data = {16'hxx_xx, rs2_dout[7:0], 8'hxx};
            end 
            4'b0100: begin 
                formatted_data = {8'hxx, rs2_dout[7:0], 16'hxx_xx};
            end 
            4'b1000: begin 
                formatted_data = {rs2_dout[7:0], 24'hxx_xx_xx};
            end 
            4'b0011: begin 
                formatted_data = {16'hxx_xx, rs2_dout[15:0]};
            end 
            4'b1100: begin 
                formatted_data = {rs2_dout[15:0], 16'hxx_xx};
            end 
            default: begin 
                formatted_data = rs2_dout;
            end 
        endcase 
    end 
endmodule 