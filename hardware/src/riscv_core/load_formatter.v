module load_formatter (input [31:0] load_mask, input signed_load, input [31:0] mem_data, output reg [31:0] formatted_data, input [1:0] shamt); 
    reg [31:0] shifted_data;  
    reg [31:0] intermediate_data;
    reg [31:0] shamt_times_eight; 
    always @(*) begin 
        shamt_times_eight = shamt * 8;
        shifted_data = mem_data >> shamt_times_eight;
        intermediate_data = shifted_data & load_mask; 
        case (signed_load) 
            1'b1: begin // for signed loads, sign extension depends on the type of load, which we can determine from the mask
                case (load_mask)
                    32'h000000ff: begin 
                        formatted_data = {{24{intermediate_data[7]}}, intermediate_data[7:0]};
                    end 
                    32'h0000ffff: begin 
                        formatted_data = {{16{intermediate_data[15]}}, intermediate_data[15:0]};
                    end 
                    default: begin 
                        formatted_data = intermediate_data; 
                    end 
                endcase 
            end 
            default: begin // for unsigned loads, just and with the load mask
                formatted_data = intermediate_data; 
            end 
        endcase 
    end 
endmodule 