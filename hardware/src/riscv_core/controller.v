module controller(
    // IF Stage
    input [31:0] pc_din, 
    output reg imem_sel,
    output reg stall,
    output reg flush_if, 

    // REG STAGE
    input [31:0] reg_inst, 
    output reg [2:0] imm_sel, 
    output reg flush_reg,

    // EX Stage
    input [31:0] ex_inst, 
    input br_eq, 
    input br_lt, 
    output reg [3:0] alu_sel, 
    output reg a_sel, 
    output reg b_sel, 
    output reg flush_ex,
    output reg [2:0] a_fwd_sel,
    output reg [2:0] b_fwd_sel,

    // MEM stage 
    input [31:0] mem_inst, 
    input [31:0] mem_alu_dout,
    output reg [3:0] dmem_wen, 
    input [31:0] mem_pc, 
    output reg [3:0] imem_wen, 
    output reg [3:0] uart_wen, 
    output reg csr_cen, 
    output reg uart_valid, 
    output reg uart_ready, 
    output reg signed_load, 
    output reg [1:0] pc_sel, 
    output reg inst_counter_cen, 
    output reg [3:0] store_mask,
    output reg counter_rst, 

    // WB Stage
    input [31:0] wb_inst, 
    input [31:0] wb_alu_dout,
    output reg [1:0] dmem_sel,
    output reg [31:0] load_mask,  
    output reg [2:0] wb_sel, 
    output reg reg_wen,

    // Post WB 
    input [31:0] post_wb_inst,

    // Post Post WB 
    input [31:0] post_post_wb_inst
);
    // opcodes
    localparam r_opcode = 7'b0110011;
    localparam load_opcode = 7'b0000011;
    localparam i_opcode = 7'b0010011;
    localparam s_opcode = 7'b0100011;
    localparam b_opcode = 7'b1100011;
    localparam auipc_opcode = 7'b0010111;
    localparam lui_opcode = 7'b0110111;
    localparam jal_opcode = 7'b1101111;
    localparam jalr_opcode = 7'b1100111; 
    localparam csr_opcode = 7'b1110011;

    // imm_sel signals
    localparam i_type_imm = 3'b000;
    localparam s_type_imm = 3'b001; 
    localparam b_type_imm = 3'b010; 
    localparam u_type_imm = 3'b011;
    localparam j_type_imm = 3'b100;
    localparam csr_rwi_type = 3'b101;
    localparam csr_rw_type = 3'b110;
    localparam shift_type_imm = 3'b111;

    // alu_sel signals
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

    // func3 for R and I instructions
    localparam func3_add = 3'b000;
    localparam func3_xor = 3'b100;
    localparam func3_and = 3'b111; 
    localparam func3_or = 3'b110;
    localparam func3_sll = 3'b001;
    localparam func3_slti = 3'b010; 
    localparam func3_srl_or_sra = 3'b101;
    localparam func3_sltu = 3'b011; 

    // wb_sel signals
    localparam wb_mem = 3'b000;
    localparam wb_alu = 3'b001;
    localparam wb_inst_counter = 3'b010;
    localparam wb_cyc_counter = 3'b011;
    localparam wb_pc_plus_four = 3'b100;
    localparam wb_uart_control = 3'b101;
    localparam wb_uart_data = 3'b110;

    // forwarding signals 
    localparam fwd_ex = 3'b010;
    localparam fwd_mem = 3'b001;
    localparam no_fwd = 3'b000;
    localparam fwd_wb = 3'b011;
    localparam fwd_post_wb = 3'b100; 
    localparam fwd_post_post_wb = 3'b111; 

    // pcsel signals
    localparam pc_sel_rst = 2'b11;
    localparam pc_sel_pc_pc = 2'b10;
    localparam pc_sel_pc_alu = 2'b01; 
    localparam pc_sel_pc_plus_four = 2'b00;

    // imem_sel signals
    localparam imem_sel_imem = 1'b0;
    localparam imem_sel_bios = 1'b1; 

    // dmem_sel signals
    localparam dmem_sel_uart = 2'b10;
    localparam dmem_sel_dmem = 2'b01; 
    localparam dmem_sel_bios = 2'b00; 
    localparam dmem_sel_imem = 2'b11; 
    
    //load_mask
    localparam load_byte_mask = 32'h000000ff;
    localparam load_half_mask = 32'h0000ffff;
    localparam load_word_mask = 32'hffffffff;

    // IF 

    // REG 
    reg [6:0] reg_opcode;
    reg [2:0] reg_func3;

    // EX 
    reg [6:0] ex_opcode;
    reg [2:0] ex_func3;
    reg [6:0] ex_func7;
    reg [4:0] ex_rs1;
    reg [4:0] ex_rs2;

    // MEM 
    reg [6:0] mem_opcode;
    reg [2:0] mem_func3;
    reg [4:0] mem_rd; 
    
    // WB 
    reg [6:0] wb_opcode;
    reg [2:0] wb_func3;
    reg [4:0] wb_rd; 

    // Post WB
    reg [6:0] post_wb_opcode;
    reg [4:0] post_wb_rd; 
    reg [4:0] post_wb_rs1;
    reg [4:0] post_wb_rs2;

    // Post Post WB
    reg [6:0] post_post_wb_opcode;
    reg [4:0] post_post_wb_rd; 

    // data hazard handling
    reg potential_one_stage_data_hazard;
    reg potential_two_stage_data_hazard; 
    reg potential_load_data_hazard; 
    reg potential_ex_post_wb_data_hazard;
    reg potential_two_load_data_hazard;
    reg potential_ex_post_post_wb_data_hazard;
    reg has_source_registers; 


    always @(*) begin 
        imem_sel = 1'b0; 
        pc_sel = 2'b0; 
        signed_load = 1'b0; 
        load_mask = 32'd0;
        counter_rst = 1'b0;
        // set imem_sel 
        case (pc_din[31:28]) 
            4'b0001: imem_sel = imem_sel_imem; 
            4'b0100: imem_sel = imem_sel_bios;
            default: imem_sel = imem_sel_imem;
        endcase 

        // set instruction values
        reg_opcode = reg_inst[6:0];
        reg_func3 = reg_inst[14:12];

        // set instruction values
        ex_opcode = ex_inst[6:0];
        ex_func3 = ex_inst[14:12];
        ex_func7 = ex_inst[31:25];
        ex_rs1 = ex_inst[19:15];
        ex_rs2 = ex_inst[24:20];

        // set instruction values
        mem_opcode = mem_inst[6:0];
        mem_func3 = mem_inst[14:12];
        mem_rd = mem_inst[11:7];

        // set instruction values
        wb_opcode = wb_inst[6:0];
        wb_func3 = wb_inst[14:12];
        wb_rd = wb_inst[11:7];

        // Post WB
        post_wb_opcode = post_wb_inst[6:0];
        post_wb_rd = post_wb_inst[11:7]; 

        // Post WB
        post_post_wb_opcode = post_post_wb_inst[6:0];
        post_post_wb_rd = post_post_wb_inst[11:7]; 
        
        // set imm_sel
        case (reg_opcode) 
            jalr_opcode: imm_sel = i_type_imm;
            s_opcode: imm_sel = s_type_imm;
            b_opcode: imm_sel = b_type_imm;
            auipc_opcode: imm_sel = u_type_imm;
            lui_opcode: imm_sel = u_type_imm;
            jal_opcode: imm_sel = j_type_imm;
            load_opcode: imm_sel = i_type_imm;
            csr_opcode: begin 
                case (reg_func3)
                    3'b101: imm_sel = csr_rwi_type;
                    default: imm_sel = csr_rw_type;
                endcase
            end 
            i_opcode: begin 
                case (reg_func3)
                    func3_sll: imm_sel = shift_type_imm;
                    func3_srl_or_sra: imm_sel = shift_type_imm;
                    default: imm_sel = i_type_imm;
                endcase 
            end
            default: imm_sel = i_type_imm; // doesn't matter for r type
        endcase 

        alu_sel = 4'b0000;
        a_sel = 1'b0;
        b_sel = 1'b0;

        // set alu_sel
        case (ex_opcode) 
            jalr_opcode: alu_sel = add_alu; 
            s_opcode: alu_sel = add_alu;
            b_opcode: alu_sel = add_alu;
            auipc_opcode: alu_sel = add_alu;
            lui_opcode: alu_sel = bsel_alu;
            jal_opcode: alu_sel = add_alu;
            load_opcode: alu_sel = add_alu;
            csr_opcode: begin 
                case (ex_func3) 
                    3'b101: alu_sel = bsel_alu;
                    default: alu_sel = add_alu;
                endcase 
            end 
            i_opcode: begin 
                case (ex_func3)
                    func3_add: alu_sel = add_alu;
                    func3_xor: alu_sel = xor_alu; 
                    func3_and: alu_sel = and_alu; 
                    func3_or: alu_sel = or_alu; 
                    func3_slti: alu_sel = slt_alu; 
                    func3_sltu: alu_sel = sltu_alu;
                    func3_sll: begin 
                        alu_sel = sll_alu; 
                    end 
                    func3_srl_or_sra: begin 
                        case (ex_func7) 
                            7'b0000000: begin 
                                alu_sel = srl_alu;
                            end
                            default: begin 
                                alu_sel = sra_alu;
                            end
                        endcase 
                    end
                    default: alu_sel = and_alu;
                endcase                 
            end 
            default: begin // r_opcode
                case (ex_func3)
                    func3_add: begin 
                        case (ex_func7) 
                            7'b0000000: alu_sel = add_alu;
                            default: alu_sel = sub_alu;
                        endcase  
                    end 
                    func3_xor: alu_sel = xor_alu; 
                    func3_and: alu_sel = and_alu; 
                    func3_or: alu_sel = or_alu; 
                    func3_sll: alu_sel = sll_alu; 
                    func3_slti: alu_sel = slt_alu; 
                    func3_sltu: alu_sel = sltu_alu;
                    func3_srl_or_sra: begin 
                        case (ex_func7) 
                            7'b0000000: alu_sel = srl_alu;
                            default: alu_sel = sra_alu;
                        endcase 
                    end 
                    default: alu_sel = and_alu;
                endcase 
            end 
        endcase 

        // set a_sel
        case (ex_opcode) 
            jalr_opcode: a_sel = 1'b0; 
            s_opcode: a_sel = 1'b0;
            b_opcode: a_sel = 1'b1; 
            auipc_opcode: a_sel = 1'b1; 
            lui_opcode: a_sel = 1'b0;
            jal_opcode: a_sel = 1'b1; 
            load_opcode: a_sel = 1'b0;
            csr_opcode: a_sel = 1'b0;
            i_opcode: a_sel = 1'b0;
            default: a_sel = 1'b0;
        endcase 

        // set b_sel
        case (ex_opcode) 
            jalr_opcode: b_sel = 1'b1; 
            s_opcode: b_sel = 1'b1;
            b_opcode: b_sel = 1'b1; 
            auipc_opcode: b_sel = 1'b1; 
            lui_opcode: b_sel = 1'b1;
            jal_opcode: b_sel = 1'b1; 
            load_opcode: b_sel = 1'b1;
            csr_opcode: b_sel = 1'b1;
            i_opcode: b_sel = 1'b1;
            default: b_sel = 1'b0;
        endcase 

        store_mask = 4'b0000;
        flush_ex = 1'b0; 
        flush_reg = 1'b0; 
        flush_if = 1'b0;
        pc_sel = 2'd0;

        // set store_mask
        case (mem_opcode) 
            load_opcode: begin 
                case (mem_func3)
                    3'b000: begin
                        signed_load = 1'b1;
                        load_mask = load_byte_mask;
                    end
                    3'b001: begin
                        signed_load = 1'b1;
                        load_mask = load_half_mask;
                    end
                    3'b010: begin
                        signed_load = 1'b0;
                        load_mask = load_word_mask;
                    end
                    3'b100: begin
                        signed_load = 1'b0;
                        load_mask = load_byte_mask;
                    end
                    default: begin
                        signed_load = 1'b0;
                        load_mask = load_half_mask;
                    end
                endcase
            end 
            default: store_mask = 4'b0000;
        endcase 

        // MEM stage 

        // update pc_sel, flush_ex, and flush_reg for jumps
        pc_sel = pc_sel_pc_plus_four;
        case (mem_opcode) 
            jalr_opcode: begin 
                pc_sel = pc_sel_pc_alu;
                flush_ex = 1'b1; 
                flush_reg = 1'b1; 
                flush_if = 1'b1;
            end 
            jal_opcode: begin 
                pc_sel = pc_sel_pc_alu;
                flush_ex = 1'b1;
                flush_reg = 1'b1;
                flush_if = 1'b1; 
            end 
            b_opcode: begin
                case (mem_func3)
                    3'b000: begin // beq
                        case (br_eq)
                            1'b1: begin
                                pc_sel = pc_sel_pc_alu;
                                flush_ex = 1'b1;
                                flush_reg = 1'b1;
                                flush_if = 1'b1;
                            end
                            default: begin
                            end
                        endcase
                    end
                    3'b001: begin // bne
                        case (br_eq)
                            1'b0: begin
                                pc_sel = pc_sel_pc_alu;
                                flush_ex = 1'b1;
                                flush_reg = 1'b1;
                                flush_if = 1'b1;
                            end
                            default: begin
                            end
                        endcase
                    end
                    3'b100: begin // blt
                        case (br_lt)
                            1'b1: begin
                                pc_sel = pc_sel_pc_alu;
                                flush_ex = 1'b1;
                                flush_reg = 1'b1;
                                flush_if = 1'b1;
                            end
                            default: begin
                            end
                        endcase
                    end
                    3'b101: begin // bge
                        case (br_lt)
                            1'b0: begin
                                pc_sel = pc_sel_pc_alu;
                                flush_ex = 1'b1;
                                flush_reg = 1'b1;
                                flush_if = 1'b1;
                            end
                            default: begin
                            end
                        endcase
                    end
                    3'b110: begin // bltu
                        case (br_lt)
                            1'b1: begin
                                pc_sel = pc_sel_pc_alu;
                                flush_ex = 1'b1;
                                flush_reg = 1'b1;
                                flush_if = 1'b1;
                            end
                            default: begin
                            end
                        endcase
                    end
                    default: begin // bgeu
                        case (br_lt)
                                1'b0: begin
                                pc_sel = pc_sel_pc_alu;
                                flush_ex = 1'b1;
                                flush_reg = 1'b1;
                                flush_if = 1'b1;
                            end
                            default: begin
                            end
                        endcase
                    end
                endcase
            end
            default: begin 
            end 
        endcase 

        dmem_wen = 4'b0000; 
        imem_wen = 4'b0000;
        uart_wen = 4'b0000;
        store_mask = 4'b0000;
        uart_valid = 1'b0;

        case (mem_opcode) 
            s_opcode: begin 
                case (mem_alu_dout[31:28]) 
                    // dmem_wen
                    4'b0001: begin 
                        case (mem_func3)
                            3'b000: begin 
                                case (mem_alu_dout[1:0]) 
                                    2'b00: begin 
                                        dmem_wen = 4'b0001; 
                                        store_mask = 4'b0001;
                                    end 
                                    2'b01: begin 
                                        dmem_wen = 4'b0010; 
                                        store_mask = 4'b0010;
                                    end 
                                    2'b11: begin 
                                        dmem_wen = 4'b1000;
                                        store_mask = 4'b1000;
                                    end 
                                    default: begin 
                                        dmem_wen = 4'b0100;
                                        store_mask = 4'b0100;
                                    end 
                                endcase 
                                end 
                                3'b001: begin 
                                    case (mem_alu_dout[1:0]) 
                                        2'b00: begin 
                                            dmem_wen = 4'b0011; //offset = 0
                                            store_mask = 4'b0011;
                                        end 
                                        2'b01: begin 
                                            dmem_wen = 4'b0110; 
                                            store_mask = 4'b0110;
                                        end 
                                        2'b10: begin 
                                            dmem_wen = 4'b1100; 
                                            store_mask = 4'b1100;
                                        end 
                                        default: begin 
                                            dmem_wen = 4'b1100; 
                                            store_mask = 4'b1100;
                                        end 
                                    endcase 
                                end 
                                default: begin 
                                    dmem_wen = 4'b1111; // no shift
                                    store_mask = 4'b1111;
                                end 
                        endcase 
                    end 
                    4'b0011: begin 
                        case (mem_func3)
                            3'b000: begin 
                                case (mem_alu_dout[1:0]) 
                                    2'b00: begin 
                                        dmem_wen = 4'b0001; 
                                        store_mask = 4'b0001;
                                            case (mem_pc[30])
                                                1'b1: begin 
                                                    imem_wen = 4'b0001; 
                                                    store_mask = 4'b0001;
                                                end 
                                                default: begin 
                                                end 
                                            endcase
                                    end 
                                    2'b01: begin 
                                        dmem_wen = 4'b0010; 
                                        store_mask = 4'b0010;
                                        case (mem_pc[30])
                                            1'b1: begin 
                                                imem_wen = 4'b0010; 
                                                store_mask = 4'b0010;
                                            end 
                                            default: begin 
                                            end 
                                        endcase 
                                    end 
                                    2'b11: begin 
                                        dmem_wen = 4'b1000;
                                        store_mask = 4'b1000;
                                        case (mem_pc[30])
                                            1'b1: begin 
                                                imem_wen = 4'b1000; 
                                                store_mask = 4'b1000;
                                            end 
                                            default: begin 
                                            end 
                                        endcase 
                                    end 
                                    default: begin 
                                        dmem_wen = 4'b0100;
                                        store_mask = 4'b0100;
                                        case (mem_pc[30])
                                            1'b1: begin 
                                                imem_wen = 4'b0100; 
                                                store_mask = 4'b0100;
                                            end 
                                            default: begin 
                                            end 
                                        endcase 
                                    end 
                                endcase 
                                end 
                                3'b001: begin 
                                    case (mem_alu_dout[1:0]) 
                                        2'b00: begin 
                                            dmem_wen = 4'b0011; //offset = 0
                                            store_mask = 4'b0011;
                                            case (mem_pc[30])
                                                1'b1: begin 
                                                    imem_wen = 4'b0011; 
                                                    store_mask = 4'b0011;
                                                end 
                                                default: begin 
                                                end 
                                            endcase 
                                        end 
                                        2'b01: begin 
                                            dmem_wen = 4'b0110; 
                                            store_mask = 4'b0110;
                                            case (mem_pc[30])
                                                1'b1: begin 
                                                    imem_wen = 4'b0110; 
                                                    store_mask = 4'b0110;
                                                end 
                                                default: begin 
                                                end 
                                            endcase 
                                        end 
                                        2'b10: begin 
                                            dmem_wen = 4'b1100; 
                                            store_mask = 4'b1100;
                                            case (mem_pc[30])
                                                1'b1: begin 
                                                    imem_wen = 4'b1100; 
                                                    store_mask = 4'b1100;
                                                end 
                                                default: begin 
                                                end 
                                            endcase 
                                        end 
                                        default: begin 
                                            dmem_wen = 4'b1100; 
                                            store_mask = 4'b1100;
                                        end 
                                    endcase 
                                end 
                                default: begin 
                                    dmem_wen = 4'b1111; // no shift
                                    store_mask = 4'b1111;
                                    if (mem_pc[30]) begin 
                                        imem_wen = 4'b1111; // no shift
                                        store_mask = 4'b1111;
                                    end 
                                end 
                        endcase 
                    end 
                    // imem_wen
                    4'b0010: begin
                        case (mem_func3)
                            3'b000: begin 
                                case (mem_alu_dout[1:0]) 
                                    2'b00: begin 
                                        case (mem_pc[30])
                                            1'b1: begin 
                                                imem_wen = 4'b0001; 
                                                store_mask = 4'b0001;
                                            end 
                                            default: begin 
                                            end 
                                        endcase 
                                    end 
                                    2'b01: begin 
                                        case (mem_pc[30])
                                            1'b1: begin 
                                                imem_wen = 4'b0010; 
                                                store_mask = 4'b0010;
                                            end 
                                            default: begin 
                                            end 
                                        endcase 
                                    end 
                                    2'b11: begin 
                                        case (mem_pc[30])
                                            1'b1: begin 
                                                imem_wen = 4'b1000; 
                                                store_mask = 4'b1000;
                                            end 
                                            default: begin 
                                            end 
                                        endcase 
                                    end 
                                    default: begin 
                                        case (mem_pc[30])
                                            1'b1: begin 
                                                imem_wen = 4'b0100; 
                                                store_mask = 4'b0100;
                                            end 
                                            default: begin 
                                            end 
                                        endcase 
                                    end 
                                endcase 
                            end 
                            3'b001: begin 
                                    case (mem_func3[1:0]) 
                                        2'b00: begin 
                                        case (mem_pc[30])
                                            1'b1: begin 
                                                imem_wen = 4'b0011; 
                                                store_mask = 4'b0011;
                                            end 
                                            default: begin 
                                            end 
                                        endcase 
                                        end 
                                        2'b01: begin 
                                            case (mem_pc[30])
                                                1'b1: begin 
                                                    imem_wen = 4'b0110; 
                                                    store_mask = 4'b0110;
                                                end 
                                                default: begin 
                                                end 
                                            endcase 
                                        end 
                                        2'b10: begin 
                                            case (mem_pc[30])
                                                1'b1: begin 
                                                    imem_wen = 4'b1100; 
                                                    store_mask = 4'b1100;
                                                end 
                                                default: begin 
                                                end 
                                            endcase 
                                        end 
                                        default: begin 
                                            case (mem_pc[30])
                                                1'b1: begin 
                                                    imem_wen = 4'b1100; 
                                                    store_mask = 4'b1100;
                                                end 
                                                default: begin 
                                                end 
                                            endcase 
                                        end 
                                    endcase 
                            end 
                            default: begin 
                                if (mem_pc[30]) begin 
                                    imem_wen = 4'b1111; // no shift
                                    store_mask = 4'b1111;
                                end 
                            end 
                            endcase 
                    end
                    // uart_wen 
                    4'b1000: begin
                        case (mem_func3)
                            3'b000: begin 
                                case (mem_alu_dout[1:0]) 
                                    2'b00: begin 
                                        uart_wen = 4'b0001; 
                                        store_mask = 4'b0001;
                                        uart_valid = 1'b1;
                                    end 
                                    2'b01: begin 
                                        uart_wen = 4'b0010; 
                                        store_mask = 4'b0010;
                                        uart_valid = 1'b1;
                                    end 
                                    2'b11: begin 
                                        uart_wen = 4'b1000;
                                        store_mask = 4'b1000;
                                        uart_valid = 1'b1;
                                    end 
                                    default: begin 
                                        uart_wen = 4'b0100;
                                        store_mask = 4'b0100;
                                        uart_valid = 1'b1;
                                    end 
                                endcase 
                            end 
                            3'b001: begin 
                                case (mem_func3[1:0]) 
                                    2'b00: begin 
                                        uart_wen = 4'b0011; //offset = 0
                                        store_mask = 4'b0011;
                                        uart_valid = 1'b1;
                                    end 
                                    2'b01: begin 
                                        uart_wen = 4'b0110; 
                                        store_mask = 4'b0110;
                                        uart_valid = 1'b1;
                                    end 
                                    2'b10: begin 
                                        uart_wen = 4'b1100; 
                                        store_mask = 4'b1100;
                                        uart_valid = 1'b1;
                                    end 
                                    default: begin 
                                        uart_wen = 4'b1100; 
                                        store_mask = 4'b1100;
                                        uart_valid = 1'b1;
                                    end 
                                endcase 
                            end 
                            default: begin 
                                uart_wen = 4'b1111; // no shift
                                store_mask = 4'b1111;
                                uart_valid = 1'b1;
                            end 
                        endcase
                    end
                default: begin 
                end 
                endcase 
            end 
        default: begin 
        end
    endcase 

        // set csr_cen 
        csr_cen = 1'b0;
        case (mem_opcode)
            csr_opcode: begin 
                csr_cen = 1'b1;
            end 
            default: csr_cen = 1'b0;
        endcase 

        // set inst_counter_cen 
        case (mem_inst)
            32'h00000013: inst_counter_cen = 1'b0; // don't count if it is a nop
            default: inst_counter_cen = 1'b1;
        endcase 

        // set dmem_sel
        case (wb_alu_dout[31:28])
            4'b0001: dmem_sel = dmem_sel_dmem; 
            4'b0011: dmem_sel = dmem_sel_dmem; // overlaps with imem
            4'b0010: dmem_sel = dmem_sel_dmem; // it's supposed to be write only
            4'b0100: dmem_sel = dmem_sel_bios; 
            4'b1000: dmem_sel = dmem_sel_uart; 
            default: dmem_sel = dmem_sel_dmem;
        endcase 

        // set load mask 
        case (wb_opcode)
            load_opcode: begin 
                case (wb_func3)
                    3'b000: begin
                        signed_load = 1'b1;
                        load_mask = load_byte_mask;
                    end
                    3'b001: begin
                        signed_load = 1'b1;
                        load_mask = load_half_mask;
                    end
                    3'b010: begin
                        signed_load = 1'b0;
                        load_mask = load_word_mask;
                    end
                    3'b100: begin
                        signed_load = 1'b0;
                        load_mask = load_byte_mask;
                    end
                    default: begin
                        signed_load = 1'b0;
                        load_mask = load_half_mask;
                    end
                endcase
            end 
            default: begin
            end 
        endcase

        // set reg_wen
        case (wb_opcode)
            jalr_opcode: reg_wen = 1'b1; 
            s_opcode: reg_wen = 1'b0;
            b_opcode: reg_wen = 1'b0;
            auipc_opcode: reg_wen = 1'b1; 
            lui_opcode: reg_wen = 1'b1; 
            jal_opcode: reg_wen = 1'b1; 
            load_opcode: reg_wen = 1'b1;
            csr_opcode: reg_wen = 1'b0;
            i_opcode: reg_wen = 1'b1; 
            default: reg_wen = 1'b1; // r type
        endcase 

        // set reg_wen to 0 if rd is 0 
        case (wb_rd) 
            5'd0: reg_wen = 1'b0;
            default: begin 
            end 
        endcase 

        // set wb_sel 
        case (wb_opcode)
            jalr_opcode: wb_sel = wb_pc_plus_four; 
            s_opcode: wb_sel = 3'b00; // doesn't matter
            b_opcode: wb_sel = 3'b00; // doesn't matter
            auipc_opcode: wb_sel = wb_alu; 
            lui_opcode: wb_sel = wb_alu; 
            jal_opcode: wb_sel = wb_pc_plus_four; 
            load_opcode: begin 
                wb_sel = wb_mem;
                case (wb_alu_dout) 
                    32'h80000000: wb_sel = wb_uart_control;
                    32'h80000004: wb_sel = wb_uart_data;
                    32'h80000010: wb_sel = wb_cyc_counter;
                    32'h80000014: wb_sel = wb_inst_counter;
                    default: wb_sel = wb_mem;
                endcase 
            end
            csr_opcode: wb_sel = 3'b00; // doesn't matter
            i_opcode: wb_sel = wb_alu; 
            default: wb_sel = wb_alu; // r type
        endcase

        uart_ready = 1'b0;

        if (mem_opcode === load_opcode && mem_alu_dout === 32'h80000004) begin 
            uart_ready = 1'b1; 
        end 

        if (mem_opcode === s_opcode && mem_alu_dout === 32'h8000_0018) begin 
            counter_rst = 1'b1;
        end 


        /* HANDLE DATA HAZARDS*/

        a_fwd_sel = no_fwd;
        b_fwd_sel = no_fwd;
        has_source_registers = ex_opcode !== auipc_opcode || ex_opcode !== lui_opcode || ex_opcode !== jal_opcode; 

        potential_ex_post_post_wb_data_hazard = (post_post_wb_opcode == load_opcode || post_post_wb_opcode == auipc_opcode || post_post_wb_opcode === lui_opcode || post_post_wb_opcode === i_opcode || post_post_wb_opcode === r_opcode
        || post_post_wb_opcode === jal_opcode || post_post_wb_opcode === jalr_opcode) && (ex_opcode === jalr_opcode || ex_opcode === s_opcode || ex_opcode === b_opcode || ex_opcode === load_opcode ||  ex_opcode === csr_opcode && ex_func3 === 3'b001 
            ||  ex_opcode === i_opcode || ex_opcode === r_opcode);
                    
        if (potential_ex_post_post_wb_data_hazard) begin 
            if (post_post_wb_rd === ex_rs1 && post_post_wb_rd !== 5'd0) begin 
                a_fwd_sel = fwd_post_post_wb;
            end
            if (post_post_wb_rd === ex_rs2 && post_post_wb_rd !== 5'd0) begin 
                b_fwd_sel = fwd_post_post_wb;
            end
        end 


        potential_ex_post_wb_data_hazard = (post_wb_opcode == load_opcode || post_wb_opcode == auipc_opcode || post_wb_opcode === lui_opcode || post_wb_opcode === i_opcode || post_wb_opcode === r_opcode
        || post_wb_opcode === jal_opcode || post_wb_opcode === jalr_opcode) 
            && (ex_opcode === jalr_opcode || ex_opcode === s_opcode || ex_opcode === b_opcode || ex_opcode === load_opcode ||  ex_opcode === csr_opcode && ex_func3 === 3'b001 
            ||  ex_opcode === i_opcode || ex_opcode === r_opcode); 
        
        if (potential_ex_post_wb_data_hazard) begin 
            if (post_wb_rd === ex_rs1 && post_wb_rd !== 5'd0) begin 
                a_fwd_sel = fwd_post_wb;
            end
            if (post_wb_rd === ex_rs2 && post_wb_rd !== 5'd0) begin 
                b_fwd_sel = fwd_post_wb;
            end
        end 


        // handle two stage data hazards
        potential_two_stage_data_hazard = (wb_opcode == load_opcode || wb_opcode == auipc_opcode || wb_opcode === lui_opcode || wb_opcode === i_opcode || wb_opcode === r_opcode
        || wb_opcode === jal_opcode || wb_opcode === jalr_opcode) 
            && (ex_opcode === jalr_opcode || ex_opcode === s_opcode || ex_opcode === b_opcode || ex_opcode === load_opcode ||  ex_opcode === csr_opcode && ex_func3 === 3'b001 
            ||  ex_opcode === i_opcode || ex_opcode === r_opcode); 
            
        if (potential_two_stage_data_hazard) begin 
            if (wb_rd === ex_rs1 && wb_rd !== 5'd0) begin 
                a_fwd_sel = fwd_wb;
            end
            if (wb_rd === ex_rs2 && wb_rd !== 5'd0) begin 
                b_fwd_sel = fwd_wb; 
            end 
        end 

        // handle load data hazards
        potential_load_data_hazard = mem_opcode === load_opcode && (ex_opcode === jalr_opcode || ex_opcode === s_opcode || ex_opcode === b_opcode || ex_opcode === load_opcode ||  ex_opcode === csr_opcode && ex_func3 === 3'b001 
            ||  ex_opcode === i_opcode || ex_opcode === r_opcode); 
        stall = 1'b0; 
        if (potential_load_data_hazard) begin 
            if (mem_rd === ex_rs1 && mem_rd !== 5'd0) begin 
                stall = 1'b1; 
                flush_ex = 1'b1; 
                pc_sel = pc_sel_pc_pc;
            end
            if (mem_rd === ex_rs2 && mem_rd !== 5'd0) begin 
                stall = 1'b1;
                flush_ex = 1'b1; 
                pc_sel = pc_sel_pc_pc;
            end 
        end 

        // handle one stage ex hazards 
        potential_one_stage_data_hazard = (mem_opcode == load_opcode || mem_opcode == auipc_opcode || mem_opcode === lui_opcode || mem_opcode === i_opcode || mem_opcode === r_opcode 
        || mem_opcode === jal_opcode || mem_opcode === jalr_opcode) 
            && (ex_opcode === jalr_opcode || ex_opcode === s_opcode || ex_opcode === b_opcode || ex_opcode === load_opcode ||  ex_opcode === csr_opcode && ex_func3 === 3'b001 
            ||  ex_opcode === i_opcode || ex_opcode === r_opcode); 
        
        if (potential_one_stage_data_hazard) begin 
            if (mem_rd === ex_rs1 && mem_rd !== 5'd0) begin 
                a_fwd_sel = fwd_ex;
            end 
            if (mem_rd === ex_rs2 && mem_rd !== 5'd0) begin 
                b_fwd_sel = fwd_ex;
            end
        end 
    end 


endmodule 