module cpu #(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000, 
    parameter BAUD_RATE = 115200,
    parameter BIOS_MIF_HEX = ""
) (
    input clk,
    input rst, // TODO figure out reset before you run the test benches!
    input bp_enable,
    input serial_in,
    output serial_out
);

   localparam DWIDTH = 32;
   localparam BEWIDTH = DWIDTH / 8;

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

    // next_wb_sel signals
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

   // BIOS Memory
   // Synchronous read: read takes one cycle
   // Synchronous write: write takes one cycle
   localparam BIOS_AWIDTH = 12;
   wire [BIOS_AWIDTH-1:0] bios_addra, bios_addrb;
   wire [DWIDTH-1:0]      bios_douta, bios_doutb;
   wire                   bios_ena, bios_enb;
   SYNC_ROM_DP #(.AWIDTH(BIOS_AWIDTH),
                 .DWIDTH(DWIDTH),
                 .MIF_HEX(BIOS_MIF_HEX))
   bios_mem(.q0(bios_douta),
            .addr0(bios_addra),
            .en0(bios_ena),
            .q1(bios_doutb),
            .addr1(bios_addrb),
            .en1(bios_enb),
            .clk(clk));

   // Data Memory
   // Synchronous read: read takes one cycle
   // Synchronous write: write takes one cycle
   // Write-byte-enable: select which of the four bytes to write
   localparam DMEM_AWIDTH = 14;
   wire [DMEM_AWIDTH-1:0] dmem_addra;
   wire [DWIDTH-1:0]      dmem_dina, dmem_douta;
   wire [BEWIDTH-1:0]     dmem_wbea;
   wire                   dmem_ena;
   SYNC_RAM_WBE #(.AWIDTH(DMEM_AWIDTH),
                  .DWIDTH(DWIDTH))
   dmem (.q(dmem_douta),
         .d(dmem_dina),
         .addr(dmem_addra),
         .wbe(dmem_wbea),
         .en(dmem_ena),
         .clk(clk));

   // Instruction Memory
   // Synchronous read: read takes one cycle
   // Synchronous write: write takes one cycle
   // Write-byte-enable: select which of the four bytes to write
   localparam IMEM_AWIDTH = 14;
   wire [IMEM_AWIDTH-1:0] imem_addra, imem_addrb;
   wire [DWIDTH-1:0]      imem_douta, imem_doutb;
   wire [DWIDTH-1:0]      imem_dina, imem_dinb;
   wire [BEWIDTH-1:0]       imem_wbea, imem_wbeb;
   wire                   imem_ena, imem_enb;
   SYNC_RAM_DP_WBE #(.AWIDTH(IMEM_AWIDTH),
                     .DWIDTH(DWIDTH))
   imem (.q0(imem_douta),
         .d0(imem_dina),
         .addr0(imem_addra),
         .wbe0(imem_wbea),
         .en0(imem_ena),
         .q1(imem_doutb),
         .d1(imem_dinb),
         .addr1(imem_addrb),
         .wbe1(imem_wbeb),
         .en1(imem_enb),
         .clk(clk));

   // Register file
   // Asynchronous read: read data is available in the same cycle
   // Synchronous write: write takes one cycle
   localparam RF_AWIDTH = 5;
   wire [RF_AWIDTH-1:0]   wa, ra1, ra2;
   wire [DWIDTH-1:0]      wd, rd1, rd2;
   wire                   we;
   ASYNC_RAM_1W2R # (.AWIDTH(RF_AWIDTH),
                     .DWIDTH(DWIDTH))
   rf (.addr0(wa),
       .d0(wd),
       .we0(we),
       .q1(rd1),
       .addr1(ra1),
       .q2(rd2),
       .addr2(ra2),
       .clk(clk));


   // UART Memory
   // Synchronous read: read takes one cycle
   // Synchronous write: write takes one cycle
   // Write-byte-enable: select which of the four bytes to write
   wire [DMEM_AWIDTH-1:0] uart_map_addra;
   wire [DWIDTH-1:0]      uart_map_dina, uart_map_douta;
   wire [BEWIDTH-1:0]     uart_map_wbea;
   wire                   uart_map_ena;
   SYNC_RAM_WBE #(.AWIDTH(DMEM_AWIDTH),
                  .DWIDTH(DWIDTH))
   uart_map (.q(uart_map_douta),
         .d(uart_map_dina),
         .addr(uart_map_addra),
         .wbe(uart_map_wbea),
         .en(uart_map_ena),
         .clk(clk));

   // On-chip UART
   //// UART Receiver
   wire [7:0]             uart_rx_data_out;
   wire                   uart_rx_data_out_valid;
   wire                   uart_rx_data_out_ready;
   //// UART Transmitter
   wire [7:0]             uart_tx_data_in;
   wire                   uart_tx_data_in_valid;
   wire                   uart_tx_data_in_ready;
   uart #(.CLOCK_FREQ(CPU_CLOCK_FREQ),
          .BAUD_RATE(BAUD_RATE))
   on_chip_uart (.clk(clk),
                 .reset(rst),
                 .serial_in(serial_in),
                 .data_out(uart_rx_data_out),
                 .data_out_valid(uart_rx_data_out_valid),
                 .data_out_ready(uart_rx_data_out_ready),
                 .serial_out(serial_out),
                 .data_in(uart_tx_data_in),
                 .data_in_valid(uart_tx_data_in_valid),
                 .data_in_ready(uart_tx_data_in_ready));

   // CSR
   wire [DWIDTH-1:0]      csr_dout, csr_din;
   wire                   csr_we;
   REGISTER_R_CE #(.N(DWIDTH))
   csr (.q(csr_dout),
        .d(csr_din),
        .rst(rst),
        .ce(csr_we),
        .clk(clk));

    /* CONTROLLER INPUTS */

    // IF STAGE
    wire [DWIDTH-1:0] pc_din;
    wire imem_sel; 
    wire stall; // shared with reg stage
    wire flush_if; 

    // REG STAGE
    wire [31:0] reg_inst;
    wire [2:0] imm_sel; 
    wire flush_reg; 

    // EX Stage
    wire [31:0] ex_inst;
    wire br_eq_din;
    wire br_lt_din;
    wire [3:0] alu_sel;
    wire a_sel;
    wire b_sel;
    wire flush_ex; 
    wire [2:0] a_fwd_sel; 
    wire [2:0] b_fwd_sel; 

    // MEM stage 
    wire [31:0] mem_inst;
    wire [31:0] mem_alu_dout;
    wire [3:0] dmem_wen;
    wire [3:0] imem_wen;
    wire [3:0] uart_wen; 
    wire signed_load; 
    wire [1:0] pc_sel;
    wire inst_counter_cen; 
    wire [3:0] store_mask; 
    wire br_eq_dout;  
    wire br_lt_dout; 
    wire [31:0] mem_pc; 
    wire counter_rst;

    // WB Stage
    wire [31:0] wb_inst;
    wire [31:0] wb_alu_dout;
    wire [1:0] dmem_sel;
    wire [31:0] load_mask; 
    wire [2:0] wb_sel;

    // Post WB 
    wire [31:0] post_wb_inst; 

    // Post Post WB
    wire [31:0] post_post_wb_inst; 

    // leave all the memories enabled
    assign bios_ena = 1'b1;
    assign imem_ena = 1'b1;
    assign imem_enb = 1'b1;
    assign dmem_ena = 1'b1; 
    assign bios_enb = 1'b1;

    /* CONTROLLER INSTANTIATION */

    controller controller (
        // IF STAGE
        .pc_din(pc_din),
        .imem_sel(imem_sel), 
        .stall(stall), // shared with reg stage
        .flush_if(flush_if),

        // REG STAGE
        .reg_inst(reg_inst),
        .imm_sel(imm_sel), 
        .flush_reg(flush_reg),

        // EX Stage
        .ex_inst(ex_inst),
        .alu_sel(alu_sel),
        .a_sel(a_sel),
        .b_sel(b_sel),
        .flush_ex(flush_ex),
        .a_fwd_sel(a_fwd_sel),
        .b_fwd_sel(b_fwd_sel),

        // MEM stage 
        .mem_inst(mem_inst),
        .mem_alu_dout(mem_alu_dout),
        .mem_pc(mem_pc),
        .dmem_wen(dmem_wen),
        .imem_wen(imem_wen),
        .uart_wen(uart_wen),
        .uart_valid(uart_tx_data_in_valid),
        .uart_ready(uart_rx_data_out_ready),
        .csr_cen(csr_we),
        .signed_load(signed_load),
        .pc_sel(pc_sel),
        .inst_counter_cen(inst_counter_cen),
        .store_mask(store_mask),
        .br_eq(br_eq_dout),
        .br_lt(br_lt_dout),
        .counter_rst(counter_rst),

        // WB Stage
        .wb_inst(wb_inst),
        .wb_alu_dout(wb_alu_dout),
        .dmem_sel(dmem_sel),
        .load_mask(load_mask),
        .wb_sel(wb_sel),
        .reg_wen(we),

        // Post WB
        .post_wb_inst(post_wb_inst),

        // Post Post WB
        .post_post_wb_inst(post_post_wb_inst)
    );


    /* WIRES THAT AREN'T INPUTS TO THE CONTROLLER */

    // IF STAGE
    wire [DWIDTH-1:0] pc_plus_four;

    // REG STAGE
    wire [DWIDTH-1:0] reg_pc;
    wire [DWIDTH-1:0] a_din; 
    wire [DWIDTH-1:0] b_din; 
    wire [DWIDTH-1:0] a_dout; 
    wire [DWIDTH-1:0] b_dout; 
    wire [DWIDTH-1:0] imm_din;
    wire signed [DWIDTH-1:0] reg_imm_gen;
    wire [DWIDTH-1:0] reg_uart_control;
    wire [DWIDTH-1:0] reg_uart_data;

    // EX STAGE
    wire [DWIDTH-1:0] ex_pc; 
    wire [DWIDTH-1:0] alu_din;
    wire [DWIDTH-1:0] asel_out;
    wire [DWIDTH-1:0] bsel_out;
    wire signed [DWIDTH-1:0] ex_imm_gen;
    wire [DWIDTH-1:0] a_fwd_out;
    wire [DWIDTH-1:0] b_fwd_out;
    wire [DWIDTH-1:0] ex_uart_control;
    wire [DWIDTH-1:0] ex_uart_data;

    // MEM STAGE
    wire [DWIDTH-1:0] mem_rs2;
    wire [DWIDTH-1:0] formatted_rs2_data;
    wire [DWIDTH-1:0] formatted_load;
    wire [DWIDTH-1:0] mem_pc_plus_four;
    wire [1:0] shamt;
    wire [DWIDTH-1:0] mem_uart_control;
    wire [DWIDTH-1:0] mem_uart_data;
    wire [DWIDTH-1:0] cycle_counter_curr; 
    wire [DWIDTH-1:0] cycle_counter_next; 
    wire [DWIDTH-1:0] inst_counter_curr; 
    wire [DWIDTH-1:0] inst_counter_next; 

    // WB STAGE
    wire [DWIDTH-1:0] dmem_sel_out;
    wire [DWIDTH-1:0] wb_sel_out;
    wire [DWIDTH-1:0] wb_pc_plus_four_wire;
    wire [DWIDTH-1:0] wb_uart_control_wire;
    wire [DWIDTH-1:0] wb_uart_data_wire;

    // POST WB 
    wire [31:0] post_wb_wb_sel_out;

    // POST POST WB 
    wire [31:0] post_post_wb_wb_sel_out; 


    /* ACTUAL DATAPATH */

    /* IF STAGE */
    assign pc_plus_four = reg_pc + 32'd4; 

    assign pc_din = rst === 1'b1 ? RESET_PC : (pc_sel === pc_sel_pc_alu) ? mem_alu_dout : (pc_sel === pc_sel_pc_pc) ? reg_pc : pc_plus_four;

    assign imem_addra = pc_din[15:2];
    assign bios_addra = pc_din[13:2];
    
    assign reg_inst = flush_if === 1'b1 || rst === 1'b1 ? 32'h00000013 : imem_sel === 0 ? imem_douta : bios_douta;
    
    // PIPELINE REGISTERS

    // pc register
    REGISTER_CE #(.N(DWIDTH))
    if_pc_register (.q(reg_pc), .d(pc_din), .clk(clk), .ce(~stall));

    /* REG STAGE */

    assign ra1 = reg_inst[19:15];
    assign ra2 = reg_inst[24:20];

    // immediate generator
    imm_gen ig(.inst(reg_inst), .out(reg_imm_gen), .imm_sel(imm_sel)); 


    // PIPELINE REGISTERS

    // instruction register
    REGISTER_R_CE #(.N(DWIDTH), .INIT(32'h00000013))
    reg_inst_register (.q(ex_inst), .d(reg_inst), .rst(flush_reg), .clk(clk), .ce(~stall));

    // branch eq register
    REGISTER_CE #(.N(1))
    reg_br_eq_comp_register (.q(br_eq_dout), .d(br_eq_din), .clk(clk), .ce(~stall));

    // branch lt register
    REGISTER_CE #(.N(1))
    reg_br_lt_comp_register (.q(br_lt_dout), .d(br_lt_din), .clk(clk), .ce(~stall));

    // a register
    REGISTER_CE #(.N(DWIDTH))
    reg_a_register (.q(a_dout), .d(rd1), .clk(clk), .ce(~stall));

    // b register
    REGISTER_CE #(.N(DWIDTH))
    reg_b_register (.q(b_dout), .d(rd2), .clk(clk), .ce(~stall));

    // imm_gen register
    REGISTER_CE #(.N(DWIDTH))
    reg_imm_register (.q(ex_imm_gen), .d(reg_imm_gen), .clk(clk), .ce(~stall));
    
    // pc register
    REGISTER_CE #(.N(DWIDTH))
    reg_pc_register (.q(ex_pc),.d(reg_pc), .clk(clk), .ce(~stall));

    /* EX STAGE */

    branch_comp bc(.a(a_fwd_out), .b(b_fwd_out), .br_eq(br_eq_din), .br_lt(br_lt_din), .inst(ex_inst)); 

    // a_fwd mux
    assign a_fwd_out = a_fwd_sel === no_fwd ? a_dout : a_fwd_sel === fwd_ex ? mem_alu_dout : a_fwd_sel === fwd_post_wb ? post_wb_wb_sel_out : a_fwd_sel === fwd_post_post_wb ? post_post_wb_wb_sel_out : wb_sel_out;

    // b_fwd mux
    assign b_fwd_out = b_fwd_sel === no_fwd ? b_dout : b_fwd_sel === fwd_ex ? mem_alu_dout : b_fwd_sel === fwd_post_wb ? post_wb_wb_sel_out : b_fwd_sel === fwd_post_post_wb ? post_post_wb_wb_sel_out : wb_sel_out;

    // asel mux 
    assign asel_out = a_sel === 1'b0 ? a_fwd_out : ex_pc;

    // bsel mux
    assign bsel_out = b_sel === 1'b1 ? ex_imm_gen : (ex_inst[24:20] === ex_inst[19:15]) ? a_fwd_out : b_fwd_out;

    alu cpu_alu(.a(asel_out), .alu_sel(alu_sel), .b(bsel_out), .out(alu_din)); 

    // PIPELINE REGISTERS

    // instruction register
    REGISTER_R_CE#(.N(DWIDTH), .INIT(32'h00000013))
    ex_inst_register (.q(mem_inst), .d(ex_inst), .rst(flush_ex), .clk(clk), .ce(~stall));

    // alu_dout register
    REGISTER_CE #(.N(DWIDTH))
    ex_alu_register (.q(mem_alu_dout), .d(alu_din), .clk(clk), .ce(~stall));

    // pc register
    REGISTER_CE #(.N(DWIDTH))
    ex_pc_register (.q(mem_pc), .d(ex_pc), .clk(clk), .ce(~stall));

    // rs2_register
    REGISTER_CE #(.N(DWIDTH))
    ex_rs2_register (.q(mem_rs2), .d(b_fwd_out), .clk(clk), .ce(~stall));

    /* MEM STAGE */
    store_formatter store_formatter(.rs2_dout(mem_rs2), .mem_mask(store_mask), .formatted_data(formatted_rs2_data));

    assign imem_addrb = mem_alu_dout[15:2]; 
    assign dmem_addra = mem_alu_dout[15:2];
    assign bios_addrb = mem_alu_dout[13:2];

    assign imem_dinb = formatted_rs2_data;
    assign dmem_dina = formatted_rs2_data;
    assign uart_map_dina = formatted_rs2_data;
    assign uart_tx_data_in = formatted_rs2_data[7:0];

    assign imem_wbeb = imem_wen;
    assign dmem_wbea = dmem_wen; 
    assign uart_map_wbea = uart_wen;

    assign mem_pc_plus_four = mem_pc + 32'd4; 

    assign csr_din = mem_alu_dout; 

    // Cycle Counter Register
    assign cycle_counter_next = cycle_counter_curr + 32'd1;
    REGISTER_R #(.N(DWIDTH))
    cycle_counter_reg (.q(cycle_counter_curr),
        .d(cycle_counter_next), .rst(counter_rst),
        .clk(clk));

    // Instruction Counter Register
    assign inst_counter_next = inst_counter_curr + 32'd1;
    REGISTER_R_CE #(.N(DWIDTH))
    inst_counter_reg (.q(inst_counter_curr),
        .d(inst_counter_next), .rst(counter_rst),
        .clk(clk), .ce(inst_counter_cen));

    // PIPELINE REGISTERS

    // instruction register
    REGISTER_R #(.N(DWIDTH), .INIT(32'h00000013))
    mem_inst_register (.q(wb_inst), .d(mem_inst), .rst(rst), .clk(clk));

    // alu_dout register
    REGISTER #(.N(DWIDTH))
    mem_alu_register (.q(wb_alu_dout), .d(mem_alu_dout), .clk(clk));

    // pc plus four register
    REGISTER #(.N(DWIDTH))
    mem_pc_plus_four_register (.q(wb_pc_plus_four_wire), .d(mem_pc_plus_four), .clk(clk));

    /* WB STAGE */ 

    assign dmem_sel_out = dmem_sel === dmem_sel_bios ? bios_doutb : dmem_sel === dmem_sel_dmem ? dmem_douta : wb_uart_data_wire;

    assign shamt = wb_alu_dout[1:0]; //  shift amount for the load formatter is the bottom two bits of the alu output (address)

    load_formatter loader(.load_mask(load_mask), .signed_load(signed_load), .mem_data(dmem_sel_out), .formatted_data(formatted_load), .shamt(shamt)); 

    // get uart signals
    assign reg_uart_data = {24'd0, uart_rx_data_out};
    assign reg_uart_control = {30'd0, uart_rx_data_out_valid, uart_tx_data_in_ready};

    assign wb_sel_out = wb_sel === wb_alu ? wb_alu_dout : wb_sel === wb_mem ? formatted_load : wb_sel === wb_uart_control ? 
    reg_uart_control : wb_sel === wb_uart_data ? reg_uart_data : wb_sel === wb_cyc_counter ? cycle_counter_curr : 
    wb_sel === wb_inst_counter ? inst_counter_curr :  wb_pc_plus_four_wire;
    
    assign wa = wb_inst[11:7];
    assign wd = wb_sel_out;

    /* POST WB */
    // instruction register

    REGISTER_R #(.N(DWIDTH), .INIT(32'h00000013))
    post_wb_inst_register (.q(post_wb_inst), .d(wb_inst), .rst(rst), .clk(clk));

    // wb_sel_out register
    REGISTER #(.N(DWIDTH))
    post_wb_sel_out_reg (.q(post_wb_wb_sel_out), .d(wb_sel_out), .clk(clk));

    /* POST POST WB */
    // instruction register

    REGISTER_R #(.N(DWIDTH), .INIT(32'h00000013))
    post_post_wb_inst_register (.q(post_post_wb_inst), .d(post_wb_inst), .rst(rst), .clk(clk));

    // wb_sel_out register
    REGISTER #(.N(DWIDTH))
    post_post_wb_sel_out_reg (.q(post_post_wb_wb_sel_out), .d(post_wb_wb_sel_out), .clk(clk));

endmodule