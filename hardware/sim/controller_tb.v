module controller_tb;
reg [31:0] inst;
reg rst; 
reg br_eq;
reg br_lt; 
reg [31:0] pc; 
reg [31:0] alu_data;
reg [31:0] mem_phase_inst;
reg [4:0] mem_hazard_signals; 
reg [44:0] curr_mem_phase_control_signals;

wire [44:0] next_mem_phase_control_signals;
wire [4:0] ex_hazard_signals;
wire [2:0] imm_sel;
wire br_un; 
wire a_sel; 
wire b_sel;
wire [3:0] alu_sel; 
wire [1:0] wb_sel; 
wire imem_sel; 
wire [1:0] pc_sel;
wire [1:0] a_fwd;
wire [1:0] b_fwd;
wire [3:0] dmem_wen; 
wire [3:0] imem_wen;
wire [3:0] uart_wen; 
wire [31:0] load_mask; 
wire uart_wen;
wire nop;
wire wr_x0;
wire [1:0] dmem_sel; 
wire [1:0] csr_cen;
wire reg_wen;
wire signed_load;
wire [4:0] wa;

reg [5:0] expected_wa;
reg [44:0] expected_next_mem_phase_control_signals;
reg [4:0] expected_ex_hazard_signals;
reg [2:0] expected_imm_sel;
reg expected_reg_wen;
reg expected_br_un; 
reg expected_a_sel; 
reg expected_b_sel;
reg [3:0] expected_alu_sel; 
reg [1:0] expected_wb_sel; 
reg expected_imem_sel; 
reg [1:0] expected_pc_sel;
reg [1:0] expected_a_fwd;
reg [1:0] expected_b_fwd;
reg [3:0] expected_dmem_wen; 
reg [3:0] expected_imem_wen;
reg [3:0] expected_uart_wen; 
reg [31:0] expected_load_mask; 
reg expected_uart_wen;
reg expected_nop;
reg expected_wr_x0;
reg [1:0] expected_dmem_sel; 
reg [1:0] expected_csr_cen;
reg expected_signed_load;
  
  reg clk;
  initial clk = 0;
  always #1 clk = ~clk;
  
  task check; begin
	if  (wa !== expected_wa || signed_load !== expected_signed_load || next_mem_phase_control_signals !== expected_next_mem_phase_control_signals || ex_hazard_signals !== expected_ex_hazard_signals || imm_sel !== expected_imm_sel || br_un !== expected_br_un || a_sel !== expected_a_sel || b_sel !== expected_b_sel || alu_sel !== expected_alu_sel || wb_sel !== expected_wb_sel || imem_sel !== expected_imem_sel || pc_sel !== expected_pc_sel || a_fwd !== expected_a_fwd || b_fwd !== expected_b_fwd || dmem_wen !== expected_dmem_wen || imem_wen !== expected_imem_wen || uart_wen !== expected_uart_wen || load_mask !== expected_load_mask || uart_wen !== expected_uart_wen || nop !== expected_nop || wr_x0 !== expected_wr_x0 || dmem_sel !== expected_dmem_sel || csr_cen !== expected_csr_cen || reg_wen !== expected_reg_wen) begin
		$display("next_mem_phase_control_signals = %b \n expected_next_mem_phase_control_signals = %b \nex_hazard_signals = %b \nexpected_ex_hazard_signals = %b \n imm_sel = %b \nexpected_imm_sel = %b \nbr_un = %b \nexpected_br_un = %b\na_sel = %b \nexpected_a_sel = %b \nb_sel = %b \nexpected_b_sel = %b \nalu_sel = %b \nexpected_alu_sel = %b \nwb_sel = %b \nexpected_wb_sel = %b \nimem_sel = %b \nexpected_imem_sel = %b \npc_sel = %b \nexpected_pc_sel = %b \na_fwd = %b \nexpected_a_fwd = %b \nb_fwd = %b \nexpected_b_fwd = %b \ndmem_wen = %b \nexpected_dmem_wen = %b \n imem_wen = %b \nexpected_imem_wen = %b \nuart_wen = %b \nexpected_uart_wen = %b \nload_mask = %b \nexpected_load_mask=%b \nuart_wen = %b \nexpected_uart_wen = %b \nnop = %b \nexpected_nop = %b \nwr_x0 = %b \nexpected_wr_x0 = %b \ndmem_sel = %b \nexpected_dmem_sel = %b \ncsr_cen = %b \nexpected_csr_cen = %b \nreg_wen = %b \nexpected_reg_wen = %b\n wa=%b\n expected_wa=%b\n", next_mem_phase_control_signals, expected_next_mem_phase_control_signals, ex_hazard_signals, expected_ex_hazard_signals, imm_sel, expected_imm_sel, br_un, expected_br_un, a_sel, expected_a_sel, b_sel, expected_b_sel, alu_sel, expected_alu_sel, wb_sel, expected_wb_sel, imem_sel, expected_imem_sel, pc_sel, expected_pc_sel, a_fwd, expected_a_fwd, b_fwd, expected_b_fwd, dmem_wen, expected_dmem_wen, imem_wen, expected_imem_wen, uart_wen, expected_uart_wen, load_mask, expected_load_mask, uart_wen, expected_uart_wen, nop, expected_nop, wr_x0, expected_wr_x0, dmem_sel, expected_dmem_sel, csr_cen, expected_csr_cen,reg_wen, expected_reg_wen, wa, expected_wa);
		$display("FAILED");
		$finish();
	end

  end endtask

  integer i;

    controller ctrler(
	.wa(wa),
	.inst(inst),
	.rst(rst), 
	.br_eq(br_eq),
	.br_lt(br_lt), 
	.pc(pc), 
	.alu_data(alu_data),
	.mem_phase_inst(mem_phase_inst),
	.mem_hazard_signals(mem_hazard_signals),
	.curr_mem_phase_control_signals(curr_mem_phase_control_signals),
	.next_mem_phase_control_signals(next_mem_phase_control_signals),
	.ex_hazard_signals(ex_hazard_signals),
	.imm_sel(imm_sel),
	.br_un(br_un), 
	.a_sel(a_sel), 
	.b_sel(b_sel),
	.alu_sel(alu_sel),
  	.wb_sel(wb_sel),
	.imem_sel(imem_sel),
	.pc_sel(pc_sel),
	.a_fwd(a_fwd),
	.b_fwd(b_fwd),
	.dmem_wen(dmem_wen),
	.imem_wen(imem_wen),
	.uart_wen(uart_wen),
	.load_mask(load_mask),
	.uart_wen(uart_wen),
	.nop(nop),
	.wr_x0(wr_x0),
	.dmem_sel(dmem_sel),
	.csr_cen(csr_cen),
	.reg_wen(reg_wen),
	.signed_load(signed_load)
);

  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
    
	// testing basic i type
	// addi x1 x0 1, no hazard
    $display("testing basic i");
	rst = 0; 
	br_eq = 1;
	br_lt = 0; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {{11{1'b0}}, 1'b1, 5'b00011,3'b000, 5'b00001, 7'b0010011}; // addi x1 x3 1, no hazard
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    
	expected_wa = 5'd0;
	expected_next_mem_phase_control_signals = {5'b00001, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_ex_hazard_signals = 5'b01000;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;
    
    #2;#2;#2;
    check;
	$display("passed basic i");

	// addi x0 x0 1, no hazard
	// testing wr_x0
    $display("testing basic i with wr_x0");
	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {{11{1'b0}}, 1'b1, 5'b00011,3'b000, 5'b00000, 7'b0010011}; // addi x0 x3 1, no hazard
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00000, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed basic i with wr_x0");

	// testing r type
	// add x1 x2 x3, no hazard
    $display("testing basic r");
	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {{7{1'b0}}, 5'b00011, 5'b00010, 3'b000,  5'b00001, 7'b0110011};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00001, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b0;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b0; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed basic r");

	// testing ori
	// ori x1 x0 1, no hazard
    $display("testing basic ori ");
	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {11'd0,1'b1, 5'b00000, 3'b110, 5'b00001, 7'b0010011};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00001, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0110; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b0; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed basic ori");


	// testing srai type
	// srai x1 x0 1, no hazard
    $display("testing basic srai ");

	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst =  {7'b0100000, 5'b00011, 5'b00001, 3'b101, 5'b00001, 7'b0010011};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00001, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b111;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b1101; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b0; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed basic srai");

	// testing load word
	// lw x1 1(x2), no hazard
    $display("testing basic load word");

	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {12'd1, 5'b00011, 3'b010, 5'b00001, 7'b0000011};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00001, 32'hffffffff, 1'b0, 2'b00, 2'b00, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed basic load word");

	// testing load signed half word
	// lw x1 1(x2), no hazard
    $display("testing load signed half word");

	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {12'd1, 5'b00011, 3'b001, 5'b00001, 7'b0000011};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00001, 32'h0000ffff, 1'b1, 2'b00, 2'b00, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'h00000000; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed load signed half word");


	// testing store 
	// sw x1 0(x3) no hazard
    $display("testing store word");
	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {7'd0, 5'b00001, 5'b00011, 3'b010, 5'b00000, 7'b0100011};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00000, 32'd0, 1'b0, 2'b00, 2'b00, 1'b0, 1'b0, 1'b0};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b001;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed store word");

	// testing branch
	// beq x1 x2 label , no hazard
    $display("testing branch");

	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'b00000, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {7'd0, 5'b00011, 5'b00011, 3'b010, 5'b00000, 7'b1100011};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00000, 32'd0, 1'b0, 2'b00, 2'b00, 1'b0, 1'b0, 1'b0};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b010;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b1; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed basic branch");

	// testing u
	// auipc x1 x2 label , no hazard
    $display("testing auipc");	rst = 0; 
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};;
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {20'd0, 5'b00011, 7'b0010111};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00011, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b011;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b1; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed basic auipc");

	// testing u
	// auipc x1 x2 label , no hazard
    $display("testing lui");
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {20'd0, 5'b00011, 7'b0110111};
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;
    

	expected_next_mem_phase_control_signals = {5'b00011, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b011;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b1111; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed basic lui");

	// testing j
	// jal x1 label, no hazard
    $display("testing jal");
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b1, 1'b0, 1'b1};
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {20'd0, 5'b00001, 7'b1101111}; 
    mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // addi x0 x0 0
    pc = 32'h4000_0000;

	expected_next_mem_phase_control_signals = {5'b00001, 32'd0, 1'b0, 2'b01, 2'b00, 1'b0, 1'b0, 1'b0};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b100;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b1; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b1; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed jal");


	// testing load hazard
	// lw x3 0(x1)
	// addi x2 x3 0

	// only tests inserting nop
    $display("testing load hazard inserting nop");
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011};
	mem_hazard_signals = 5'd0; 
	curr_mem_phase_control_signals  = {5'b00011, 32'd0, 1'b0, 2'b00, 2'b00, 1'b0, 1'b0, 1'b1};
    alu_data = 32'd0; 
    rst = 0; 
    mem_hazard_signals = 5'd0;
    inst = {12'd0, 5'b00011, 3'b000, 5'b00010, 7'b0010011};
    mem_phase_inst = {{7{1'b0}}, 5'b00001, 3'b010, 5'b00011, 7'b0000011};
    pc = 32'h4000_0000;

	expected_next_mem_phase_control_signals = {5'b00010, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'b00011;
	expected_ex_hazard_signals = 5'b10011;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b00;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b10; // no it should be 10
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b1;
	expected_wr_x0 = 1'b0; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed load hazard inserting nop");

	// testing load hazard
	// lw x3 0(x1)
	// addi x2 x3 0 (turned to nop)
	// addi x2 x3 0 
	// only tests inserting nop
    $display("testing load hazard after inserting nop");
	br_eq = 0;
	br_lt = 1; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_phase_inst = {{24{1'b0}}, 7'b0010011}; // nop
	mem_hazard_signals = 5'b10011;
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
    alu_data = 32'd0; 
    rst = 0; 
    inst = {12'd0, 5'b00011, 3'b000, 5'b00010, 7'b0010011}; // addi x2 x3 0 
    pc = 32'h4000_0000;

	expected_next_mem_phase_control_signals = {5'b00010, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b10000;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b1;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b01;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b00;
	expected_a_fwd = 2'b01;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b0;
	expected_wr_x0 = 1'b0; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;
    
    #2;#2;#2;
    check;
	$display("passed load hazard after inserting nop");

	// testing taken branch hazard
	// beq x3 x1 label
	// addi x2 x3 0
    $display("testing taken branch hazard");
	br_eq = 1;
	br_lt = 0; 
	pc = 32'h4000_0000; 
	alu_data = 32'd0;
	mem_hazard_signals = 5'd01000; 
	curr_mem_phase_control_signals  = {5'd0, 32'd0, 1'b0, 2'b00, 2'b00, 1'b0, 1'b0, 1'b0};
    alu_data = 32'd0; 
    rst = 0; 
    inst = {12'd0, 5'b00011, 3'b000, 5'b00010, 7'b0010011};
    mem_phase_inst = {7'd0, 5'b00011, 5'b00001, 3'b000, 5'd0, 7'b1100011};
    pc = 32'h4000_0000;

	expected_next_mem_phase_control_signals = {5'b00010, 32'd0, 1'b0, 2'b00, 2'b01, 1'b0, 1'b0, 1'b1};
	expected_wa = 5'd0;
	expected_ex_hazard_signals = 5'b01001;
	expected_imm_sel = 3'b000;
	expected_reg_wen = 1'b0;
	expected_br_un = 1'b0; 
	expected_a_sel = 1'b0; 
	expected_b_sel = 1'b1;
	expected_alu_sel = 4'b0000; 
  	expected_wb_sel = 2'b00;
	expected_imem_sel = 1'b1; 
	expected_pc_sel = 2'b01;
	expected_a_fwd = 2'b00;
	expected_b_fwd = 2'b00;
	expected_dmem_wen = 4'b0000;
	expected_imem_wen = 4'b0000;
	expected_uart_wen = 4'b0000;
	expected_load_mask = 32'd0; 
	expected_uart_wen = 4'b0000;
	expected_nop = 1'b1;
	expected_wr_x0 = 1'b0; 
	expected_dmem_sel = 2'b01; 
	expected_csr_cen = 1'b0;
	expected_signed_load = 1'b0;

    #2;#2;#2;
    check;
	$display("passed taken branch hazard");

	$display("PASS");

    $finish();
  end
endmodule