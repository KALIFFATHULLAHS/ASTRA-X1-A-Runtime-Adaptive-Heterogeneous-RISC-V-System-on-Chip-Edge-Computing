`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: 7-Stage Pipeline Memory (MEM) Stage
// Features integrated Wishbone B4 Classic Master Adapter and Wait-State Stalls.
//=============================================================================

module mem_stage (
    input  wire        clk_mem,
    input  wire        reset,

    // From EX2
    input  wire        mem_valid_in,
    input  wire [31:0] ex2_result,
    input  wire [31:0] ex2_op_b,         // store data
    input  wire [31:0] ex2_pc,
    input  wire [4:0]  ex2_rd,
    input  wire [23:0] ex2_ctrl_bus,
    input  wire [31:0] mem_instr_in,
    
    // Wishbone B4 Master Interface
    output wire [31:0] wb_adr_o,
    output wire [31:0] wb_dat_o,
    output wire [3:0]  wb_sel_o,
    output wire        wb_we_o,
    output wire        wb_cyc_o,
    output wire        wb_stb_o,
    input  wire [31:0] wb_dat_i,
    input  wire        wb_ack_i,
    input  wire        wb_err_i,

    // Pipeline stall output to hazard unit & EX2
    output wire        stall_mem,

    // Pipeline output to WB
    output reg  [31:0] wb_instr,
    output reg         wb_valid,
    output reg  [31:0] wb_result,
    output reg  [31:0] wb_pc,
    output reg  [4:0]  wb_rd,
    output reg  [23:0] wb_ctrl_bus
);

    // Unpack control signals
    wire reg_write = ex2_ctrl_bus[15];
    wire mem_read  = ex2_ctrl_bus[14];
    wire mem_write = ex2_ctrl_bus[13];
    wire is_mem_op = mem_valid_in && (mem_read || mem_write);

    // Suppress unused warning
    wire _unused_reg_write = reg_write;

    // STORE DATA PACKING
    reg [31:0] store_data;
    reg [3:0]  store_strb;

    always @(*) begin
        store_data = 32'd0;
        store_strb = 4'b0000;

        case (ex2_ctrl_bus[22:20])   // funct3 for store
            3'b000: begin // SB
                store_strb = (4'b0001 << ex2_result[1:0]);
                store_data = {4{ex2_op_b[7:0]}};
            end
            3'b001: begin // SH
                store_strb = (ex2_result[1] == 1'b0) ? 4'b0011 : 4'b1100;
                store_data = {2{ex2_op_b[15:0]}};
            end
            3'b010: begin // SW
                store_strb = 4'b1111;
                store_data = ex2_op_b;
            end
            default: begin
                store_strb = 4'b0000;
                store_data = 32'd0;
            end
        endcase
    end

    // LOAD BYTE SELECT
    reg [3:0] load_sel;
    always @(*) begin
        case (ex2_ctrl_bus[22:20])
            3'b000, 3'b100: load_sel = (4'b0001 << ex2_result[1:0]);                  // Byte
            3'b001, 3'b101: load_sel = (ex2_result[1] == 1'b0) ? 4'b0011 : 4'b1100; // Halfword
            default:        load_sel = 4'b1111;                                      // Word
        endcase
    end

    // Wishbone Master Outputs
    assign wb_adr_o = mem_valid_in ? ex2_result : 32'd0;
    assign wb_dat_o = store_data;
    assign wb_sel_o = mem_write ? store_strb : load_sel;
    assign wb_we_o  = mem_write;
    assign wb_cyc_o = is_mem_op;
    assign wb_stb_o = is_mem_op;

    // Stall MEM and earlier stages while waiting for slave response
    assign stall_mem = is_mem_op && !wb_ack_i && !wb_err_i;

    // LOAD SIGN EXTENSION FROM WISHBONE DATA INPUT
    reg [31:0] load_data;

    always @(*) begin
        case (ex2_ctrl_bus[22:20])  // funct3 for load
            3'b000: begin // LB
                case (ex2_result[1:0])
                    2'b00: load_data = {{24{wb_dat_i[7]}},  wb_dat_i[7:0]};
                    2'b01: load_data = {{24{wb_dat_i[15]}}, wb_dat_i[15:8]};
                    2'b10: load_data = {{24{wb_dat_i[23]}}, wb_dat_i[23:16]};
                    default: load_data = {{24{wb_dat_i[31]}}, wb_dat_i[31:24]};
                endcase
            end
            3'b001: begin // LH
                if (ex2_result[1] == 1'b0)
                    load_data = {{16{wb_dat_i[15]}}, wb_dat_i[15:0]};
                else
                    load_data = {{16{wb_dat_i[31]}}, wb_dat_i[31:16]};
            end
            3'b010: begin // LW
                load_data = wb_dat_i;
            end
            3'b100: begin // LBU
                case (ex2_result[1:0])
                    2'b00: load_data = {24'd0, wb_dat_i[7:0]};
                    2'b01: load_data = {24'd0, wb_dat_i[15:8]};
                    2'b10: load_data = {24'd0, wb_dat_i[23:16]};
                    default: load_data = {24'd0, wb_dat_i[31:24]};
                endcase
            end
            3'b101: begin // LHU
                if (ex2_result[1] == 1'b0)
                    load_data = {16'd0, wb_dat_i[15:0]};
                else
                    load_data = {16'd0, wb_dat_i[31:16]};
            end
            default: load_data = 32'd0;
        endcase
    end

    // PIPELINE REGISTER (MEM → WB)
    always @(posedge clk_mem or posedge reset) begin
        if (reset) begin
            wb_valid    <= 1'b0;
            wb_pc       <= 32'd0;
            wb_rd       <= 5'd0;
            wb_ctrl_bus <= 24'd0;
            wb_instr    <= 32'd0;
            wb_result   <= 32'd0;
        end
        else if (stall_mem) begin
            // While waiting for Wishbone slave response, send bubble to WB
            wb_valid <= 1'b0;
        end
        else if (mem_valid_in) begin
            wb_valid    <= 1'b1;
            wb_pc       <= ex2_pc;
            wb_rd       <= ex2_rd;
            wb_ctrl_bus <= ex2_ctrl_bus;
            wb_instr    <= mem_instr_in;
            wb_result   <= (mem_read) ? load_data : ex2_result;
        end else begin
            wb_valid <= 1'b0;
        end
    end

endmodule
