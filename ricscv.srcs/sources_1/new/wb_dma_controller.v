`timescale 1ns/1ps

//=============================================================================
// ASTRA-X1 SoC: Autonomous Wishbone B4 DMA Controller (Milestone 5B)
// Base Address: 0x4000_1000 (Slave 3 on Wishbone Interconnect)
// Master Port : Master 1 on Wishbone Interconnect
//
// Register Map (Offset from 0x4000_1000):
//   0x0000 [RO] DMA_REG_ID         : 32'h444D_4131 ("DMA1")
//   0x0004 [RO] DMA_REG_STATUS     : [0]=Ready (!busy), [1]=Done, [2]=Error
//   0x0008 [RW] DMA_REG_SRC_ADDR   : 32-bit Source Base Address
//   0x000C [RW] DMA_REG_DST_ADDR   : 32-bit Destination Base Address
//   0x0010 [RW] DMA_REG_LEN_WORDS  : Number of 32-bit words to transfer
//   0x0014 [RW] DMA_REG_CTRL       : [0]=Start pulse, [1]=Clear status
//   0x0018 [RO] DMA_REG_CYCLES     : Hardware clock cycles for last transfer
//   0x001C [RO] DMA_REG_WORDS_DONE : Words transferred count
//=============================================================================

module wb_dma_controller (
    input  wire        clk,
    input  wire        reset,

    // ========================================================================
    // Wishbone Slave Interface (CPU MMIO Control/Status @ 0x4000_1000)
    // ========================================================================
    input  wire [31:0] s_wb_adr_i,
    input  wire [31:0] s_wb_dat_i,
    input  wire [3:0]  s_wb_sel_i,
    input  wire        s_wb_we_i,
    input  wire        s_wb_stb_i,
    input  wire        s_wb_cyc_i,
    output reg  [31:0] s_wb_dat_o,
    output reg         s_wb_ack_o,

    // ========================================================================
    // Wishbone Master Interface (Autonomous Memory/Accelerator Transfers)
    // ========================================================================
    output reg  [31:0] m_wb_adr_o,
    output reg  [31:0] m_wb_dat_o,
    output wire [3:0]  m_wb_sel_o,
    output reg         m_wb_we_o,
    output reg         m_wb_stb_o,
    output reg         m_wb_cyc_o,
    input  wire [31:0] m_wb_dat_i,
    input  wire        m_wb_ack_i,
    input  wire        m_wb_err_i,

    // Completion Interrupt & Activity Status
    output reg         dma_irq,
    output wire        dma_busy_o
);

    wire _unused = &{1'b0, s_wb_adr_i[31:6], s_wb_adr_i[1:0], s_wb_sel_i};

    localparam [31:0] DMA_ID_CONST = 32'h444D_4131; // "DMA1"

    // MMIO Registers
    reg [31:0] reg_src_addr;
    reg [31:0] reg_dst_addr;
    reg [31:0] reg_len_words;
    reg [31:0] reg_cycles;
    reg [31:0] reg_words_done;
    reg        reg_busy;
    reg        reg_done;
    reg        reg_err;
    reg        start_pulse;
    reg        clr_pulse;

    assign dma_busy_o = reg_busy;

    // Slave Request Detection
    wire s_is_req = s_wb_cyc_i && s_wb_stb_i && !s_wb_ack_o;
    wire [3:0] reg_idx = s_wb_adr_i[5:2];

    // ========================================================================
    // Wishbone Slave MMIO Read / Write Logic
    // ========================================================================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            s_wb_ack_o    <= 1'b0;
            s_wb_dat_o    <= 32'd0;
            reg_src_addr  <= 32'd0;
            reg_dst_addr  <= 32'd0;
            reg_len_words <= 32'd0;
            start_pulse   <= 1'b0;
            clr_pulse     <= 1'b0;
        end else begin
            start_pulse <= 1'b0;
            clr_pulse   <= 1'b0;

            if (s_is_req) begin
                s_wb_ack_o <= 1'b1;
                if (s_wb_we_i) begin
                    case (reg_idx)
                        4'h2: reg_src_addr  <= s_wb_dat_i; // 0x08: SRC_ADDR
                        4'h3: reg_dst_addr  <= s_wb_dat_i; // 0x0C: DST_ADDR
                        4'h4: reg_len_words <= s_wb_dat_i; // 0x10: LEN_WORDS
                        4'h5: begin                        // 0x14: CTRL
                            if (s_wb_dat_i[0] && !reg_busy) start_pulse <= 1'b1;
                            if (s_wb_dat_i[1])              clr_pulse   <= 1'b1;
                        end
                        default: ;
                    endcase
                end else begin
                    case (reg_idx)
                        4'h0: s_wb_dat_o <= DMA_ID_CONST;                             // 0x00: "DMA1"
                        4'h1: s_wb_dat_o <= {29'd0, reg_err, reg_done, !reg_busy};    // 0x04: STATUS
                        4'h2: s_wb_dat_o <= reg_src_addr;                             // 0x08: SRC_ADDR
                        4'h3: s_wb_dat_o <= reg_dst_addr;                             // 0x0C: DST_ADDR
                        4'h4: s_wb_dat_o <= reg_len_words;                            // 0x10: LEN_WORDS
                        4'h5: s_wb_dat_o <= {31'd0, reg_busy};                        // 0x14: CTRL
                        4'h6: s_wb_dat_o <= reg_cycles;                               // 0x18: CYCLES
                        4'h7: s_wb_dat_o <= reg_words_done;                           // 0x1C: WORDS_DONE
                        default: s_wb_dat_o <= 32'd0;
                    endcase
                end
            end else begin
                s_wb_ack_o <= 1'b0;
            end
        end
    end

    // ========================================================================
    // Autonomous DMA Master FSM
    // ========================================================================
    localparam [1:0] FSM_IDLE  = 2'd0;
    localparam [1:0] FSM_READ  = 2'd1;
    localparam [1:0] FSM_WRITE = 2'd2;

    reg [1:0]  fsm_state;
    reg [31:0] curr_src;
    reg [31:0] curr_dst;
    reg [31:0] rem_words;
    reg [31:0] data_buf;
    reg [31:0] cycle_timer;

    assign m_wb_sel_o = 4'b1111;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            fsm_state      <= FSM_IDLE;
            curr_src       <= 32'd0;
            curr_dst       <= 32'd0;
            rem_words      <= 32'd0;
            data_buf       <= 32'd0;
            cycle_timer    <= 32'd0;
            reg_cycles     <= 32'd0;
            reg_words_done <= 32'd0;
            reg_busy       <= 1'b0;
            reg_done       <= 1'b0;
            reg_err        <= 1'b0;
            m_wb_adr_o     <= 32'd0;
            m_wb_dat_o     <= 32'd0;
            m_wb_we_o      <= 1'b0;
            m_wb_stb_o     <= 1'b0;
            m_wb_cyc_o     <= 1'b0;
            dma_irq        <= 1'b0;
        end else begin
            dma_irq <= 1'b0;

            if (clr_pulse && !reg_busy) begin
                reg_done <= 1'b0;
                reg_err  <= 1'b0;
            end

            case (fsm_state)
                FSM_IDLE: begin
                    m_wb_cyc_o <= 1'b0;
                    m_wb_stb_o <= 1'b0;
                    m_wb_we_o  <= 1'b0;
                    reg_busy   <= 1'b0;

                    if (start_pulse) begin
                        reg_words_done <= 32'd0;
                        reg_err        <= 1'b0;
                        if (reg_len_words == 32'd0) begin
                            reg_done   <= 1'b1;
                            reg_cycles <= 32'd0;
                        end else begin
                            curr_src    <= reg_src_addr;
                            curr_dst    <= reg_dst_addr;
                            rem_words   <= reg_len_words;
                            cycle_timer <= 32'd0;
                            reg_busy    <= 1'b1;
                            reg_done    <= 1'b0;

                            // Issue first read request immediately
                            m_wb_adr_o  <= reg_src_addr;
                            m_wb_we_o   <= 1'b0;
                            m_wb_cyc_o  <= 1'b1;
                            m_wb_stb_o  <= 1'b1;
                            fsm_state   <= FSM_READ;
                        end
                    end
                end

                FSM_READ: begin
                    cycle_timer <= cycle_timer + 1'b1;

                    if (m_wb_err_i) begin
                        m_wb_cyc_o <= 1'b0;
                        m_wb_stb_o <= 1'b0;
                        reg_err    <= 1'b1;
                        reg_busy   <= 1'b0;
                        reg_done   <= 1'b1;
                        reg_cycles <= cycle_timer + 1'b1;
                        fsm_state  <= FSM_IDLE;
                    end else if (m_wb_ack_i) begin
                        // Latch read word and immediately launch write transaction
                        data_buf   <= m_wb_dat_i;
                        curr_src   <= curr_src + 32'd4;

                        m_wb_adr_o <= curr_dst;
                        m_wb_dat_o <= m_wb_dat_i;
                        m_wb_we_o  <= 1'b1;
                        m_wb_cyc_o <= 1'b1;
                        m_wb_stb_o <= 1'b1;
                        fsm_state  <= FSM_WRITE;
                    end
                end

                FSM_WRITE: begin
                    cycle_timer <= cycle_timer + 1'b1;

                    if (m_wb_err_i) begin
                        m_wb_cyc_o <= 1'b0;
                        m_wb_stb_o <= 1'b0;
                        reg_err    <= 1'b1;
                        reg_busy   <= 1'b0;
                        reg_done   <= 1'b1;
                        reg_cycles <= cycle_timer + 1'b1;
                        fsm_state  <= FSM_IDLE;
                    end else if (m_wb_ack_i) begin
                        curr_dst       <= curr_dst + 32'd4;
                        reg_words_done <= reg_words_done + 1'b1;

                        if (rem_words == 32'd1) begin
                            // Transfer complete!
                            m_wb_cyc_o <= 1'b0;
                            m_wb_stb_o <= 1'b0;
                            m_wb_we_o  <= 1'b0;
                            reg_busy   <= 1'b0;
                            reg_done   <= 1'b1;
                            reg_cycles <= cycle_timer + 1'b1;
                            dma_irq    <= 1'b1;
                            fsm_state  <= FSM_IDLE;
                        end else begin
                            // Advance to next word read
                            rem_words  <= rem_words - 1'b1;
                            m_wb_adr_o <= curr_src;
                            m_wb_we_o  <= 1'b0;
                            m_wb_cyc_o <= 1'b1;
                            m_wb_stb_o <= 1'b1;
                            fsm_state  <= FSM_READ;
                        end
                    end
                end

                default: fsm_state <= FSM_IDLE;
            endcase
        end
    end

endmodule
