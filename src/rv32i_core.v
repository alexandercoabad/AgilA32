// rv32i_core.v -- minimal multi-cycle RV32I core for Tiny Tapeout

`default_nettype none
`include "rv32i_defs.vh"

module rv32i_core (
    input  wire        clk,
    input  wire        rst_n,

    // memory-mapped bus
    output reg  [7:0]  mem_addr,
    output reg  [31:0] mem_wdata,
    output reg  [1:0]  mem_size,
    output reg         mem_we,
    output reg         mem_valid,
    input  wire        mem_ready,
    input  wire [31:0] mem_rdata,

    output wire        halted
);

    reg [2:0]  state;
    reg [7:0]  pc;
    reg [31:0] ir;          // latched instruction
    reg [31:0] alu_result;
    reg [31:0] load_data;   // latched memory read data

    // ------------------------------------------------------------
    // Register file (x0 hardwired to 0)
    // ------------------------------------------------------------
    (* mem2reg *) reg [31:0] regs [1:31];
    integer i;
`ifndef SYNTHESIS
    initial for (i = 1; i <= 31; i = i + 1) regs[i] = 32'h0;
`endif

    wire [4:0] rd     = ir[11:7];
    wire [4:0] rs1    = ir[19:15];
    wire [4:0] rs2    = ir[24:20];
    wire [2:0] funct3 = ir[14:12];
    wire funct7_b5    = ir[30];
    wire [6:0] opcode = ir[6:0];

    wire is_ebreak = (opcode == `OP_SYSTEM) && (funct3 == 3'b000) &&
                      (ir[31:20] == 12'h001);

    assign halted = (state == `ST_HALTED);

    wire [31:0] rs1_val = (rs1 == 5'd0) ? 32'h0 : regs[rs1];
    wire [31:0] rs2_val = (rs2 == 5'd0) ? 32'h0 : regs[rs2];

    // ------------------------------------------------------------
    // Immediate decode
    // ------------------------------------------------------------
    wire [31:0] imm_i = {{20{ir[31]}}, ir[31:20]};
    wire [31:0] imm_s = {{20{ir[31]}}, ir[31:25], ir[11:7]};
    wire [31:0] imm_u = {ir[31:12], 12'b0};
    wire [7:0]  imm_b = {ir[27], ir[26], ir[25], ir[11], ir[10], ir[9], ir[8], 1'b0};
    wire [7:0]  imm_j = {ir[27], ir[26], ir[25], ir[24], ir[23], ir[22], ir[21], 1'b0};

    // ------------------------------------------------------------
    // ALU
    // ------------------------------------------------------------
    reg [31:0] alu_a, alu_b;
    reg [3:0]  alu_op;
    reg [31:0] alu_y;

    always @(*) begin
        case (alu_op)
            4'd0: alu_y = alu_a + alu_b;
            4'd1: alu_y = alu_a - alu_b;
            4'd2: alu_y = alu_a << alu_b[4:0];
            4'd3: alu_y = ($signed(alu_a) < $signed(alu_b)) ? 32'd1 : 32'd0;
            4'd4: alu_y = (alu_a < alu_b) ? 32'd1 : 32'd0;
            4'd5: alu_y = alu_a ^ alu_b;
            4'd6: alu_y = alu_a >> alu_b[4:0];
            4'd7: alu_y = $signed(alu_a) >>> alu_b[4:0];
            4'd8: alu_y = alu_a | alu_b;
            4'd9: alu_y = alu_a & alu_b;
            default: alu_y = 32'h0;
        endcase
    end

    always @(*) begin
        alu_a = rs1_val;
        alu_b = rs2_val;
        alu_op = 4'd0;
        case (opcode)
            `OP_LUI: begin
                alu_a = 32'h0;
                alu_b = imm_u;
                alu_op = 4'd0;
            end
            `OP_AUIPC: begin
                alu_a = {24'b0, pc};
                alu_b = imm_u;
                alu_op = 4'd0;
            end
            `OP_IMM: begin
                alu_b = imm_i;
                case (funct3)
                    3'b000: alu_op = 4'd0;
                    3'b010: alu_op = 4'd3;
                    3'b011: alu_op = 4'd4;
                    3'b100: alu_op = 4'd5;
                    3'b110: alu_op = 4'd8;
                    3'b111: alu_op = 4'd9;
                    3'b001: alu_op = 4'd2;
                    3'b101: alu_op = funct7_b5 ? 4'd7 : 4'd6;
                    default: alu_op = 4'd0;
                endcase
            end
            `OP_REG: begin
                case (funct3)
                    3'b000: alu_op = funct7_b5 ? 4'd1 : 4'd0;
                    3'b001: alu_op = 4'd2;
                    3'b010: alu_op = 4'd3;
                    3'b011: alu_op = 4'd4;
                    3'b100: alu_op = 4'd5;
                    3'b101: alu_op = funct7_b5 ? 4'd7 : 4'd6;
                    3'b110: alu_op = 4'd8;
                    3'b111: alu_op = 4'd9;
                    default: alu_op = 4'd0;
                endcase
            end
            `OP_LOAD, `OP_STORE: begin
                alu_a = rs1_val;
                alu_b = (opcode == `OP_LOAD) ? imm_i : imm_s;
                alu_op = 4'd0;
            end
            `OP_BRANCH: begin
                alu_a = rs1_val;
                alu_b = rs2_val;
                alu_op = 4'd1;
            end
            default: begin
                alu_a = rs1_val;
                alu_b = rs2_val;
                alu_op = 4'd0;
            end
        endcase
    end

    reg branch_cond;
    always @(*) begin
        case (funct3)
            3'b000: branch_cond = (rs1_val == rs2_val);
            3'b001: branch_cond = (rs1_val != rs2_val);
            3'b100: branch_cond = ($signed(rs1_val) <  $signed(rs2_val));
            3'b101: branch_cond = ($signed(rs1_val) >= $signed(rs2_val));
            3'b110: branch_cond = (rs1_val < rs2_val);
            3'b111: branch_cond = (rs1_val >= rs2_val);
            default: branch_cond = 1'b0;
        endcase
    end

    // Next PC calculation
    reg [7:0] next_pc_calc;
    always @(*) begin
        case (opcode)
            `OP_JAL:    next_pc_calc = pc + imm_j;
            `OP_JALR:   next_pc_calc = (rs1_val[7:0] + imm_i[7:0]) & 8'hFE;
            `OP_BRANCH: next_pc_calc = branch_cond ? (pc + imm_b) : (pc + 8'd4);
            default:    next_pc_calc = pc + 8'd4;
        endcase
    end

    // ------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= `ST_FETCH;
            pc        <= 8'h00;
            ir        <= 32'h0;
            mem_we    <= 1'b0;
            mem_addr  <= 8'h0;
            mem_size  <= 2'd2;
            mem_wdata <= 32'h0;
            mem_valid <= 1'b0;
            load_data <= 32'h0;
        end else begin
            case (state)
                `ST_FETCH: begin
                    mem_addr  <= pc;
                    mem_we    <= 1'b0;
                    mem_size  <= 2'd2;
                    mem_valid <= 1'b1;
                    state     <= `ST_FETCH_WAIT;
                end

                `ST_FETCH_WAIT: begin
                    if (mem_ready) begin
                        ir        <= mem_rdata;
                        mem_valid <= 1'b0;
                        state     <= `ST_DECODE;
                    end
                end

                `ST_DECODE: begin
                    state <= `ST_EXEC;
                end

                `ST_EXEC: begin
                    if (is_ebreak) begin
                        mem_we    <= 1'b0;
                        mem_valid <= 1'b0;
                        state     <= `ST_HALTED;
                    end else begin
                        alu_result <= alu_y;
                        if (opcode == `OP_LOAD || opcode == `OP_STORE) begin
                            state <= `ST_MEM;
                        end else begin
                            state <= `ST_WB;
                        end
                    end
                end

                `ST_MEM: begin
                    case (opcode)
                        `OP_LOAD: begin
                            mem_addr  <= alu_result[7:0];
                            mem_we    <= 1'b0;
                            mem_size  <= (funct3[1:0] == 2'b00) ? 2'd0 :
                                         (funct3[1:0] == 2'b01) ? 2'd1 : 2'd2;
                            mem_valid <= 1'b1;
                        end
                        `OP_STORE: begin
                            mem_addr  <= alu_result[7:0];
                            mem_wdata <= rs2_val;
                            mem_we    <= 1'b1;
                            mem_size  <= (funct3[1:0] == 2'b00) ? 2'd0 :
                                         (funct3[1:0] == 2'b01) ? 2'd1 : 2'd2;
                            mem_valid <= 1'b1;
                        end
                        default: begin
                            mem_we    <= 1'b0;
                            mem_valid <= 1'b0;
                        end
                    endcase
                    state <= `ST_MEM_WAIT;
                end

                `ST_MEM_WAIT: begin
                    if (mem_ready) begin
                        load_data <= mem_rdata;
                        mem_valid <= 1'b0;
                        mem_we    <= 1'b0;
                        state     <= `ST_WB;
                    end
                end

                `ST_WB: begin
                    mem_we    <= 1'b0;
                    mem_valid <= 1'b0;
                    if (rd != 5'd0) begin
                        case (opcode)
                            `OP_LUI:   regs[rd] <= imm_u;
                            `OP_AUIPC: regs[rd] <= alu_result;
                            `OP_JAL:   regs[rd] <= {24'b0, pc} + 32'd4;
                            `OP_JALR:  regs[rd] <= {24'b0, pc} + 32'd4;
                            `OP_LOAD: begin
                                case (funct3)
                                    3'b000: regs[rd] <= {{24{load_data[7]}},  load_data[7:0]};
                                    3'b001: regs[rd] <= {{16{load_data[15]}}, load_data[15:0]};
                                    3'b010: regs[rd] <= load_data;
                                    3'b100: regs[rd] <= {24'b0, load_data[7:0]};
                                    3'b101: regs[rd] <= {16'b0, load_data[15:0]};
                                    default: regs[rd] <= load_data;
                                endcase
                            end
                            `OP_IMM, `OP_REG: regs[rd] <= alu_result;
                            default: ;
                        endcase
                    end
                    pc    <= next_pc_calc;
                    state <= `ST_FETCH;
                end

                `ST_HALTED: begin
                    mem_valid <= 1'b0;
                    mem_we    <= 1'b0;
                    state     <= `ST_HALTED;
                end

                default: state <= `ST_FETCH;
            endcase
        end
    end

endmodule
