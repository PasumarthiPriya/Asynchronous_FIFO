`timescale 1ns / 1ps
//======================================================================
//  tb_sync_fifo.v   --   self-checking testbench for sync_fifo.v
//
//  Structure:
//    1. A reference model (a circular buffer written in plain Verilog)
//       that mirrors what the FIFO *should* contain.
//    2. One low-level task, fifo_op(), that drives exactly one clock
//       cycle of activity and checks the DUT against the model.
//    3. A sequence of directed tests, then a random traffic test.
//
//  Everything is driven on the NEGATIVE clock edge so that signals are
//  stable well before the DUT samples them on the positive edge.  This
//  avoids race conditions between the testbench and the design.
//======================================================================

module tb_sync_fifo;

    localparam DATA_WIDTH = 8;
    localparam ADDR_WIDTH = 4;
    localparam DEPTH      = 1 << ADDR_WIDTH;

    // ---------------- DUT connections ----------------
    reg                    clk;
    reg                    rst_n;
    reg                    wr_en;
    reg  [DATA_WIDTH-1:0]  wr_data;
    reg                    rd_en;
    wire [DATA_WIDTH-1:0]  rd_data;
    wire                   full;
    wire                   empty;

    sync_fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (wr_en),
        .wr_data (wr_data),
        .full    (full),
        .rd_en   (rd_en),
        .rd_data (rd_data),
        .empty   (empty)
    );

    // ---------------- clock: 100 MHz ----------------
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ---------------- bookkeeping ----------------
    integer errors;
    integer checks;

    // ==================================================================
    //  REFERENCE MODEL
    //
    //  Plain Verilog has no queues, so the scoreboard is a circular
    //  buffer with its own pointers and an occupancy counter.  This is
    //  completely independent of the DUT -- that independence is what
    //  makes the comparison meaningful.
    // ==================================================================
    reg [DATA_WIDTH-1:0] ref_mem [0:DEPTH-1];
    integer ref_wp;
    integer ref_rp;
    integer ref_cnt;

    task ref_reset;
        begin
            ref_wp  = 0;
            ref_rp  = 0;
            ref_cnt = 0;
        end
    endtask

    task ref_push(input [DATA_WIDTH-1:0] d);
        begin
            ref_mem[ref_wp] = d;
            ref_wp  = (ref_wp + 1) % DEPTH;
            ref_cnt = ref_cnt + 1;
        end
    endtask

    task ref_pop(output [DATA_WIDTH-1:0] d);
        begin
            d = ref_mem[ref_rp];
            ref_rp  = (ref_rp + 1) % DEPTH;
            ref_cnt = ref_cnt - 1;
        end
    endtask

    // ==================================================================
    //  CHECKERS
    // ==================================================================
    task check_flags;
        begin
            checks = checks + 1;
            if (empty !== (ref_cnt == 0)) begin
                errors = errors + 1;
                $display("[%0t] FLAG ERROR: empty=%b but model holds %0d entries",
                          $time, empty, ref_cnt);
            end
            if (full !== (ref_cnt == DEPTH)) begin
                errors = errors + 1;
                $display("[%0t] FLAG ERROR: full=%b but model holds %0d entries",
                          $time, full, ref_cnt);
            end
        end
    endtask

    // ==================================================================
    //  fifo_op : drive ONE cycle of FIFO activity and check it.
    //
    //  do_wr / do_rd may both be 1 -- that is the concurrent access case.
    //  The task deliberately checks the flags BEFORE the operation, so a
    //  blocked write (FIFO full) or blocked read (FIFO empty) is exercised
    //  and verified rather than avoided.
    // ==================================================================
    task fifo_op(input do_wr, input [DATA_WIDTH-1:0] d, input do_rd);
        reg [DATA_WIDTH-1:0] expected;
        begin
            @(negedge clk);
            wr_en   = do_wr;
            wr_data = d;
            rd_en   = do_rd;
            #1;                       // let combinational outputs settle

            check_flags;

            // --- read side: rd_data must already show the head entry ---
            if (do_rd && !empty) begin
                ref_pop(expected);
                checks = checks + 1;
                if (rd_data !== expected) begin
                    errors = errors + 1;
                    $display("[%0t] DATA ERROR: read 0x%02h, expected 0x%02h",
                              $time, rd_data, expected);
                end
            end

            // --- write side ---
            if (do_wr && !full)
                ref_push(d);

            @(posedge clk);           // the DUT samples here
        end
    endtask

    task idle(input integer n);
        integer i;
        begin
            @(negedge clk);
            wr_en = 1'b0;
            rd_en = 1'b0;
            for (i = 0; i < n; i = i + 1) @(posedge clk);
        end
    endtask

    task do_reset;
        begin
            rst_n = 1'b0;
            wr_en = 1'b0;
            rd_en = 1'b0;
            wr_data = {DATA_WIDTH{1'b0}};
            ref_reset;
            repeat (3) @(posedge clk);
            @(negedge clk);
            rst_n = 1'b1;
            @(posedge clk);
        end
    endtask

    // ==================================================================
    //  TEST SEQUENCE
    // ==================================================================
    integer i;
    reg [DATA_WIDTH-1:0] rnd;
    reg wr_rnd, rd_rnd;

    initial begin
        $dumpfile("tb_sync_fifo.vcd");
        $dumpvars(0, tb_sync_fifo);

        errors = 0;
        checks = 0;

        $display("\n=== sync_fifo testbench : DEPTH=%0d WIDTH=%0d ===\n",
                  DEPTH, DATA_WIDTH);

        // ---------- TEST 1: reset state ----------
        do_reset;
        if (empty !== 1'b1 || full !== 1'b0) begin
            errors = errors + 1;
            $display("[%0t] TEST1 FAIL: after reset expected empty=1 full=0, got empty=%b full=%b",
                      $time, empty, full);
        end else
            $display("TEST 1 pass : reset leaves FIFO empty");

        // ---------- TEST 2: fill completely ----------
        for (i = 0; i < DEPTH; i = i + 1)
            fifo_op(1'b1, i[DATA_WIDTH-1:0] + 8'hA0, 1'b0);
        idle(1);
        if (full !== 1'b1) begin
            errors = errors + 1;
            $display("[%0t] TEST2 FAIL: FIFO should be full after %0d writes", $time, DEPTH);
        end else
            $display("TEST 2 pass : full asserts after %0d writes", DEPTH);

        // ---------- TEST 3: overflow is blocked ----------
        for (i = 0; i < 4; i = i + 1)
            fifo_op(1'b1, 8'hEE, 1'b0);        // must all be ignored
        $display("TEST 3 pass : %0d writes to a full FIFO were ignored", 4);

        // ---------- TEST 4: drain completely, check order ----------
        for (i = 0; i < DEPTH; i = i + 1)
            fifo_op(1'b0, 8'h00, 1'b1);
        idle(1);
        if (empty !== 1'b1) begin
            errors = errors + 1;
            $display("[%0t] TEST4 FAIL: FIFO should be empty after draining", $time);
        end else
            $display("TEST 4 pass : all %0d entries read back in order", DEPTH);

        // ---------- TEST 5: underflow is blocked ----------
        for (i = 0; i < 4; i = i + 1)
            fifo_op(1'b0, 8'h00, 1'b1);        // must all be ignored
        $display("TEST 5 pass : %0d reads from an empty FIFO were ignored", 4);

        // ---------- TEST 6: concurrent read + write ----------
        for (i = 0; i < DEPTH/2; i = i + 1)        // half fill
            fifo_op(1'b1, 8'h10 + i[DATA_WIDTH-1:0], 1'b0);
        for (i = 0; i < 20; i = i + 1)             // read and write together
            fifo_op(1'b1, 8'h50 + i[DATA_WIDTH-1:0], 1'b1);
        $display("TEST 6 pass : 20 cycles of simultaneous read+write, depth held steady");

        // ---------- TEST 7: random traffic (covers pointer wraparound) ----------
        for (i = 0; i < 500; i = i + 1) begin
            wr_rnd = $random;
            rd_rnd = $random;
            rnd    = $random;
            fifo_op(wr_rnd, rnd, rd_rnd);
        end
        $display("TEST 7 pass : 500 randomised cycles");

        // ---------- drain and finish ----------
        while (!empty)
            fifo_op(1'b0, 8'h00, 1'b1);
        idle(2);

        $display("\n---------------------------------------------");
        $display("  checks performed : %0d", checks);
        $display("  errors found     : %0d", errors);
        if (errors == 0)
            $display("  RESULT: *** PASS ***");
        else
            $display("  RESULT: *** FAIL ***");
        $display("---------------------------------------------\n");

        $finish;
    end

    // safety net so the simulation can never hang
    initial begin
        #200000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
