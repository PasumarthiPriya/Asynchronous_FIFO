`timescale 1ns / 1ps
//======================================================================
//  stage3_tb.v   --   testbench for the asynchronous FIFO
//
//  Compile with stage2_design.v and stage3_design.v.
//
//  ---------------------------------------------------------------
//  HOW THIS TESTBENCH CHECKS CORRECTNESS
//
//  With two unrelated clocks you cannot write a scoreboard that knows
//  how many entries the FIFO holds at a given instant -- "a given
//  instant" is not a meaningful idea across asynchronous domains, and
//  the flags are deliberately pessimistic anyway.
//
//  So instead of checking occupancy, we check the property that
//  actually matters:
//
//      THE READER MUST SEE EXACTLY THE SEQUENCE THE WRITER SENT,
//      IN ORDER, WITH NOTHING LOST AND NOTHING DUPLICATED.
//
//  The writer sends an incrementing pattern.  The reader keeps its own
//  count of how many words it has taken and checks each arrival against
//  what that count predicts.  Any drop, duplicate or reordering breaks
//  the match immediately.  At the end we also confirm the totals agree.
//
//  Four traffic phases are run, at different clock ratios, because CDC
//  bugs tend to hide at one ratio and appear at another.
//  ---------------------------------------------------------------
//======================================================================

module stage3_tb;

    localparam DATA_WIDTH = 8;
    localparam ADDR_WIDTH = 4;
    localparam DEPTH      = 1 << ADDR_WIDTH;

    //=================================================================
    //  Clocks with run-time adjustable periods, so one simulation can
    //  sweep several write:read speed ratios.
    //=================================================================
    real whalf, rhalf;
    reg  wclk, rclk;

    // rclk starts with a deliberate phase offset so that even when the
    // two periods are equal the clocks are still unrelated -- edge-aligned
    // clocks would quietly hide exactly the bugs we are looking for.
    initial begin wclk = 1'b0;        forever #(whalf) wclk = ~wclk; end
    initial begin rclk = 1'b0; #1.3;  forever #(rhalf) rclk = ~rclk; end

    //=================================================================
    //  DUT
    //=================================================================
    reg                   arst_n;
    reg                   winc, rinc;
    wire [DATA_WIDTH-1:0] wdata;
    wire [DATA_WIDTH-1:0] rdata;
    wire                  wfull, rempty;

    async_fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) dut (
        .arst_n (arst_n),
        .wclk   (wclk),  .winc(winc), .wdata(wdata), .wfull (wfull),
        .rclk   (rclk),  .rinc(rinc), .rdata(rdata), .rempty(rempty)
    );

    //=================================================================
    //  Traffic generation and checking
    //=================================================================
    integer wr_count;          // words successfully written
    integer rd_count;          // words successfully read
    integer errors;
    reg     run_wr, run_rd;    // enable the two traffic generators
    reg     dense_wr, dense_rd;// 1 = every cycle, 0 = about half the cycles

    // the pattern being sent: simply the running write count
    assign wdata = wr_count[DATA_WIDTH-1:0];

    // ---- writer -----------------------------------------------------
    // The condition here must be EXACTLY the one the DUT uses to decide
    // whether a write happens: winc and not full.  It must NOT also test
    // run_wr.  run_wr can drop part-way through a cycle while winc is
    // still high, in which case the FIFO accepts a word that the
    // testbench would then forget to count -- and the totals would
    // disagree at the end for no real reason.
    always @(posedge wclk) begin
        if (winc && !wfull)
            wr_count = wr_count + 1;
    end

    always @(negedge wclk) begin
        if (!run_wr)      winc = 1'b0;
        else if (dense_wr) winc = 1'b1;
        else               winc = $random;      // random single bit
    end

    // ---- reader and checker -----------------------------------------
    always @(posedge rclk) begin
        if (rinc && !rempty) begin
            if (rdata !== rd_count[DATA_WIDTH-1:0]) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("  [%0t] DATA ERROR: word %0d came out as 0x%02h, expected 0x%02h",
                              $time, rd_count, rdata, rd_count[DATA_WIDTH-1:0]);
            end
            rd_count = rd_count + 1;
        end
    end

    always @(negedge rclk) begin
        if (!run_rd)      rinc = 1'b0;
        else if (dense_rd) rinc = 1'b1;
        else               rinc = $random;
    end

    // ---- invariant: the reader can never get ahead of the writer -----
    integer impossible_reported;
    always @(posedge rclk) begin
        if (rd_count > wr_count) begin
            errors = errors + 1;
            impossible_reported = impossible_reported + 1;
            if (impossible_reported <= 5)
                $display("  [%0t] IMPOSSIBLE: read %0d words but only %0d were written",
                          $time, rd_count, wr_count);
        end
    end

    //=================================================================
    //  Observability: convert the Gray pointers back to binary so the
    //  waveform is readable.  Purely for viewing -- not part of the DUT.
    //=================================================================
    wire [ADDR_WIDTH:0] wptr_bin_obs, rptr_bin_obs;
    gray2bin #(.WIDTH(ADDR_WIDTH+1)) u_obs_w (.gray(dut.wptr), .bin(wptr_bin_obs));
    gray2bin #(.WIDTH(ADDR_WIDTH+1)) u_obs_r (.gray(dut.rptr), .bin(rptr_bin_obs));

    //=================================================================
    //  Helper tasks
    //=================================================================
    task do_reset;
        begin
            run_wr = 1'b0;  run_rd = 1'b0;
            winc   = 1'b0;  rinc   = 1'b0;
            wr_count = 0;   rd_count = 0;
            arst_n = 1'b0;
            #100;
            arst_n = 1'b1;
            repeat (5) @(posedge wclk);
            repeat (5) @(posedge rclk);
        end
    endtask

    // let both sides run for a while, then stop writing and drain
    task run_phase(input [8*24:1] name,
                   input real     wp,      // write clock period, ns
                   input real     rp,      // read  clock period, ns
                   input          dw,      // dense writes?
                   input          dr,      // dense reads?
                   input integer  duration);
        integer before_err;
        begin
            before_err = errors;
            whalf = wp/2.0;
            rhalf = rp/2.0;
            dense_wr = dw;  dense_rd = dr;
            run_wr = 1'b1;  run_rd = 1'b1;

            #duration;

            run_wr = 1'b0;                  // stop writing, let it drain
            #(duration/2);
            run_rd = 1'b0;
            repeat (10) @(posedge rclk);

            $display("  %-24s wclk=%0.1fns rclk=%0.1fns  written=%0d read=%0d  %s",
                      name, wp, rp, wr_count, rd_count,
                      (errors == before_err) ? "OK" : "*** ERRORS ***");
        end
    endtask

    //=================================================================
    //  TEST SEQUENCE
    //=================================================================
    integer i;

    initial begin
        $dumpfile("stage3_tb.vcd");
        $dumpvars(0, stage3_tb);

        errors   = 0;
        impossible_reported = 0;
        whalf    = 5.0;
        rhalf    = 5.0;
        dense_wr = 1'b1;
        dense_rd = 1'b1;

        $display("\n================================================================");
        $display("  ASYNCHRONOUS FIFO  --  depth %0d, width %0d", DEPTH, DATA_WIDTH);
        $display("================================================================");

        //-------------------------------------------------------------
        $display("\n  TEST 1 : reset");
        do_reset;
        if (rempty !== 1'b1) begin
            errors = errors + 1;
            $display("    FAIL: rempty should be 1 after reset, got %b", rempty);
        end
        if (wfull !== 1'b0) begin
            errors = errors + 1;
            $display("    FAIL: wfull should be 0 after reset, got %b", wfull);
        end
        if (errors == 0)
            $display("    after reset: rempty=1, wfull=0  OK");

        //-------------------------------------------------------------
        $display("\n  TEST 2 : fill to full with no reading");
        run_wr = 1'b1;  dense_wr = 1'b1;
        run_rd = 1'b0;
        wait (wfull == 1'b1);
        run_wr = 1'b0;
        repeat (5) @(posedge wclk);
        if (wr_count !== DEPTH) begin
            errors = errors + 1;
            $display("    FAIL: full after %0d writes, expected %0d", wr_count, DEPTH);
        end else
            $display("    wfull asserted after exactly %0d writes  OK", DEPTH);

        //-------------------------------------------------------------
        $display("\n  TEST 3 : writes to a full FIFO are ignored");
        run_wr = 1'b1;
        repeat (20) @(posedge wclk);
        run_wr = 1'b0;
        repeat (5) @(posedge wclk);
        if (wr_count !== DEPTH) begin
            errors = errors + 1;
            $display("    FAIL: write count moved to %0d while full", wr_count);
        end else
            $display("    20 write attempts while full changed nothing  OK");

        //-------------------------------------------------------------
        $display("\n  TEST 4 : drain to empty, checking every word");
        run_rd = 1'b1;  dense_rd = 1'b1;
        wait (rempty == 1'b1);
        repeat (5) @(posedge rclk);
        run_rd = 1'b0;
        if (rd_count !== DEPTH) begin
            errors = errors + 1;
            $display("    FAIL: read %0d words, expected %0d", rd_count, DEPTH);
        end else
            $display("    all %0d words read back in order  OK", DEPTH);

        //-------------------------------------------------------------
        $display("\n  TEST 5 : reads from an empty FIFO are ignored");
        run_rd = 1'b1;
        repeat (20) @(posedge rclk);
        run_rd = 1'b0;
        repeat (5) @(posedge rclk);
        if (rd_count !== DEPTH) begin
            errors = errors + 1;
            $display("    FAIL: read count moved to %0d while empty", rd_count);
        end else
            $display("    20 read attempts while empty changed nothing  OK");

        //-------------------------------------------------------------
        $display("\n  TEST 6 : sustained traffic at several clock ratios");
        do_reset;

        run_phase("equal clocks",     10.0, 10.0, 1'b1, 1'b1, 3000);
        run_phase("fast write 6:14",   6.0, 14.0, 1'b1, 1'b1, 3000);
        run_phase("slow write 14:6",  14.0,  6.0, 1'b1, 1'b1, 3000);
        run_phase("awkward 7:3",       7.0,  3.0, 1'b1, 1'b1, 3000);
        run_phase("bursty 9:11",       9.0, 11.0, 1'b0, 1'b0, 4000);

        //-------------------------------------------------------------
        $display("\n  TEST 7 : final accounting");
        if (wr_count !== rd_count) begin
            errors = errors + 1;
            $display("    FAIL: %0d words written but %0d read -- data was lost",
                      wr_count, rd_count);
        end else
            $display("    %0d words written, %0d read, none lost or duplicated  OK",
                      wr_count, rd_count);

        //-------------------------------------------------------------
        $display("\n================================================================");
        $display("   total words transferred : %0d", rd_count);
        $display("   errors found            : %0d", errors);
        if (errors == 0)
            $display("   RESULT: *** PASS ***");
        else
            $display("   RESULT: *** FAIL ***");
        $display("================================================================\n");

        $finish;
    end

    initial begin
        #500000;
        $display("TIMEOUT -- something is stuck");
        $finish;
    end

endmodule
