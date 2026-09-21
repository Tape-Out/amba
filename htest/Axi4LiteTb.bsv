package Axi4LiteTb;

import StmtFSM::*;
import ConfigReg::*;
import RegIf::*;
import Axi4Lite::*;

// 零等待假外设：0x04 读写（按选通合并）· 0x08 读回写过几次（一笔 AXI 写只许动一次）· 0x0C 一律回错
module mkLitePeriph(RegIf#(8, 32));
  Reg#(Bit#(32)) v  <- mkReg(0);
  Reg#(Bit#(16)) wc <- mkReg(0);

  method ActionValue#(RegRsp#(32)) access(RegReq#(8, 32) r);
    Bool     er = r.addr == 8'h0C;
    Bit#(32) rd = r.addr == 8'h08 ? zeroExtend(wc) : v;
    if (r.write && !er) begin
      wc <= wc + 1;
      if (r.addr == 8'h04) v <= applyStrb(v, r.wdata, r.wstrb);
    end
    return RegRsp { rdata: rd, err: er };
  endmethod
endmodule

// 会停顿的同一个外设：收下之后压 n 拍才答。规则与 always_enabled 的方法同写这几个量，一律 CReg：
// 端口 0 归规则，端口 1 归方法
module mkLiteSlow#(Integer n)(RegTarget#(8, 32));
  Reg#(Bit#(8))        cnt[2]  <- mkCReg(2, 0);
  Reg#(Bool)           busy[2] <- mkCReg(2, False);
  Reg#(Bool)           ansV[2] <- mkCReg(2, False);
  Reg#(RegReq#(8, 32)) q[2]    <- mkCReg(2, unpack(0));
  Reg#(RegRsp#(32))    ans[2]  <- mkCReg(2, unpack(0));
  Reg#(Bit#(32))       v       <- mkReg(0);
  Reg#(Bit#(16))       wc      <- mkReg(0);

  rule tick;
    if (busy[0] && cnt[0] == 0) begin
      let r = q[0];
      Bool er = r.addr == 8'h0C;
      busy[0] <= False;
      ansV[0] <= True;
      ans[0]  <= RegRsp { rdata: r.addr == 8'h08 ? zeroExtend(wc) : v, err: er };
      if (r.write && !er) begin
        wc <= wc + 1;
        if (r.addr == 8'h04) v <= applyStrb(v, r.wdata, r.wstrb);
      end
    end else begin
      if (busy[0]) cnt[0] <= cnt[0] - 1;
      ansV[0] <= False;
    end
  endrule

  method Action req(Bool valid, RegReq#(8, 32) r);
    if (valid && !busy[1] && !ansV[1]) begin
      busy[1] <= True;
      cnt[1]  <= fromInteger(n);
      q[1]    <= r;
    end
  endmethod
  method Bool ready = !busy[1] && !ansV[1];
  method Bool rspValid = ansV[1];
  method RegRsp#(32) rsp = ans[1];
endmodule

// 一个 AXI4-Lite 主机。所有从设备输出都出自寄存器，所以这一拍读 READY/VALID、这一拍驱动引脚
// 写在同一条规则里，就是真主机在时钟沿上做的事。主机寄存器端口 0 归这条规则（握手后清掉 VALID），
// 端口 1 归命令序列（发起一笔）。
(* synthesize *)
module mkAxi4LiteTb(Empty);
  RegIf#(8, 32)             dev  <- mkLitePeriph;
  RegTarget#(8, 32)         sdev <- mkLiteSlow(3);
  Axi4LiteSlavePins#(8, 32) sa   <- mkAxi4LiteBind(dev);
  Axi4LiteSlavePins#(8, 32) sb   <- mkAxi4LiteBindT(sdev);

  Reg#(Bool)      onB <- mkReg(False);
  Reg#(UInt#(16)) cyc <- mkReg(0);

  Reg#(Bool) awv[2]    <- mkCReg(2, False);
  Reg#(Bool) wv[2]     <- mkCReg(2, False);
  Reg#(Bool) arv[2]    <- mkCReg(2, False);
  Reg#(Bool) awDone[2] <- mkCReg(2, False);
  Reg#(Bool) wDone[2]  <- mkCReg(2, False);
  Reg#(Bool) arDone[2] <- mkCReg(2, False);
  Reg#(Bool) gotB[2]   <- mkCReg(2, False);
  Reg#(Bool) gotR[2]   <- mkCReg(2, False);
  Reg#(Bool) bad[2]    <- mkCReg(2, False);

  Reg#(Bit#(8))  awa    <- mkReg(0);
  Reg#(Bit#(32)) wd     <- mkReg(0);
  Reg#(Bit#(4))  ws     <- mkReg(0);
  Reg#(Bit#(8))  ara    <- mkReg(0);
  Reg#(Bool)     brdy   <- mkReg(True);
  Reg#(Bool)     rrdy   <- mkReg(True);
  // 主机规则写、命令序列读：用 ConfigReg，两边才排得出先后（否则 G0010，命令序列饿死）
  Reg#(Bit#(2))  bRespR <- mkConfigReg(0);
  Reg#(Bit#(32)) rDataR <- mkConfigReg(0);
  Reg#(Bit#(2))  rRespR <- mkConfigReg(0);

  // 上一拍看到的：查「握手之前答复不许变」
  Reg#(Bool)     pBv  <- mkReg(False);
  Reg#(Bool)     pBhs <- mkReg(False);
  Reg#(Bit#(2))  pBr  <- mkReg(0);
  Reg#(Bool)     pRv  <- mkReg(False);
  Reg#(Bool)     pRhs <- mkReg(False);
  Reg#(Bit#(32)) pRd  <- mkReg(0);
  Reg#(Bit#(2))  pRr  <- mkReg(0);
  Reg#(Bool) saidB0 <- mkReg(False);
  Reg#(Bool) saidR0 <- mkReg(False);
  Reg#(Bool) saidB1 <- mkReg(False);
  Reg#(Bool) saidR1 <- mkReg(False);

  rule master;
    Bool     awr = onB ? sb.awready : sa.awready;
    Bool     wr  = onB ? sb.wready  : sa.wready;
    Bool     arr = onB ? sb.arready : sa.arready;
    Bool     bv  = onB ? sb.bvalid  : sa.bvalid;
    Bit#(2)  br  = onB ? sb.bresp   : sa.bresp;
    Bool     rv  = onB ? sb.rvalid  : sa.rvalid;
    Bit#(32) rd  = onB ? sb.rdata   : sa.rdata;
    Bit#(2)  rr  = onB ? sb.rresp   : sa.rresp;

    sa.aw_in(awv[0] && !onB, awa, 3'b000);
    sb.aw_in(awv[0] && onB,  awa, 3'b000);
    sa.w_in(wv[0] && !onB, wd, ws);
    sb.w_in(wv[0] && onB,  wd, ws);
    sa.b_in(brdy);
    sb.b_in(brdy);
    sa.ar_in(arv[0] && !onB, ara, 3'b000);
    sb.ar_in(arv[0] && onB,  ara, 3'b000);
    sa.r_in(rrdy);
    sb.r_in(rrdy);

    if (awv[0] && awr) begin awv[0] <= False; awDone[0] <= True; end
    if (wv[0] && wr)   begin wv[0]  <= False; wDone[0]  <= True; end
    if (arv[0] && arr) begin arv[0] <= False; arDone[0] <= True; end
    Bool bhs = bv && brdy;
    Bool rhs = rv && rrdy;
    if (bhs) begin gotB[0] <= True; bRespR <= br; end
    if (rhs) begin gotR[0] <= True; rDataR <= rd; rRespR <= rr; end

    Bool fail = False;
    if (bv && !(awDone[0] && wDone[0]) && !saidB0) begin
      $display("FAIL BVALID came before both the AW and W handshakes");
      saidB0 <= True; fail = True;
    end
    if (rv && !arDone[0] && !saidR0) begin
      $display("FAIL RVALID came before the AR handshake");
      saidR0 <= True; fail = True;
    end
    if (pBv && !pBhs && (!bv || br != pBr) && !saidB1) begin
      $display("FAIL BVALID or BRESP changed before BREADY took the response");
      saidB1 <= True; fail = True;
    end
    if (pRv && !pRhs && (!rv || rd != pRd || rr != pRr) && !saidR1) begin
      $display("FAIL RVALID, RDATA or RRESP changed before RREADY took the data");
      saidR1 <= True; fail = True;
    end
    pBv <= bv; pBhs <= bhs; pBr <= br;
    pRv <= rv; pRhs <= rhs; pRd <= rd; pRr <= rr;
    if (fail) bad[0] <= True;
  endrule

  function Action beginWrite(Bit#(8) a, Bit#(32) d, Bit#(4) s) = action
    awa <= a; wd <= d; ws <= s;
    awDone[1] <= False; wDone[1] <= False; gotB[1] <= False;
  endaction;

  function Action beginRead(Bit#(8) a) = action
    ara <= a; arDone[1] <= False; gotR[1] <= False; arv[1] <= True;
  endaction;

  function Stmt writeTogether(Bit#(8) a, Bit#(32) d, Bit#(4) s) = seq
    action beginWrite(a, d, s); awv[1] <= True; wv[1] <= True; endaction
    await(gotB[1]);
  endseq;

  function Stmt writeAwFirst(Bit#(8) a, Bit#(32) d) = seq
    action beginWrite(a, d, 4'hF); awv[1] <= True; endaction
    delay(3);
    action wv[1] <= True; endaction
    await(gotB[1]);
  endseq;

  function Stmt writeWFirst(Bit#(8) a, Bit#(32) d) = seq
    action beginWrite(a, d, 4'hF); wv[1] <= True; endaction
    delay(3);
    action awv[1] <= True; endaction
    await(gotB[1]);
  endseq;

  function Stmt readAt(Bit#(8) a) = seq
    beginRead(a);
    await(gotR[1]);
  endseq;

  function Stmt wantB(Bit#(2) resp, String what) = seq
    action
      if (bRespR != resp) begin
        $display("FAIL %s: BRESP %0d, want %0d", what, bRespR, resp);
        bad[1] <= True;
      end
    endaction
  endseq;

  function Stmt wantR(Bit#(32) val, Bit#(2) resp, String what) = seq
    action
      if (rDataR != val || rRespR != resp) begin
        $display("FAIL %s: RDATA %08h RRESP %0d, want %08h and %0d", what, rDataR, rRespR, val, resp);
        bad[1] <= True;
      end
    endaction
  endseq;

  Stmt suite = seq
    writeTogether(8'h04, 32'h11111111, 4'hF);  wantB(0, "a write with AW and W in the same cycle");
    readAt(8'h04);                              wantR(32'h11111111, 0, "the read after that write");
    writeAwFirst(8'h04, 32'h22222222);          wantB(0, "a write with AW three cycles before W");
    readAt(8'h04);                              wantR(32'h22222222, 0, "the read after the AW-first write");
    writeWFirst(8'h04, 32'h33333333);           wantB(0, "a write with W three cycles before AW");
    readAt(8'h04);                              wantR(32'h33333333, 0, "the read after the W-first write");
    writeTogether(8'h04, 32'hAABBCCDD, 4'b0101); wantB(0, "a write strobing bytes 0 and 2");
    readAt(8'h04);                              wantR(32'h33BB33DD, 0, "the read after the half-strobed write");
    readAt(8'h08);                              wantR(4, 0, "the write count after four writes");
    writeTogether(8'h0C, 0, 4'hF);              wantB(2'b10, "a write to the address that always errors");
    readAt(8'h0C);
    action
      if (rRespR != 2'b10) begin
        $display("FAIL a read of the address that always errors: RRESP %0d, want 2", rRespR);
        bad[1] <= True;
      end
    endaction

    // 响应要等 BREADY，等的时候不许变
    action brdy <= False; endaction
    action beginWrite(8'h04, 32'h55555555, 4'hF); awv[1] <= True; wv[1] <= True; endaction
    delay(8);
    action brdy <= True; endaction
    await(gotB[1]);                             wantB(0, "a write whose response waited eight cycles for BREADY");
    action rrdy <= False; endaction
    beginRead(8'h04);
    delay(8);
    action rrdy <= True; endaction
    await(gotR[1]);                             wantR(32'h55555555, 0, "a read whose data waited eight cycles for RREADY");

    // 读写同一拍发起，两笔都要完成
    action beginWrite(8'h04, 32'h66666666, 4'hF); beginRead(8'h08); awv[1] <= True; wv[1] <= True; endaction
    await(gotB[1] && gotR[1]);
    readAt(8'h04);                              wantR(32'h66666666, 0, "the read after a write that raced a read");
  endseq;

  Stmt test = seq
    suite;
    action onB <= True; endaction
    suite;
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule count;
    cyc <= cyc + 1;
    if (cyc > 6000) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  rule fin (started && fsm.done);
    if (bad[1]) $display("FAILED");
    else $display("PASS axi4lite: both binders answer a write only after AW and W, in any order, hold responses "
                  + "until BREADY and RREADY, merge strobes, map errors to SLVERR, and finish a write and a read "
                  + "started in the same cycle, for a zero-wait and a stalling target");
    $finish(bad[1] ? 1 : 0);
  endrule
endmodule

endpackage
