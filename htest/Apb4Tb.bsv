package Apb4Tb;

import RegIf::*;
import Apb4::*;

// 假外设，零等待。三处地址各挂一条判据：
//   0x04  普通读写
//   0x08  读回「写过多少次」——SETUP 拍要是有副作用，这个数就不是 1
//   0x0C  一律回错误——PSLVERR 的判据挂在它上面
module mkPeriph(RegIf#(8, 32));
  Reg#(Bit#(32)) v0 <- mkReg(0);
  Reg#(Bit#(32)) v1 <- mkReg(0);
  Reg#(Bit#(16)) wc <- mkReg(0);

  method ActionValue#(RegRsp#(32)) access(RegReq#(8, 32) r);
    Bool     er = r.addr == 8'h0C;
    Bit#(32) rd = 0;
    if (r.addr == 8'h08)      rd = zeroExtend(wc);
    else if (r.addr == 8'h04) rd = v1;
    else                      rd = v0;
    if (r.write && !er) begin
      wc <= wc + 1;
      if (r.addr == 8'h04) v1 <= r.wdata;
      else if (r.addr == 8'h00) v0 <= r.wdata;
    end
    return RegRsp { rdata: rd, err: er };
  endmethod
endmodule

// 会停顿的假外设：收下之后压 n 拍才答。SRAM 宏与缓存就是这个形状。
// 规则与 always_enabled 的方法同写这几个量，所以一律 CReg：端口 0 归规则、
// 端口 1 归方法。
module mkSlowPeriph#(Integer n)(RegTarget#(8, 32));
  Reg#(Bit#(8))  cnt[2]  <- mkCReg(2, 0);
  Reg#(Bool)     busy[2] <- mkCReg(2, False);
  Reg#(Bool)     ansV[2] <- mkCReg(2, False);
  Reg#(Bit#(32)) v[2]    <- mkCReg(2, 0);
  Reg#(Bool)     wr[2]   <- mkCReg(2, False);
  Reg#(Bit#(32)) wd[2]   <- mkCReg(2, 0);

  rule tick;
    if (busy[0] && cnt[0] == 0) begin
      busy[0] <= False;
      ansV[0] <= True;
      if (wr[0]) v[0] <= wd[0];
    end else begin
      if (busy[0]) cnt[0] <= cnt[0] - 1;
      ansV[0] <= False;
    end
  endrule

  method Action req(Bool valid, RegReq#(8, 32) r);
    if (valid && !busy[1] && !ansV[1]) begin
      busy[1] <= True;
      cnt[1]  <= fromInteger(n);
      wr[1]   <= r.write;
      wd[1]   <= r.wdata;
    end
  endmethod
  method Bool ready = !busy[1] && !ansV[1];
  method Bool rspValid = ansV[1];
  method RegRsp#(32) rsp = RegRsp { rdata: v[1], err: False };
endmodule

// 一个 APB4 主机，照 IHI 0024D 第 4 章的三态走：IDLE -> SETUP -> ACCESS。
//
// 结果只在「PSEL、PENABLE、PREADY 同时为高」的那一拍有效（附录 A），所以采样
// 必须落在那一拍。但驱动引脚与读回引脚不能写在同一条规则里：零等待从设备的
// PRDATA 是一条从 PADDR 过来的组合通路，在 BSV 里表现为同一条线的 wset 与
// wget，同规则即 G0004。真实电路里两头都是端口、不经规则，所以这条约束只落在
// 测试台上——办法是 snap 按拍采样、judge 晚一拍比对，跟真主机在时钟沿采
// PRDATA 是同一件事。
(* synthesize *)
module mkApb4Tb(Empty);
  RegIf#(8, 32)     dev  <- mkPeriph;
  RegTarget#(8, 32) sdev <- mkSlowPeriph(3);
  Apb4SlavePins#(8, 32) sa <- mkApb4Bind(dev);
  Apb4SlavePins#(8, 32) sb <- mkApb4BindT(sdev);

  Reg#(Bit#(2))  st    <- mkReg(0);   // 0=IDLE 1=SETUP 2=ACCESS
  Reg#(Bit#(8))  ph    <- mkReg(0);
  Reg#(Bit#(8))  addrR <- mkReg(0);
  Reg#(Bool)     wrR   <- mkReg(False);
  Reg#(Bit#(32)) wdR   <- mkReg(0);
  Reg#(Bit#(16)) wcyc  <- mkReg(0);   // 这一笔等了几拍
  Reg#(Bool)     bad   <- mkReg(False);
  Reg#(Bit#(16)) cyc   <- mkReg(0);

  // 采样与待判的那一笔。两个从设备都照采：采样规则一旦读了 ph，它就既要排在
  // run 之后（线要先写后读）又要排在 run 之前（ph 要先读后写），bsc 于是判 run
  // 永不触发——表现是相位卡死而不是报错。
  Reg#(Bit#(32)) rdA  <- mkReg(0);
  Reg#(Bool)     erA  <- mkReg(False);
  Reg#(Bit#(32)) rdB  <- mkReg(0);
  Reg#(Bool)     erB  <- mkReg(False);
  Reg#(Bool)     jval <- mkReg(False);
  Reg#(Bit#(8))  jph  <- mkReg(0);
  Reg#(Bit#(16)) jwc  <- mkReg(0);

  Bool onB = ph >= 5;   // 第 5 步起换会停顿的那个绑定器

  rule timeout;
    cyc <= cyc + 1;
    if (cyc > 2000) begin $display("TIMEOUT at phase %0d", ph); $finish(1); end
  endrule

  rule snap;
    rdA <= sa.prdata;
    erA <= sa.pslverr;
    rdB <= sb.prdata;
    erB <= sb.pslverr;
  endrule

  rule judge (jval);
    Bit#(32) rdS = jph >= 5 ? rdB : rdA;
    Bool     erS = jph >= 5 ? erB : erA;
    Bool w = False;
    case (jph)
      1: if (rdS != 32'h1234ABCD) begin
           $display("FAIL read back %08h in the cycle PREADY was high, want 1234ABCD",
                    rdS);
           w = True;
         end
      2: if (rdS != 32'h00000001) begin
           $display("FAIL the peripheral saw %0d writes for one APB write", rdS);
           w = True;
         end
      3: if (!erS) begin
           $display("FAIL PSLVERR was low on the address that always errors");
           w = True;
         end
      4: if (erS) begin
           $display("FAIL PSLVERR stayed high into the next transfer");
           w = True;
         end
      5: if (jwc < 2) begin
           $display("FAIL the slow write finished after %0d wait cycles", jwc);
           w = True;
         end
      6: begin
           if (rdS != 32'h55AA55AA) begin
             $display("FAIL the slow read gave %08h, want 55AA55AA", rdS);
             w = True;
           end
           if (jwc < 2) begin
             $display("FAIL the slow read finished after %0d wait cycles", jwc);
             w = True;
           end
         end
    endcase
    if (w) bad <= True;
  endrule

  rule run;
    Bool     psel = st != 0;
    Bool     pen  = st == 2;
    // 读传输的 PSTRB 必须全低（3.2）——主机这一侧的规矩，顺手也守上
    Bit#(4)  strb = wrR ? 4'hF : 4'h0;

    sa.req(addrR, 3'b000, psel && !onB, pen && !onB, wrR, wdR, strb);
    sb.req(addrR, 3'b000, psel && onB,  pen && onB,  wrR, wdR, strb);

    Bool fin = pen && (onB ? sb.pready : sa.pready);

    Bit#(2)  nst = st;
    Bit#(8)  nph = ph;
    Bit#(8)  na  = addrR;
    Bool     nwr = wrR;
    Bit#(32) nwd = wdR;
    Bit#(16) nwc = wcyc;

    if (st == 1) begin
      nst = 2;
      nwc = 0;
    end else if (st == 2) begin
      if (fin) begin
        nst = 0;
        nph = ph + 1;
      end else
        nwc = wcyc + 1;
    end else if (ph < 7) begin
      nst = 1;
      case (ph)
        0: begin na = 8'h04; nwr = True;  nwd = 32'h1234ABCD; end
        1: begin na = 8'h04; nwr = False; end
        2: begin na = 8'h08; nwr = False; end
        3: begin na = 8'h0C; nwr = False; end
        4: begin na = 8'h00; nwr = False; end
        5: begin na = 8'h04; nwr = True;  nwd = 32'h55AA55AA; end
        6: begin na = 8'h04; nwr = False; end
      endcase
    end

    st    <= nst;
    ph    <= nph;
    addrR <= na;
    wrR   <= nwr;
    wdR   <= nwd;
    wcyc  <= nwc;
    jval  <= fin;
    jph   <= ph;
    jwc   <= wcyc;
  endrule

  // 收工单列一条：run 要是自己读 bad，它就跟 judge 互为前后，同样成环
  rule done (st == 0 && ph >= 7 && !jval);
    if (bad) $display("FAILED");
    else $display("PASS apb4: PRDATA and PSLVERR are valid in the cycle PREADY "
                  + "is high, SETUP has no side effect, and a slow target "
                  + "stalls with PREADY low");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
