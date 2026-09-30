package AhbTb;

import RegFile::*;
import ConfigReg::*;
import RegIf::*;
import Ahb::*;

// mkAhbMgr 的行为测试：测试台一头当 AHB 发起方（照规范的地址相、数据相与流水），
// 一头当片上总线（延迟 0 到 3 拍不等，0x1000 起回错）。
//
// 序列：字、字节、半字的写与读回 · 三写三读首尾相接不留空拍（流水）· 回错的两拍
// 与紧跟其后的一笔照常 · 独占读写的 hexokay · 中间夹几个空闲拍。

typedef struct {
  Bool w; Bit#(32) a; Bit#(3) sz; Bit#(32) d; Bool ex; Bool err; Bit#(32) want; Bool chk; Bool gap;
} Op deriving (Bits);

function Op op(Bit#(5) i);
  function Op wr(Bit#(32) a, Bit#(3) sz, Bit#(32) d) =
    Op { w: True, a: a, sz: sz, d: d, ex: False, err: False, want: 0, chk: False, gap: False };
  function Op rd(Bit#(32) a, Bit#(32) want) =
    Op { w: False, a: a, sz: 2, d: 0, ex: False, err: False, want: want, chk: True, gap: False };
  case (i)
    0: return wr('h100, 2, 'h11223344);
    1: return rd('h100, 'h11223344);
    2: return wr('h101, 0, 'h0000AA00);
    3: return rd('h100, 'h1122AA44);
    4: return wr('h102, 1, 'hBEEF0000);
    5: return rd('h100, 'hBEEFAA44);
    6: return wr('h104, 2, 'hA0A0A0A0);
    7: return wr('h108, 2, 'hB1B1B1B1);
    8: return wr('h10C, 2, 'hC2C2C2C2);
    9: return rd('h104, 'hA0A0A0A0);
    10: return rd('h108, 'hB1B1B1B1);
    11: return rd('h10C, 'hC2C2C2C2);
    12: return Op { w: False, a: 'h1000, sz: 2, d: 0, ex: False, err: True, want: 0, chk: False, gap: False };
    13: return rd('h108, 'hB1B1B1B1);
    14: begin let o = rd('h100, 'hBEEFAA44); o.gap = True; return o; end
    15: return Op { w: False, a: 'h110, sz: 2, d: 0, ex: True, err: False, want: 0, chk: False, gap: False };
    16: return Op { w: True, a: 'h110, sz: 2, d: 'h5EC0DE5, ex: True, err: False, want: 0, chk: False, gap: True };
    17: return rd('h110, 'h5EC0DE5);
    18: return wr('h103, 0, 'h7F000000);
    19: return rd('h100, 'h7FEFAA44);
    default: return unpack(0);
  endcase
endfunction

Integer n = 20;

(* synthesize *)
module mkAhbTb(Empty);
  RegFile#(Bit#(6), Bit#(32)) mem <- mkRegFileFull;

  // ---- 发起方
  Reg#(Bit#(5))        ai   <- mkReg(0);          // 地址相上的是第几笔
  Reg#(Maybe#(Bit#(5))) di  <- mkReg(tagged Invalid);   // 数据相上的是第几笔
  Reg#(Bool)           idle <- mkReg(True);       // 这一拍地址相放空
  // 计拍规则与发起方规则互相读对方写的计数，普通寄存器会绕成环，把发起方整条挡掉
  Reg#(Bit#(8))        done <- mkConfigReg(0);
  Reg#(Bool)           bad  <- mkReg(False);
  Reg#(Bit#(32))       cyc  <- mkConfigReg(0);
  Reg#(Bool)           err1 <- mkReg(False);      // 上一拍是回错的第一拍

  Wire#(Bit#(1))  hr <- mkBypassWire;
  Wire#(Bit#(1))  hrs <- mkBypassWire;
  Wire#(Bit#(1))  hx <- mkBypassWire;
  Wire#(Bit#(32)) hd <- mkBypassWire;

  Op ao = op(ai);
  Bool addrOn = !idle && ai < fromInteger(n);
  Op dop = op(fromMaybe(0, di));

  AhbMgrPins pins = interface AhbMgrPins;
    method Bit#(32) haddr = ao.a;
    method Bit#(1)  hwrite = pack(ao.w);
    method Bit#(2)  htrans = addrOn ? 2'b10 : 2'b00;
    method Bit#(3)  hsize = ao.sz;
    method Bit#(3)  hburst = 0;
    method Bit#(4)  hprot = 4'b0011;
    method Bit#(1)  hmastlock = 0;
    method Bit#(8)  hmaster = 0;
    method Bit#(1)  hexcl = pack(ao.ex);
    method Bit#(32) hwdata = (isValid(di) && dop.w) ? dop.d : 32'hDEADBEEF;
    method Action hready(Bit#(1) v); hr <= v; endmethod
    method Action hresp(Bit#(1) v); hrs <= v; endmethod
    method Action hexokay(Bit#(1) v); hx <= v; endmethod
    method Action hrdata(Bit#(32) v); hd <= v; endmethod
  endinterface;

  RegManager#(32, 32) m <- mkAhbMgr(pins);

  // ---- 片上总线：延迟 0 到 3 拍，按地址的第 2、3 位
  Reg#(Bool)            busy <- mkReg(False);
  Reg#(Bit#(2))         lat  <- mkReg(0);
  Reg#(RegReq#(32, 32)) q    <- mkReg(unpack(0));
  Reg#(Bit#(8))         reqs <- mkReg(0);

  rule fabric;
    Bool v = False;
    RegRsp#(32) x = RegRsp { rdata: 0, err: False };
    if (busy && lat == 0) begin
      v = True;
      busy <= False;
      Bit#(6) wi = q.addr[7:2];
      if (q.addr >= 'h1000) x.err = True;
      else if (q.write) mem.upd(wi, applyStrb(mem.sub(wi), q.wdata, q.wstrb));
      else x.rdata = mem.sub(wi);
    end else if (busy) lat <= lat - 1;
    else if (m.valid) begin
      q <= m.req;
      busy <= True;
      lat <= m.req.addr[3:2];
      reqs <= reqs + 1;
    end
    m.ready(!busy);
    m.resp(v, x);
  endrule

  rule tick;
    cyc <= cyc + 1;
    if (cyc > 5000) begin
      $display("FAIL TIMEOUT: %0d of %0d transfers done", done, n);
      $finish(1);
    end
  endrule

  // 发起方看的都是这一拍末尾的采样：hready 为高时，数据相那一笔完成、地址相那一笔被收下
  rule manager (cyc > 2);
    Bool nb = bad;
    if (hr == 0 && hrs == 1) begin
      if (!isValid(di) || !dop.err) begin
        $display("FAIL transfer %0d answered an error it should not have", fromMaybe(0, di));
        nb = True;
      end
      if (err1) begin
        $display("FAIL an error response longer than two cycles");
        nb = True;
      end
    end
    err1 <= hr == 0 && hrs == 1;
    if (hr == 1 && hrs == 1 && !err1) begin
      $display("FAIL an error response without its first cycle");
      nb = True;
    end
    if (hr == 1) begin
      if (di matches tagged Valid .k) begin
        if (dop.err && hrs == 0) begin
          $display("FAIL transfer %0d should have answered an error", k);
          nb = True;
        end
        if (dop.chk && hd != dop.want) begin
          $display("FAIL transfer %0d read %h, want %h", k, hd, dop.want);
          nb = True;
        end
        if (dop.ex && hx != 1) begin
          $display("FAIL exclusive transfer %0d got no hexokay", k);
          nb = True;
        end
        if (!dop.ex && hx != 0) begin
          $display("FAIL hexokay on a plain transfer %0d", k);
          nb = True;
        end
        done <= done + 1;
      end
      if (addrOn) begin
        di <= tagged Valid ai;
        ai <= ai + 1;
        idle <= op(ai + 1).gap;
      end else begin
        di <= tagged Invalid;
        idle <= False;
      end
    end
    bad <= nb;
  endrule

  rule finish (done == fromInteger(n));
    if (reqs != fromInteger(n)) begin
      $display("FAIL %0d bus requests for %0d transfers", reqs, n);
      $finish(1);
    end
    if (bad) $finish(1);
    $display("PASS ahb: %0d transfers, byte and half-word strobes, pipelined back to back, two-cycle error, exclusives", n);
    $finish(0);
  endrule
endmodule

endpackage
