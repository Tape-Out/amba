# amba

![maturity](https://img.shields.io/badge/maturity-simulated-yellow) ![license](https://img.shields.io/badge/license-Apache--2.0-blue)

AMBA protocol suite in Bluespec: pin-level interfaces and binders that attach a
bus-neutral `RegIf` (from `hwcore`) to real bus pins.

## Status

| Protocol | Spec | Pins | Binder |
| :--: | :--: | :--: | :--: |
| APB4 | ARM IHI 0024 | `Apb4SlavePins` | `mkApb4Bind` |
| AXI4-Lite | ARM IHI 0022 | planned | planned |
| AXI4-Stream | ARM IHI 0051 | planned | planned |
| AXI4 | ARM IHI 0022 | planned | planned |
| AHB-Lite | ARM IHI 0033 | planned | planned |

## Why binders instead of per-IP wrappers

An IP implements `RegIf` and nothing else. Attaching it to a bus is one line at
integration time:

```bsv
RegIf#(32, 32)          bus <- mkFabric(devs);
Apb4SlavePins#(32, 32)  apb <- mkApb4Bind(bus);
```

Changing the bus means changing that one line. Ten IPs across four buses would
otherwise need forty wrappers; here it is ten IPs plus four binders.

## Bridges

Protocol bridges (TL-UL to AXI4-Lite, AXI4-Lite to APB4, and so on) are needed
in two cases: attaching third-party IP that already exposes bus pins, and
building a bus hierarchy where a fast fabric carries a slower subordinate
segment. Our own IPs need no bridge, since the bus is chosen at bind time.

## License

Apache License 2.0.
