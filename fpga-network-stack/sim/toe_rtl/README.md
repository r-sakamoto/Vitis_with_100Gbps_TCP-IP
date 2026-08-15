# TOE RTL simulation environment (xsim)

HLS合成済みの `toe_top`(64bit / 10G / RX_DDR_BYPASS=1 / TCP_NODELAY=1 /
WINDOW_SCALE=1 / MSS=1460 / MAX_SESSIONS=1000)を Vivado xsim でRTLシミュレーション
する環境。実機(`kernel/network_krnl/src/hdl/tcp_stack.sv`)と同じ外部環境を
SystemVerilogでモデル化している:

| TBコンポーネント | 実機での対応物 |
|---|---|
| session lookup CAM モデル (96/120/144/152bit AXIS) | `hash_table_ip` (cuckoo hash) |
| TXバッファメモリモデル (mmCmd/data/status) | `axi_datamover` + DDR |
| RXバイパスFIFO (word count フィードバック) | `axis_data_fifo_64_d1024` |
| アプリBFM (listen/open/close/read/send) | user kernel |
| TCPピアモデル (IPv4+TCPパケット生成/解析/チェックサム検証) | 対向ホスト(Linux等) |

## 使い方

```bash
# 前提: RTLが未生成なら先に  ../../build_toe_10g.sh  を実行
cd sim/toe_rtl
./run_sim.sh                  # 全テスト実行
./run_sim.sh passive_open     # 単一テスト
DUMP=1 ./run_sim.sh active_open   # VCD波形付き (run/tb_toe_top.vcd)
```

### RX_DDR_BYPASS=0 構成のテスト

RXペイロードをDDR(メモリモデル)経由にした構成。単一ギャップのOOOバッファ
リングが有効になる。TBは `-d NO_RX_BYPASS` で自動的にポート構成・RXメモリ
モデル・OOOテスト期待値を切り替える:

```bash
../../build_toe_10g_nobypass.sh              # build10g_nb/ にRTL合成
NO_RX_BYPASS=1 ./run_sim.sh                  # 全テスト (作業dir: run_nb/)
# RTO再送テスト(タイマ短縮×bypass=0):
../../build_toe_10g_nobypass.sh buildsim_nb 1000
NO_RX_BYPASS=1 WORK_TAG=_nb_sim \
  RTL_DIR=$PWD/../../buildsim_nb/hls/toe/toe_prj/solution1/syn/verilog \
  ./run_sim.sh rto_retransmit
```

bypass=0でのOOO動作(TBで検証済み): OOOセグメントはDDRにバッファされ
dup ACKのみ送出(アプリ通知なし。内部の長さ0通知はrxAppNotificationDelayerが
ステータス整合のため消費し、アプリには転送されない)。ギャップが充填されると
結合された全長の通知が一度に届く。保持できるギャップは1つだけで、2つ目の
穴になるセグメントはドロップされる。

ログは `run/xsim_<test>.log`。パケットの送受信は
`NET->DUT:` / `DUT->NET:` 行としてtcpdump風に出力される。

## テスト一覧

| テスト | 内容 |
|---|---|
| `passive_open` | listen → 3WHS(SYN/SYN-ACK/ACK) → データ受信+app読出し+ACK確認 → 相手FINクローズ |
| `active_open` | appからopen → SYN送出(MSS/WSオプション検証) → 3WHS → appデータ送信 → appクローズ(FIN) |
| `rx_out_of_order` | 順序入替セグメント: bypass構成ではドロップ+dup ACKになることを確認、再送で回復 |
| `fast_retransmit` | 3 dup ACKでfast retransmit (go-back-N) が起きseqが正しいことを確認 |
| `rst_handling` | 非listenポートへのセグメント→RST応答、確立済セッションへのRST→appへclose通知 |
| `zero_window` | 相手win=0広告→app送信拒否(NOSPACE)、win更新後に送信再開 |

| `rto_retransmit` | ACKを返さずRTO再送(同一seqの再送)を確認。**タイマ短縮ビルド専用** |

## タイマについての注意(タイマ短縮ビルド)

RTL(合成時)のタイマ定数は実時間スケール(RTO初回1秒 ≒ 1.56億サイクル)のため、
RTO再送・persistプローブ・TIME_WAIT解放は通常ビルドでは実用時間で観測できない。
このため `TCP_STACK_SIM_TIMER_SCALE`(cmakeオプション、`toe_internals.hpp`の
`TIME_*`を割る係数、デフォルト1)を追加した。タイマ短縮RTLの生成と実行:

```bash
../../build_toe_10g_sim.sh          # buildsim/ に timer_scale=1000 のRTLを合成
WORK_TAG=_sim RTL_DIR=$PWD/../../buildsim/hls/toe/toe_prj/solution1/syn/verilog \
    ./run_sim.sh rto_retransmit
```

ハードウェア用ビルド(build10g等)ではこのオプションを必ず1のままにすること。

## 既知の制約(TBが前提とするDUT仕様)

- RX_DDR_BYPASS=1 のため、順序入替(OOO)セグメントはDUT側でドロップされる
- TCP_NODELAY=1 のため、appの1書き込み=1セグメント(MSS分割なし)
- app送信はtxMeta→txRsp(空き確認)→データの順で行う必要がある
