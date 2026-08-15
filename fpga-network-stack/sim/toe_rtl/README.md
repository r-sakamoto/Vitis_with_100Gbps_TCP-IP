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

| テスト | 構成 | 内容 |
|---|---|---|
| `passive_open` | 両方 | listen → 3WHS → データ受信+app読出し+ACK確認 → 相手FINクローズ |
| `active_open` | 両方 | appからopen → SYN送出(MSS/WSオプション検証) → 3WHS → appデータ送信 → appクローズ(FIN) |
| `rx_out_of_order` | 両方 | OOOセグメント: bypass=1はドロップ+dup ACK、bypass=0は単一ギャップバッファリング |
| `fast_retransmit` | 両方 | 3 dup ACKでfast retransmit (go-back-N) のseq正当性 |
| `rst_handling` | 両方 | 非listenポート→RST応答、確立済セッションへのRST→appへclose通知 |
| `zero_window` | 両方 | 相手win=0→app送信拒否、win更新後に再開 |
| `port_release` | 両方 | B1回帰: セッション解体でエフェメラルポートがfreePortTableに返却(BRAM白箱プローブ) |
| `opt_parser_robust` | 両方 | B5回帰: 不正/特殊オプション列(len=0、未知kind、WS=15)でも確立・転送可能 |
| `buffer_wrap` | 両方 | B4回帰: 256KBバッファwrap/64KB境界をまたぐRX書込・app読出・再送読出のデータ整合 |
| `window_update` | 両方 | F1回帰: appがバッファを読むと自発的にwindow update ACKが出る |
| `rx_ooo_overlap` | bypass=0 | B6回帰: ギャップ充填と重複する再送でもストールせず結合通知 |
| `seq_window_check` | bypass=0 | F2回帰: ウィンドウ外セグメント排除(バッファ別名書込み防止)+2^32 wrapまたぎOOO再構成 |
| `sack_rx_gen` | bypass=0 | S1回帰: SACK-Permitted交渉とdup ACK上の単一SACKブロック生成/拡大/消滅 |
| `pure_fin_close` | 両方 | F3回帰: ACKビットなしFINでもクローズ処理が走る |
| `mss_option` | 両方 | F4回帰: 相手SYNのMSS(536)が再送セグメントサイズを制限する |
| `tx_segmentation` | 両方 | P1回帰: MSS超過のapp書込み(1400B)が536B単位で分割送信され、直後のバイパス送信も正常 |
| `fast_recovery` | 両方 | P3回帰: cwnd軌跡(IW10+16060→17520→回復entry 9280→膨張10740→収縮4900)をtx_sar BRAMプローブで検証 |
| `rto_retransmit` | 両方 | RTO再送(同一seq)。**タイマ短縮ビルド専用** |
| `fin_retransmit` | 両方 | B7回帰: FINのRTO再送でファントムデータが出ない。**タイマ短縮ビルド専用** |
| `adaptive_rto` | 両方 | P2回帰: RTTサンプル後にRTOが初期1s→RTO_MIN(250ms)へ収束し指数バックオフする。**タイマ短縮ビルド専用** |

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
