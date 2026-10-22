# TOE (TCP Offload Engine) vs Linux TCP 機能ギャップ分析

対象: `fpga-network-stack/hls/toe/` (ETH Zurich TOE, 現行ビルド構成:
`MSS=1460, MAX_SESSIONS=1000, DATA_WIDTH=64bit, TCP_NODELAY=1,
RX_DDR_BYPASS=1, FAST_RETRANSMIT=1, WINDOW_SCALE=1(shift=2固定)`)

ソースコードの精査(rx_engine / tx_engine / tx_sar_table / rx_sar_table /
retransmit_timer / probe_timer / close_timer / session_lookup_controller /
port_table / state_table / ack_delay / tx_app_* / rx_app_*)、および
[fpgasystems/fpga-network-stack の GitHub issue](https://github.com/fpgasystems/fpga-network-stack/issues)
(#19, #21, #26, #31, #39, #42, #43, #44) に基づく。

凡例: 優先度 ★★★=相互運用・スループットに直接影響 / ★★=堅牢性・性能改善 / ★=セキュリティ・運用性

---

## 1. TCPオプション(RFC 793 / 7323 / 2018)

| 機能 | 現状 | Linux | 優先度 |
|---|---|---|---|
| MSSオプション(受信) | **実装済** (F4+P1, `mss_option`/`tx_segmentation`テスト): 実効MSS=min(peer,local)をセッション毎に保存し、初送・再送ともMSS超過書込みをDDR読出しで分割送信(バイパス複製ストリームは破棄モードで整合)。PMTUDは未 | 相手MSSとPMTUからMSSを決定 | ★(残: PMTUD) |
| Window Scale(受信) | SYN時のみパース。**自側shiftは0か2の固定** (`rx_engine.cpp:1366,1429`)。相手shiftは**14超の値を無検証で格納** → `tx_sar_table.cpp:121` でビット範囲オーバーフロー(issue #42) | shift 0〜14を交渉、バッファサイズから導出 | ★★★ |
| SACK / SACK-Permitted | **実装済** (S1+S2+M1+C2, `sack_rx_gen`/`sack_selective_rtx`/`multi_gap_sack`/`sack_multi_rtx`テスト): 交渉+最大2ブロックのSACK生成(TS共存で32Bオプション)+相手SACK最大2ブロックの解釈と選択的再送(スキップ2ラウンド+2重クランプで順不同ブロック対応)。D-SACKは未 | あり(選択再送・D-SACK) | ★(残: D-SACK) |
| Timestamps (RFC 7323) | **実装済** (T1, `ts_negotiate`/`paws_drop`テスト): SYN/SYN-ACKでTS交渉(常時提示/条件エコー)、全セグメント(データ含む)にNOP NOP TS 12B付与、64bitデータパスでボディを4Bシフト。TS.Recent追跡+PAWSで旧タイムスタンプのセグメントを破棄しdup ACK応答。実効MSSは-12B。TSval=~0.84ms粒度のフリーランカウンタ。RTT計測は既存rtt_estimator(Karn)のまま(TSecrベースRTTは未)。512bitデータパスはTS無効(TCP_TS_OPTIONゲート) | あり | 完了 |
| ECN (ECE/CWR/ECT) | **なし**。フラグは常時0固定 (`toe_internals.hpp:522`) | あり | ★ |
| オプションパーサの正当性 | dataOffsetの単位バグ(ワード vs バイト混在, issue #43)、不正オプション列でRXパイプラインがストールし得る (`rx_engine.cpp:645-700,730-738`) | — | ★★★(バグ修正) |
| PSH / URG | PSHは常に0、URG・緊急ポインタ未対応 | あり | ★ |

## 2. 再送・RTT・輻輳制御(RFC 6298 / 5681)

| 機能 | 現状 | Linux | 優先度 |
|---|---|---|---|
| RTT計測 / 適応RTO | **実装済** (P2, `adaptive_rto`テスト): 新モジュール`rtt_estimator`がKarn方式でRTT計測、RFC 6298のSRTT/RTTVAR→RTO=clamp(SRTT+4V, 250ms, 30s)。retransmit_timerはRTO×2^retriesの指数バックオフ化。Timestampsオプション(PAWS含む)は未 | SRTT/RTTVARによるRFC 6298、最小RTO 200ms | ★(残: TSopt) |
| 再送回数上限 | **改善済** (C3, `rt_abort`テスト): データ再送は7回(RTO上限30sで累計1分超)後にセッション解放+app通知、SYN/SYN-ACKは4回のまま。LinuxもRTO枯渇時にRSTは送らない(ETIMEDOUT) | tcp_retries2(15回) | ★(残: 回数) |
| 再送方式 | **改善済** (S2+C2): peer SACKブロック(最大2個)の範囲をスキップ/クランプする選択的再送。RTOあたり最大4セグメントの上限は残る(SACKで実質補完) | SACKベースの選択再送 | 完了 |
| Fast Retransmit | **実装済** (P3, `fast_recovery`テスト): 3rd dup ACKでssthresh=max(flight/2,2MSS)・cwnd=ssthresh+3MSS、dup ACK毎に+MSS膨張、回復脱出でcwnd=ssthresh収縮。NewReno partial-ACK再送はgo-back-N再送のため実質不要 | RFC 5681 fast recovery、partial ACK処理 | 完了 |
| 輻輳制御 | **正規化済** (P3+C1+G2+G3, `ca_growth`/`cwnd_idle_restart`/`rto_single_seg`テスト): cwnd初期=IW10、RTO後=1×MSSかつ再送は先頭1セグメントのみ(ACKクロック駆動)、slow start+sessMSS/ACK、輻輳回避はRFC 3465バイトカウント、~1秒無送信でcwnd→IWへ再検証(RFC 5681 4.1)。CUBIC/pacing/RACK-TLPは未(Reno相当) | CUBIC(デフォルト)ほか、ABC、pacing、RACK-TLP | ★(残: CUBIC/RACK) |
| dup ACK判定 | **準拠** (P3+G1, `dupack_win_change`テスト): staleACKガード+広告ウィンドウ不変チェック(純window updateはdupカウントされずfast retransmitを誤発火しない) | RFC 5681準拠 | 完了 |
| シーケンス番号の巡回演算 | **統一済** (F2+P3+C4): OOO受理・dup ACK判定・ACK受理窓(snd_una≤ack≤snd_nxt)・PAWS・SACK処理はすべてseq_lt/leq/gt/geqヘルパ(mod 2^32)。等値比較はwrap非依存 | 全比較がmod 2^32 | 完了 |

## 3. 受信処理・フロー制御

| 機能 | 現状 | Linux | 優先度 |
|---|---|---|---|
| 順序入替(OOO)受信 | **2ギャップ対応** (M1, `multi_gap_sack`テスト): 非bypass構成で同時2レンジをバッファし、充填時のカスケードマージ・上方拡張・レンジ昇格をサポート。RX_DDR_BYPASS=1はバッファなしのため引き続き全ドロップ(アーキテクチャ制約)。3ギャップ以上は最初の2レンジのみ保持 | 完全な再構成キュー | ★(残: bypass構成) |
| RFC 793受理判定 | **一部実装済** (F2, `seq_window_check`テスト): 非bypassのOOO受理を広告ウィンドウ内に制限(バッファ別名書込み防止)、OOO系比較をmod 2^32化(seq_lt等ヘルパ追加)。ACK側の全面的なwrap対応は未 | あり | ★(残) |
| ウィンドウ更新通知 | **実装済** (F1, `window_update`テスト): app読出しでappdが進むとACKイベントを発行しwindow update ACKを送出 | 即時window update | 完了 |
| ゼロウィンドウプローブ | **実装済** (Z1, `zero_window_probe`テスト): rx側がゼロウィンドウ広告でpersistタイマ(50ms)をセット、満了でseq=snd_nxt-1の純ACKプローブ送出(相手が現ウィンドウを返答)、ウィンドウ回復のACKで解除+送信再開イベント。probe_timerは統一read-modify-writeパスに書換え(書込み順序ハザード根治) | RFC 1122準拠persist | 完了 |
| Delayed ACK | 64µs固定(Linuxの40msの約1/600)。イベント負荷で無限延期あり得る (`ack_delay.cpp`) | 40ms/クオックACK | ★ |
| SWS回避 | **受信側実装済** (W1, `sws_avoidance`テスト): rx_sar_tableに影のadvertised edgeを導入、広告ウィンドウ右端は1 MSS以上の単位でのみ前進(RFC 1122 4.2.3.3)。送信側SWS(Nagle)はTCP_NODELAY=1のため対象外 | あり | 完了(受信側) |
| Keepalive | **実装済** (N2, `keepalive`テスト): 新モジュール`keepalive_timer`(RFC 1122 4.2.3.6)。受信セグメントごとにアイドルタイマをリセット、60秒無通信でseq=snd_nxt-1の純ACKプローブを10秒間隔で最大4回送出、無応答なら切断(app通知closed+セッション解放)。Linuxのデフォルト(7200s/75s/9probes)より積極的な定数だがハードコード | SO_KEEPALIVE | 完了 |

## 4. コネクション管理・ステートマシン

| 機能 | 現状 | Linux | 優先度 |
|---|---|---|---|
| ハーフクローズ | **実装済** (N1, `half_close`テスト): 相手FINでCLOSE_WAITへ入りACKのみ返答、appはclose通知後も送信可能(tx_app_stream_ifがCLOSE_WAIT送信を許可)。app closeでFIN→LAST_ACK→解放 | shutdown(SHUT_WR)対応 | 完了 |
| FIN_WAIT_2 | **実装済** (N1+R1, `time_wait_ack`テスト): FIN_WAIT_1→FIN_WAIT_2遷移、FIN_WAIT_2でのデータ受信・相手FIN→TIME_WAIT。TIME_WAIT中の迷子ACK/再送FINにはRSTでなくACKを返す(RFC 793) | 準拠 | 完了 |
| ACKなしFIN | **実装済** (F3, `pure_fin_close`テスト): 純FINもFIN+ACKと同様に処理(ACK番号依存処理はACKフラグでガード) | 処理する | 完了 |
| 同時オープン | 非対応(SYN_SENT中のSYNをパッシブオープン扱い) | 対応 | ★ |
| TIME_WAIT | 60秒固定(2MSLでない)。**再利用は実装済** (R4, `time_wait_reuse`テスト): listenポート宛てでバックログに空きがあれば、TIME_WAIT中の新SYNをTS有効ならTSval>TS.Recent(RFC 6191)、無効ならSEQ>RCV.NXT(RFC 1122 4.2.2.13)で受理し、新規接続として初期化+SYN-ACK、TIME_WAITタイマは取消。条件外のSYNはACK応答 | 60s + 再利用(RFC 1122) | 完了 |
| listen解除 | **実装済** (L1, `listen_close`テスト): listen要求のbit15をclose-listenフラグとして解釈(listenポートは0..32767なので既存AXIS I/F互換)。閉鎖後のSYNにはRST、再listenで復旧 | close/bind準拠 | 完了 |
| ISN生成 | **改善済** (R1, `isn_quality`テスト): ISN = LFSR ^ (sessionID<<16) + フリーランカウンタ。旧実装の「下位6bit常に0x3F」パターンを解消し時間成分を追加。RFC 6528完全準拠(秘密鍵+4タプルハッシュ)は未 | RFC 6528(秘密鍵+4タプルハッシュ+時刻) | ★(残) |
| SYNバックログ制限 | **実装済** (R3, `syn_backlog`テスト): SYN_RECEIVED状態のセッション数をstate_tableで計数し、`SYN_BACKLOG_MAX`(既定MAX_SESSIONS/2)に達したら新規SYNを無応答で破棄(セッション即解放)。確立済み接続用に残り半分のセッションを確保。SYN cookiesは未 | tcp_max_syn_backlog + syncookies | ★(残: cookies) |
| RST検証 | **実装済** (R2, `challenge_ack`/`challenge_ack_limit`テスト): RFC 5961。seq==RCV.NXTのRSTのみ切断、窓内RST・同期状態のSYN/SYN-ACKにはchallenge ACK、ACK検証(SEG.ACK>SND.NXT または SND.UNA-バッファサイズ未満はデータごと破棄+ACK)、challenge ACKは全体で1000回/秒に制限 | RFC 5961 | 完了 |
| マルチホーム | 3タプル(theirIp,myPort,theirPort)キーで**ローカルIPは1つのみ** | 4タプル | ★ |

## 5. 実装バグ(機能追加とは別に修正が必要)

| # | 状態 | 内容 | 場所 |
|---|---|---|---|
| B1 | **修正済** (`port_release`テストで検証) | エフェメラルポートが**解放されない**(エンディアン不一致: `releaseTuple.myPort`がスワップ済のままport_tableの`>=32768`判定に使われる) → active openがいずれ枯渇。release時に`reverse()`を追加 | `session_lookup_controller.cpp` reverseLookupTableInterface |
| B2 | **修正済** (NODELAY=0パス、比較の向きを訂正) | tx_sar_tableの`usableWindow`計算の比較が逆(`minWindow < usedLength`)。NODELAY=1では潜在 | `tx_sar_table.cpp:132-136` |
| B3 | **修正済** (NODELAY=0構成のcsynth完走を確認) | TCP_NODELAY=0(Nagle)ビルドは**コンパイル不能**(`txSar`未宣言、セミコロン欠落) | `tx_engine.cpp:208-331,1427` |
| B4 | **修正済** (`buffer_wrap`テストで検証) | (a)再送時のバッファwrap: 2つ目の読み出しコマンドの先頭アドレスが0固定(issue #31)、(b)`rxAppMemAccessBreakdown`が64KB境界ハードコード分割で誤アドレス継続 | `tx_engine.cpp` txEngMemAccessBreakdown / `toe.cpp` rxAppMemAccessBreakdown |
| B5 | **修正済** (`opt_parser_robust`テストで検証) | オプションパーサのdataOffset単位バグ(issue #43)、不正オプション(len=0)での無限ループ=RXパス恒久停止、win_shift>14未クランプ(issue #42)。バイト単位カウント+堅牢化+14クランプに書換え | `rx_engine.cpp` parse_optional_header_fields |
| B6 | **修正済** (`rx_ooo_overlap`テストで検証) | OOOギャップ充填時の重複セグメントで分岐が無くパイプラインストール(非bypass構成) | `rx_engine.cpp` ギャップ充填分岐 |
| B7 | **修正済** (`fin_retransmit`テストで検証。RT再送長からFINファントムバイトを除外) | FIN再送タイマのイベント型がRT(FIXMEコメントあり) | `tx_engine.cpp:643-644` |
| B8 | **修正済** (`#ifndef __SYNTHESIS__`ガードを28ブロックに適用) | 合成対象コードに`std::cout`が無ガードで残存 | `tx_engine.cpp`他多数 |

## 6. 実装優先順位の提案(Linux同等化ロードマップ)

1. **フェーズ1(正しさ・相互運用)**: B1〜B6のバグ修正、MSSオプション受信対応、
   ウィンドウ更新ACK送出、汎用シーケンス窓チェック+wrap対応比較、ACKなしFIN処理
2. **フェーズ2(性能)**: RTT計測(Timestampsオプション)+RFC 6298適応RTO、
   RFC 5681 fast recovery(ssthresh/cwnd正規化、MSS依存定数除去)、
   受信OOOバッファリング(bypass構成でも単一ギャップ保持)、SWS回避
3. **フェーズ3(SACK)**: SACK-Permitted/SACKブロック生成・解釈、SACKベース選択再送
4. **フェーズ4(堅牢性)**: keepalive、persist正規化、half-close(CLOSE_WAIT)、
   FIN_WAIT_2、RFC 6528 ISN、RFC 5961、TIME_WAIT再利用、listen解除

## 7. 検証環境

- RTLシミュレーション: `sim/toe_rtl/`(xsim + SystemVerilog TB、`sim/toe_rtl/README.md`参照)
- タイマ系(RTO/persist/TIME_WAIT)の検証はタイマ定数を縮小したシミュレーション用
  ビルドが必要(`toe_internals.hpp` の `TIME_*` スケール化。TODO)
