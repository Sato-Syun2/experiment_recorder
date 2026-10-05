# 30秒録画の保存時間計測

- 計測日: 2026-10-05（JST）
- ZED 2i: CPM 0（Lossless PNG）、HD720、30 FPS、900フレーム
- Livox Mid-360S: MCAP `zstd_fast`、`/livox/lidar`と`/livox/imu`
- 2センサーをほぼ同時に録画。時間は同じPC上の単調時計で測定。

| 対象 | ファイル容量 | 録画時間 | 保存完了まで |
|---|---:|---:|---:|
| ZED SVO2 | 878,322,282 bytes（878.3 MB / 0.818 GiB） | 900フレーム、約30秒 | ZEDコマンド開始から31.84秒。録画開始表示からSDKの`SVO saved.`まで30.28秒 |
| Livox MCAP | 89,164,420 bytes（89.2 MB / 85.0 MiB） | 31.92秒 | recorder開始から終了まで32.65秒。停止信号からbag終了まで0.314秒 |

ZEDの31.84秒にはカメラ初期化の約1.38秒が含まれるため、録画開始表示後は約30.45秒でプロセスが終了した。900フレームの取得・PNG圧縮とSVO保存は録画中に進み、完了表示後に数秒単位の追加待ちはなかった。`100%`表示と`SVO saved.`は同じ出力更新で検出されたため、そこから保存表示までの時間は個別には分離できていない。

Livoxは点群319件、IMU 6,385件を記録。圧縮は録画中に進み、停止後のMCAP確定は約0.31秒だった。Livoxドライバは停止時にexit -11となったが、rosbag2 recorderはexit 0で、`ros2 bag info`による読み取りを確認した。

録画ファイルは`recordings/zed/save_time_test_20261005_retry/`と`recordings/livox/save_time_test_20261005_retry/`、生の計測値は`timing.json`に保存。
