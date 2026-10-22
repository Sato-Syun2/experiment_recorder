# 比較に用いた外部実装

PointPillarsの学習済み重みはOpenPCDet由来、KITTIのCar / Pedestrian / Cyclistの3クラス。
実行実装はtorch-pointcloud 0.0.18。重みはパッケージ登録の固定revision
`caf39bd441409bdd2997d3099cc524ac451ff06d`からダウンロードした。
当フォルダーのpointpillars.py等はwheelから抽出した参照用コピーで、変更していない。
推論は隔離環境へインストールしたパッケージを使う。
LICENSEとTHIRD_PARTY_NOTICES.mdは同じwheelの同梱文書。

- https://huggingface.co/torch-pointcloud/pointpillars.kitti.openpcdet
- https://github.com/arthurdjn/pytorch-pointcloud
- https://github.com/open-mmlab/OpenPCDet/blob/master/tools/cfgs/kitti_models/pointpillar.yaml

CPUでのボクセル化はリポジトリのcompare_pretrained_lidar.pyが担当し、
0.16 m格子、1柱32点、先着順、最大40000柱を使う。
ネットワークの重み・層構成・ボックス復号は変更していない。
