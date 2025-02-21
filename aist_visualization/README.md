aist_visualization
==================================================

## 概要
本パッケージは，visualizationに関するROSノードを提供する．


## mesh_generator
3D空間の中に設定された仮想スクリーンに画像を投影する際に，投影された画像に対応するスクリーン上の四辺形領域を表すメッシュを生成する．


### 入力トピック

- **/camera_info**: [sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)型．投影される画像を撮影するカメラの内部パラメータ


### 出力トピック
 - **~mesh**: [aist_visualization/TexturedMeshStamped](./msg/TexturedMeshStamped.msg)型．生成された四辺形メッシュ


### パラメータ

 - **~referece_frame**: `string`型．メッシュを構成する三角形の頂点座標を記述するフレーム名 (default: **~screen_frame**の値)
 - **~screen_frame**: `string`型．スクリーン平面の位置と向きを指定するフレーム名．仮想スクリーンはこのフレームの原点を通りz軸を法線方向とする平面となる．仮想スクリーンの3自由度が指定できれば良いので，x軸とy軸の向きや平面内でのフレーム原点の位置は重要でない (default: `"world"`)
 - **~nsteps_u**: `int`型．メッシュを三角形パッチの集合として表現する際の横方向分割数 (default: `10`)
 - **~nsteps_v**: `int`型．メッシュを三角形パッチの集合として表現する際の縦方向分割数 (default: `10`)

