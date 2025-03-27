g_bot_moveit_config
==================================================

## 概要
`g_bot` configurationは，FGE(Fast Graspability Estimation)を用いてUR5eによりコンビニ商品のpick & placeを行うROSシステムの実行環境である．本パッケージは，`g_bot` configurationに対する[MoveIt](https://moveit.ai/)の設定を提供する．

## g_bot configuirationの記述ファイル
本パッケージは，`g_bot` configurationのシーンを記述した[URDFファイル](../../aist_description/scenes/urdf/g_bot_base_scene.urdf.xacro)から[MoveIt Setup Assistant](https://moveit.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html)によって生成される．シーンを構成する主な要素は，以下のとおりである．
- **g_bot**: `UR5e`アーム
- **g_bot_gripper**: 掃除機を駆動源とする吸着グリッパ
- **g_bot_camera**: g_bot_gripperに取り付けられた`Realsense d435i`カメラ
- **mounter_table**: g_botの台座
- **cabinet**: 商品棚
- **tray**: 商品回収かご

**g_bot**は，**cabinet**上の商品を**g_bot_camera**で観測してgraspabilityを検出し，**g_bot_gripper**によってピックして**tray**に入れる．

**g_bot_camera**の位置・姿勢はURDFに記述される必要があり，その値はハンド=アイキャリブレーションによって推定される．しかし，キャリブレーション前にはその値は不明なので，以下のように仮の値をコピーする．
```
$ cd catkin_ws/src/artros/aist_handeye_calibration/calib
$ cp g_bot_camera-nominal.yaml g_bot_camera.yaml
```
この後，キャリブレーションを実行すると，`g_bot_camera.yaml`の内容は推定値に置き換えられる．

`g_bot_camera.yaml`が存在すれば，以下のコマンドによりシーンを可視化できる．
```bash
$ roslaunch aist_description display_scene.launch config:=g_bot
```
また，シーンを変更するために[URDFファイル](../../aist_description/scenes/urdf/g_bot_base_scene.urdf.xacro)を編集した場合は，[MoveIt Setup Assistant](https://moveit.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html)によって本パッケージを再生成する必要がある．
```
$ roslaunch g_bot_moveit_config setup_assistant.launch
````
## ハンド=アイキャリブレーション
**g_bot_camera**の位置・姿勢を推定するためにハンド=アイキャリブレーションを行う．具体的には，カメラのベースリンク(`g_bot_camera_connector_link`)からグリッパのベースリンク(`g_bot_gripper_base_link`)への剛体変換を求める．
キャリブレーションの結果は，[aist_handeye_calibration/calib/g_bot_camera.yaml](../../aist_handeye_calibration/calib/g_bot_camera.yaml)にセーブされる．

キャリブレーションを実行するには，まず次のコマンドによってアームとグリッパのコントローラ, `MoveIt`および`rviz`を起動する．
```
$ roslaunch aist_bringup g_bot_bringup.launch scene:=g_bot_calibration [sim:=true]
```
"scene:=g_bot_calibration"を指定することにより，**cabinet**の最上段の棚にキャリブレーションマーカのモデルが表示される．"sim:=true"を指定しない場合は，実環境のこの位置（厳密でなくて良い）に実マーカを置く．実マーカは，[PDFファイル](../../aist_aruco_ros/aist_aruco_ros/markers/aruco-100_101_102_103-105x105-7.pdf)を印刷して作成する．

実環境でキャリブレーションを行う（"sim:=true"を指定しない）場合，次に，別ターミナルで以下のコマンドを打ってカメラドライバを起動する．
```
$ roslaunch aist_routines conveni.launch
```
さらに，別ターミナルで以下のコマンドによりマーカ検出ノードとキャリブレーションサーバを起動する．
```
$ roslaunch aist_handeye_calibration handeye_calibration.launch camera_name:=g_bot_camera
```
キャリブレーションサーバは，[キャリブレーション設定ファイル](../../aist_handeye_calibration/config/g_bot_camera.yaml)からマーカのIDやカメラの親リンク等のパラメータを読み込む．

最後に，別ターミナルで以下のコマンドによりキャリブレーションクライアントを起動する．
```
$ roslaunch aist_handeye_calibration run_calibration.launch config:=g_bot camera_name:=g_bot_camera
```
キャリブレーションクライアントは，[キャリブレーション設定ファイル](../../aist_handeye_calibration/config/g_bot_camera.yaml)からマーカ画像を撮影するアームのポーズ(`keypose`)等を読み込む．

キャリブレーションクライアント起動後にコマンドラインから"calib"コマンドを入力すると，アームは各`keypose`に移動してマーカを撮影し，カメラから見たその位置・姿勢を取得する．この情報と各`keypose`におけるグリッパの位置・姿勢から，全`keypose`での撮影終了後，カメラのベースリンク(`g_bot_camera_connector_link`)からグリッパのベースリンク(`g_bot_gripper_base_link`)への剛体変換を計算して[aist_handeye_calibration/calib/g_bot_camera.yaml](../../aist_handeye_calibration/calib/g_bot_camera.yaml)にセーブする．

## ピッキング
ハンド=アイキャリブレーションの実行後，得られたカメラの位置・姿勢の推定値をシーンに反映させるため，あらためてアームとグリッパのコントローラ, `MoveIt`および`rviz`を起動する．
```
$ roslaunch aist_bringup g_bot_bringup.launch
```
次に，カメラドライバ，法線方向計算のためのdepth filterおよびgraspabilityサーバを起動する．
```
$ roslaunch aist_routines conveni.launch
```
最後に，システムを操作するコマンドラインプログラムを起動する．
```
$ roslaunch aist_routines run_conveni.launch
```
このプログラムには，pick & placeを実行するROSアクションが組み込まれており，"a"コマンドによってそれを呼び出す．プログラムを構成する主要なファイルは以下のとおりである．
- [run_conveni.py](../../aist_routines/scripts/run_conveni.py): 最上位のエントリポイント
- [ConveniRoutines.py](../../aist_routines/src/aist_routines/ConveniRoutines.py): コマンドラインを介したユーザインターフェースを提供
- [ConveniPickAction.py](../../aist_routines/src/aist_routines/ConveniPickAction.py): アームを撮影ポーズ(`pick_ready`)に移動して取得した距離画像からgraspabilityを検出し，最もカメラに近いものを把持点としてピックして指定した行き先にプレースする[アクション](../../aist_msgs/action/ConveniPick.action)の実装（サーバとクライアント）を提供
- [PickOrPlaceAction.py](../../aist_routines/src/aist_routines/PickOrPlaceAction.py): アームを`approach_pose`に移動 =>`[pick|place]_pose`に移動 => グリッパでgrasp/release => `departure_pose`に移動という一連の動作を行う[アクション](../../aist_msgs/action/PickOrPlace.action)の実装を提供．[ConveniPickAction.py](../../aist_routines/src/aist_routines/ConveniPickAction.py)から呼ばれる
- [__init__.py](../../aist_routines/src/aist_routines/__init__.py): `MoveIt`を用いてアームを動かすための汎用クラス`AistRoutines`を提供

また，このプログラムは起動時に[conveni.yaml](../../aist_routines/config/conveni.yaml)からgraspability検出のためのパラメータやpick & place動作における`approach_pose`や`departure_pose`を指定するパラメータを読み込む．