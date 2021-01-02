# kondo_b3m_control
KONDO製サーボモータB3MシリーズをROS Controlで制御するためのパッケージです。

## 動作環境
このパッケージは次の環境で動作確認を行っています。
- OS
  - Ubuntu 20.04 LTS
- ROS
  - Noetic
- 対応サーボモータ
  - KONDO [B3Mシリーズ](https://kondo-robot.com/product-category/servomotor/b3m)
- シリアル通信
  - KONDO [RS485USB/シリアル変換アダプター](https://kondo-robot.com/product/02133)

## 環境構築
### パッケージをmakeする
適当なcatkinワークスペースにソースコードを取得し、catkin_makeを実行して下さい。操作例を示します。  
```
~$ mkdir -p kondo_ws/src
~$ cd kondo_ws/src
~/kondo_ws/src$ catkin_init_workspace
~/kondo_ws/src$ git clone https://github.com/nomumu/kondo_b3m_control.git
~/kondo_ws/src$ rosdep install -r -y --from-paths --ignore-src kondo_b3m_control
~/kondo_ws/src$ cd ..
~/kondo_ws$ catkin_make
~/kondo_ws$ source devel/setup.bash
```

### 通信設定
このパッケージはUSBシリアルポートを使用してB3Mと通信します。USBシリアルアダプタにはKONDOの純正品（RS485USB）を使用します。  
`config/99-kondo485.rules`を`/etc/udev/rules.d/`へ次のようにコピーしてシステムを再起動して下さい。
```
$ cd ~/kondo_ws/src/kondo_b3m_control/config
~/kondo_ws/src/kondo_b3m_control/config$ sudo cp 99-kondo485.rules /etc/udev/rules.d/
```

### 実行
このパッケージは次のように実行することができます。  
launch時にサーボトルクがON、終了時にトルクOFFになります。位置制御モードでは初期状態で0度へ動くので注意してください。  
```
$ roslaunch kondo_b3m_control pos_control.launch
```


### インストール
T.B.D.
