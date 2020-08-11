# simplebot
simplebot 高コスパポンコツロボット

## インストール方法
1.　まず[ROS](http://wiki.ros.org/ROS/Installation)をインストールしてください。

2.　その後に、ワークスペースを構築します。   
   
```
cd   
mkdir simplebot_ws/src -p   
cd simplebot_ws/src   
catkin_init_workspace   
```

3.　構築したワークスペース内にこのプロジェクトをクローンしてください。({ユーザー名} <= 自分のアカウント名で)
```
git clone https://github.com/yukimakura/simplebot.git
```

4.　次にこのパッケージの依存関係を満たすためにこれらのコマンドを入力します。
```
cd ~/simplebot_ws
rosdep install --from-paths src --ignore-src -r -y
```

5.　あとは、`catkin_make`をしてパスを通せばsimplebotと戯れることができるでしょう！！
```
cd ~/simplebot_ws
catkin_make
source devel/setup.bash
```

オプションで`.bashrc`に登録する人は   
`echo "source ~/simplebot_ws/devel/setup.bash" >> ~/.bashrc`    
をしましょう。

## launch方法
`roslaunch simplebot_description simplebot.launch`

## Licence
[MIT](https://bitbucket.org/nitt_mix/temple_description/src/master/LICENSE)


## Author
urdf及びパッケージ作成　ゆきまくら
