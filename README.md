# simplebot
simplebot 高コスパポンコツロボット
![simplebot_img](https://user-images.githubusercontent.com/20347923/89854247-5be98d00-dbce-11ea-84a8-12687ee7ad05.png)
## インストール方法
1.　まず[ROS](http://wiki.ros.org/ROS/Installation)をインストールしてください。

2.　その後に、ワークスペースを構築します。   
   
```
cd   
mkdir simplebot_ws/src -p   
cd simplebot_ws/src   
catkin_init_workspace   
```

3.　構築したワークスペース内にこのプロジェクトをクローンしてください。
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
`roslaunch simplebot_description simplebot_cafe.launch`

## Licence
[MIT](https://github.com/yukimakura/simplebot/blob/master/LICENSE)


## Author
urdf及びパッケージ作成　ゆきまくら
