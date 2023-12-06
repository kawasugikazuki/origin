#!/bin/bash

# カレントディレクトリを取得
now_dir=`pwd`

# apt update && upgrade
apt update && apt upgrade -y

# OpenCVの依存ライブラリのインストール
apt install -y libatlas-base-dev \
               libhdf5-dev \
               libopenjp2-7-dev \
               libavcodec-extra58 \
               libavformat58 \
               libswscale5 \
               libgtk-3-dev \

# pip3のインストール
apt install -y python3-pip

# pigpio, picameraのインストール
apt install -y python3-pigpio python3-picamera python3-smbus

# mjpg-streamerのインストール
apt install -y cmake libjpeg62-turbo-dev
cd /home/red
git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
make
make install
cp -r ./www /usr/local/lib/mjpg-streame

# もとのディレクトリに戻る
cd $now_dir

# 起動と終了時にGPIOの初期状態を設定するスクリプトを実行するための設定をする
service_file=$(<./etc/gpio_initializer_base)
exec_start="ExecStart=`pwd`/bin/init_gpio.sh"
exec_stop="ExecStop=`pwd`/bin/init_gpio.sh"
service_file=`echo "${service_file//ExecStart/$exec_start}"`
service_file=`echo "${service_file//ExecStop/$exec_stop}"`
echo "$service_file" > /etc/systemd/system/gpio_initializer.service

# pigpiodを自動起動する設定をする
pigpiod_file=$(<./etc/pigpiod)
echo "$pigpiod_file" > /etc/systemd/system/pigpiod.service

# MainProgram.pyを自動起動する設定をする
red_service_file=$(<./etc/red2.0_main)
red_script_path=`pwd`
red_script_path="${red_script_path%/*}/explore_algorithm/MainProgram.py"
exec_path="ExecStart=${red_script_path}"
red_service_file=`echo "${red_service_file//ExecStart/$exec_path}"`
echo "$red_service_file" > /etc/systemd/system/red2.0_main.service
chmod +x "${red_script_path}"

# 上2つの設定を反映させる
systemctl daemon-reload
systemctl enable gpio_initializer.service
systemctl enable pigpiod.service
systemctl enable red2.0_main.service

# pythonパッケージのインストール
pip3 install -r ./etc/requirements.txt