sudo apt update

cd 
git clone git@github.com:ryuichiueda/ros2_setup_scripts.git
cd ~/ros2_setup_scripts.git
./setup.bash
cd

# 必要なパッケージをインストール
sudo apt-get install git vim curl unzip 

export PATH="$HOME/.local/bin:$PATH"  # PATHの追加


