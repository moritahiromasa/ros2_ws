sudo apt update

ssh-keygen

cd 
git clone git@github.com:ryuichiueda/ros2_setup_scripts.git
cd ~/ros2_setup_scripts.git
./setup.bash
cd

# 必要なパッケージをインストール
sudo apt-get install git vim curl unzip imagemagick texlive-lang-japanese  texlive-latex-extra xdvik-ja evince ibus-mozc shotwell

# pip3 をインストール
sudo apt install python3-pip
pip3 install Jupyter jupyterlab pandas numpt Scipy

export PATH="$HOME/.local/bin:$PATH"  # PATHの追加

