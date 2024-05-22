sudo apt-get update
sudo apt-get upgrade -y

RED='\033[0;31m'
NC='\033[0m' # No Color
# current working directory
cwd=$(pwd)

echo "Installing Midnight Commander..."
sudo apt-get install mc -y
echo "Done."

echo "Installing Ranger..."
sudo apt-get install ranger -y
echo "Done."

echo "Installing fzf..."
sudo apt-get install fzf -y
echo "Done."

echo "Installing htop..."
sudo apt-get install htop -y
echo "Done."

echo "Installing thefu*k..."
sudo apt-get install thefuck -y
echo "Done."

echo "Installing tmux..."
sudo apt-get install tmux -y
echo "Done."

echo "Configuring tmux..."

echo "Installing tmux tpm..."
git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
echo "Done."

mkdir ~/.config/tmux
cp ./tmux.conf ~/.config/tmux/

~/.tmux/plugins/tpm/bin/install_plugins
~/.tmux/plugins/tpm/bin/update_plugins all
echo "Done."

echo "Installing neovim..."
echo "Installing dependencies..."
sudo apt-get install ninja-build gettext cmake unzip curl build-essential

git clone https://github.com/neovim/neovim ~/
cd ~/neovim && make CMAKE_BUILD_TYPE=RelWithDebInfo
cd build && cpack -G DEB && sudo dpkg -i nvim-linux64.deb

echo "Done."

echo "Installing nvchad..."
git clone https://github.com/NvChad/starter ~/.config/nvim
echo -e "${RED}Done! Run :MasonInstallAll after nvchad is done installing."
echo -e "${RED}Then delete ~/.config/nvim/.git"

echo "Installing ROS2 Humble..."

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "Done."

echo "alias rg="ranger "" >> ~/.bashrc

echo "Building Nereo ROS2 packages..."
cd $(cwd)/../ros2_ws
colcon build

echo "All set up!"