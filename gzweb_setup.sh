#!/bin/bash
sudo apt install gazebo9 libgazebo9-dev
sudo apt install libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
source ~/.bashrc
nvm install 8
cd ~; git clone https://github.com/osrf/gzweb
cd ~/gzweb
git checkout gzweb_1.4.1
source /usr/share/gazebo/setup.sh
npm run deploy --- -m local # only load the local model
npm update minimatch@3.0.2
npm update -d
npm i ajv
npm install ajv@^5.0.0
npm audit fix
npm audit fix --force
npm run deploy --- -m local
