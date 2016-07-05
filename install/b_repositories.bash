#!/usr/bin/env bash

echo "####################################################################################################"
echo "##### Cloning git repositories"
echo "####################################################################################################"

catkin_ws=${1:-"$HOME/catkin_ws"}

cd "${catkin_ws}/src"
if [ $? -ne 0 ]; then
	mkdir -p "${catkin_ws}/src"
	cd "${catkin_folder}/src"
	catkin_init_workspace
fi

wstool init
function clone_git_repository() {
	repository_host=${1:?'Must specify repository host'}
	repository_name=${2:?'Must specify repository name'}
	ls "${repository_name}" &> /dev/null
	if [ $? -ne 0 ]; then
		echo -e "\n"
		echo "-------------------------------------------"
		echo "==> Cloning ${repository_name} (branch: ${branch})"
		git clone "${repository_host}/${repository_name}.git"
		wstool set ${repository_name} "${repository_host}/${repository_name}.git" --git -y
	else
		echo -e "\n"
		echo "-------------------------------------------"
		echo "==> Updating ${repository_name}"
		cd ${repository_name}
		git pull
		cd ..
	fi
}

clone_git_repository "https://github.com/carlosmccosta" "laserscan_to_pointcloud"


echo -e "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Cloning git repositories finished"
echo ">>>>> For updating each git repository use: git pull"
echo ">>>>> For updating all repositories use:"
echo ">>>>> cd ${catkin_ws}/src"
echo ">>>>> wstool status"
echo ">>>>> Commit or stash modified files"
echo ">>>>> wstool update"
echo "----------------------------------------------------------------------------------------------------"
