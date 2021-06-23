# ros2_package_template

My ROS2 package template

## Prerequisite

```sh
brew install fd sd rename
```

## Usage

```sh
cd YOUR_ROS_WORKSPACE
git clone git@github.com:kenji-miyake/ros2_package_template.git YOUR_PACKAGE_NAME
cd YOUR_PACKAGE_NAME

# case1. If you'd like to cleanup manually
./setup.fish
mv PACKAGE_README.md README.md
rm setup.fish
rm -rf .git # When you add this to an existing repository

# case2. If you'd like to cleanup automatically
./setup.fish --clean
```
