DEMO_NAME="community" # 可选值：garage、cave、community、ruins

case "$DEMO_NAME" in
    garage|cave)
        SCENE_SIZE="large"
        ;;
    community|ruins)
        SCENE_SIZE="small"
        ;;
    *)
        SCENE_SIZE="large"
        ;;
esac

gnome-terminal --tab -t "marsim" -- bash -c "cd ~/SHIELD && source devel/setup.bash && roslaunch exploration_manager simulation_marsim.launch demo_name:=$DEMO_NAME scene_size:=$SCENE_SIZE"

sleep 5

gnome-terminal --tab -t "explore" -- bash -c "cd ~/SHIELD; source devel/setup.bash; roslaunch exploration_manager exploration_marsim.launch demo_name:=$DEMO_NAME"